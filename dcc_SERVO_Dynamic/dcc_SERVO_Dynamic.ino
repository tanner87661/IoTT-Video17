#include <Arduino.h>
//#include <avr/power.h>

#include <NmraDcc.h>

#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_PWMServoDriver.h>
#include <TimerOne.h>
#include "Adafruit_WS2801.h"
#include "SPI.h" 
#include <Math.h>

#define SERVO 0
#define RELAY 1

#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

//#define measureMode

#ifdef measureMode
  int loopCounter = 0;
  uint32_t loopTimer = millis();
#endif

#define THROWN 0
#define CLOSED 1

#define SERVOMIN 210 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 850 // this is the 'maximum' pulse length count (out of 4096)
#define SERVOACCEL 1500 //increments / s2 used for soft start
#define SERVODECEL 1500 //increments / s2 used for soft stop

const uint16_t refreshInterval = 5000; //microseconds. Might be too low for Nano if 2 servos are processed at the same time

//#define SERVOMIN 50 // this is the 'minimum' pulse length count (out of 4096)
//#define SERVOMAX 4000 // this is the 'maximum' pulse length count (out of 4096)

//dimension considerations
// servo speed in incr/s, ranging from -1500 to 1500 (max speed of typical servo about +/- 1200 incr/s
// modeled as 16 bit integer times 20 to make room for fractions,creating a range from -30000 .. 30000
//fractions are important to keep track of actual speed in case of small accel/decel over several steps, not all resulting in a full increment

// position in incr ranging from SERVOMIN to SERVOMAX, typical range 200= 0.5ms) to 850 (= 2.07ms in 100Hz PWM)
// modeled as 16 bit with 6bit left shift and offset 150, allowing positions from 0 (150) (0.36ms) to 1023 (1173) (2.86ms)

// accel/decel in incr/s2, typical range from 1000 - 2500 (?)
// constant defined for sketch, not adjustable per channel

#define colorDark 0x00000000    //all LED's dark
#define relayON  0x0000007F     //50% blue to indicate active coil on relay
#define relayOFF 0x00000000     //same as dark for when relay is off
#define relayThrown 0x001F0000  //relay is off and on thrown position
#define relayClosed 0x00001F00  //relay is off in closed position
#define servoMinPos 0x00001F00  //servo is in minimum position
#define servoMaxPos 0x001F0000  //servo is in maximum position
#define servoMove 0x00050500    //servo is currently moving
#define aspectHalt 0x007F0000   //signal aspect color for halt (red)
#define aspectSlow 0x005F0F00   //signal aspect color yellow (may also be used for blinking)
#define aspectClear 0x00007F00  //signal aspect color green
#define level1Col 0x000F0000    //red used for level crossing blink light

#define blinkLEDInterval 800    //duration of blink cycle (half of it, it is symmetrical)

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_MCP23017 mcp;

NmraDcc Dcc;

const int DccIntrpt = 0;
const int DccDataPin = 2;
const int DccAckPin = 3; //not connected
//const int LEDDataPin = 6; //comment out if using SPI
//const int LEDClockPin = 7; //comment out if using SPI

#define numChannels 24
#define numPixels 50

Adafruit_WS2801 thisStrip = Adafruit_WS2801(numPixels);
uint32_t pixCopy[numPixels];
bool ledChg = false;

uint32_t ledBlinkTimer = millis();
uint32_t timeElapsed = 0;
bool blinkFlag = false;
float faderCtr = 0;

typedef void (*ledProcess) (void *);

typedef struct
{
  uint16_t dccAddr;
  uint8_t driveType; //obsolete for SERVO only decoder
  ledProcess ledProc; //can be substituted by a 1 byte process number
  uint8_t ledPos[5];
  uint16_t portA; //can be substituted by the position in the array for a SERVO only decoder
  uint16_t portB; //only needed for relays
  uint16_t minPos; //lower boundary in increments from SERVOMINPOS to SERVOMAXPOS
  uint16_t maxPos; //upper boundary
  uint8_t  moveConfig; //determins acceleration and deceleration of servo movements in both directions. LS nibble when increasing/CL, MS nibble when decreasing/TH
                     //nibble content: bits 0,1 define stop mode 0: hard stop 1: soft stop; 2: overshoot; 3: bounceback
                     // bit 2 defines start mode: 0: immediate start; 1: soft start
                     // bit 3 defines hesitation: 0: no hesitation; 1: hesitation active
  uint8_t  oscLambda; //4 bit value for oscillation damper, 0 - 15 interpreted as 0.5 .. 8.0  LS nibble when increasing/CL, MS nibble when decreasing/TH
  uint8_t  oscFrequency; //4 bit value for oscillation frequency, 0 - 15 interpreted as 0.5 .. 8.0 Hz  LS nibble when increasing/CL, MS nibble when decreasing/TH
  uint16_t hesitatePosition; //increments per second
  uint8_t  hesitateSpeed; //increments per second
  uint16_t targetPos; //runtime data
  uint16_t currPos; //runtime data, increments shifted left by 6, with six bits for fractions
  uint16_t moveDelayTH; //milliseconds pulse time for relays
  uint16_t moveDelayCL; //servo movement increments per second for servos. Real speed depends onf PWM frequency; CL is up, TH is down the range

  uint32_t nextMove; //runtime data milliseconds when next calculation is triggered
  float    currSpeed; //runtime data, increments per millisecond. Pos val is up, neg val is down
  uint32_t timeNull; //runtime data
  uint8_t  currMoveMode; //runtime data 0: at target; 1: accelerating; 2: linear movememnt; 3: hesitating; 4: stopping
} myServo;

void relayFct(void * thisServo);
void servoFct(void * thisServo);
void servoLinear(void * thisServo);
void aspectFct(void * thisServo);
void levelCrossingRelay(void * thisServo);
void levelCrossingServo(void * thisServo);

myServo servoArray[numChannels] = {
    {50, SERVO, &servoFct, {4,0,0,0,0}, 0, 0, SERVOMIN, SERVOMAX-150, 0x00, 0x22, 0x33, 450, 0, SERVOMIN, SERVOMIN + 1, 0, 0, 0,0,0,0},
    {51, SERVO, &servoFct, {4,0,0,0,0}, 1, 1, SERVOMIN, SERVOMAX-150, 0x00, 0x11, 0x55, 450, 0, SERVOMIN, SERVOMIN + 1, 0, 0, 0,0,0,0},
    {52, SERVO, &servoFct, {4,0,0,0,0}, 2, 2, SERVOMIN, SERVOMAX-150, 0x00, 0x11, 0x55, 500, 0, SERVOMIN, SERVOMIN + 1, 0, 0, 0,0,0,0},
    {53, SERVO, &servoFct, {4,0,0,0,0}, 3, 3, SERVOMIN, SERVOMAX-150, 0x00, 0x11, 0x33, 500, 0, SERVOMIN, SERVOMIN + 1, 0, 0, 0,0,0,0},

//    {50, SERVO, &servoFct, {4,0,0,0,0}, 0, 0, SERVOMIN, SERVOMAX-150, 0xE7, 0x22, 0x33, 450, 0, SERVOMIN, SERVOMIN + 1, 500, 550, 0,0,0,0},
//    {51, SERVO, &servoFct, {4,0,0,0,0}, 1, 1, SERVOMIN, SERVOMAX, 0x58, 0x11, 0x55, 450, 0, SERVOMIN, SERVOMIN + 1, 320, 320, 0,0,0,0},
//    {52, SERVO, &servoFct, {4,0,0,0,0}, 2, 2, SERVOMIN, SERVOMAX, 0xFE, 0x11, 0x55, 500, 0, SERVOMIN, SERVOMIN + 1, 500, 500, 0,0,0,0},
//    {53, SERVO, &servoFct, {4,0,0,0,0}, 3, 3, SERVOMIN, SERVOMAX-150, 0x00, 0x11, 0x33, 500, 0, SERVOMIN, SERVOMIN + 1, 0, 0, 0,0,0,0},

    {54, SERVO, &servoFct, {5,0,0,45,46}, 4, 4, SERVOMIN, SERVOMAX-150, 0x00, 0x11, 0x55, 0,0, SERVOMIN, SERVOMIN + 1, 0, 0, 0,0,0,0},
    {55, SERVO, &servoFct, {6,0,0,0,0}, 5, 5, SERVOMIN, SERVOMAX-150, 0x00, 0x11, 0x55, 0,0, SERVOMIN, SERVOMIN + 1, 0, 0, 0,0,0,0},
    {56, SERVO, &servoFct, {7,0,0,0,0}, 6, 6, SERVOMIN, SERVOMAX-150, 0x00, 0x11, 0x55, 0,0, SERVOMIN, SERVOMIN + 1, 0, 0, 0,0,0,0},
    {57, SERVO, &servoFct, {8,0,0,0,0}, 7, 7, SERVOMIN, SERVOMAX-150, 0x00, 0x11, 0x55, 0,0, SERVOMIN, SERVOMIN + 1, 0, 0, 0,0,0,0},

    {18, SERVO, &servoFct, {9,0,0,0,0}, 8, 8, SERVOMIN, SERVOMAX, 0x00, 0x11, 0x55, 0,0, SERVOMIN, SERVOMIN + 1, 0, 0, 0,0,0,0},
    {23, SERVO, &servoFct, {10,0,0,0,0}, 9, 9, SERVOMIN, SERVOMAX, 0x00, 0x11, 0x55, 0,0, SERVOMIN, SERVOMIN + 1, 0, 0, 0,0,0,0},
    {33, SERVO, &servoFct, {11,0,0,0,0}, 10, 10, SERVOMIN, SERVOMAX, 0x00, 0x11, 0x55, 0,0, SERVOMIN, SERVOMIN + 1, 0, 0, 0,0,0,0},
    {34, SERVO, &servoFct, {12,0,0,0,0}, 11, 11, SERVOMIN, SERVOMAX, 0x00, 0x11, 0x55, 0,0, SERVOMIN, SERVOMIN + 1, 0, 0, 0,0,0,0},

    {10800, SERVO, &aspectFct, {15,0,0,0,0}, 12, 12, SERVOMIN, SERVOMAX, 0x66, 0x11, 0x55, 0,0, SERVOMIN, SERVOMIN + 1, 400, 400, 0,0,0,0},
    {10802, SERVO, &aspectFct, {16,0,0,0,0}, 13, 13, SERVOMIN, SERVOMAX, 0x00, 0x11, 0x55, 0,0, SERVOMIN, SERVOMIN + 1, 0, 0, 0,0,0,0},
    {10804, SERVO, &aspectFct, {17,0,0,0,0}, 14, 14, SERVOMIN, SERVOMAX, 0x00, 0x11, 0x55, 0,0, SERVOMIN, SERVOMIN + 1, 0, 0, 0,0,0,0},
    {10806, SERVO, &aspectFct, {18,0,0,0,0}, 15, 15, SERVOMIN, SERVOMAX, 0x00, 0x11, 0x55, 0,0, SERVOMIN, SERVOMIN + 1, 0, 0, 0,0,0,0},

    {61, RELAY, &relayFct, {21,22,23,0,0}, 0, 1, THROWN, CLOSED, 0,0,0,0,0, CLOSED, THROWN, 2000, 2000, 0, 0, 0, 0},
    {62, RELAY, &relayFct, {24,25,26,0,0}, 2, 3, THROWN, CLOSED, 0,0,0,0,0, CLOSED, THROWN, 200, 200, 0, 0, 0, 0},
    {63, RELAY, &relayFct, {27,28,29,0,0}, 4, 5, THROWN, CLOSED, 0,0,0,0,0, CLOSED, THROWN, 200, 200, 0, 0, 0, 0},
    {64, RELAY, &relayFct, {30,31,32,0,0}, 6, 7, THROWN, CLOSED, 0,0,0,0,0, CLOSED, THROWN, 200, 200, 0, 0, 0, 0},

    {65, RELAY, &relayFct, {33,34,35,0,0}, 8, 9, THROWN, CLOSED, 0,0,0,0,0, CLOSED, THROWN, 200, 200, 0, 0, 0, 0},
    {66, RELAY, &relayFct, {36,37,38,0,0}, 10, 11, THROWN, CLOSED, 0,0,0,0,0, CLOSED, THROWN, 200, 200, 0, 0, 0, 0},
    {67, RELAY, &relayFct, {39,40,41,0,0}, 12, 13, THROWN, CLOSED, 0,0,0,0,0, CLOSED, THROWN, 4000, 1000, 0, 0, 0, 0},
    {68, RELAY, &levelCrossingRelay, {42,43,44,47,48}, 14, 15, THROWN, CLOSED, 0,0,0,0,0, CLOSED, THROWN, 200, 200, 0, 0, 0, 0},

};

uint32_t Color(byte r, byte g, byte b)
{
  //Take the lowest 5 bits of each value and append them end to end
  uint32_t retVal = (((uint32_t)r<<16) + ((uint32_t)g<<8) + (uint32_t)b);
  return retVal; 
}

uint32_t blinkColor(uint32_t posCol, uint32_t negCol=0, bool inPhase=true)
{
  if (blinkFlag ^ inPhase)
    return negCol;
  else
    return posCol;
}

uint32_t fadeColor(uint32_t posCol, uint32_t negCol=0, bool inPhase=true)
{
  float dimFactHi, dimFactLo;
  uint8_t faderExp = 8;
  if (negCol != 0)
    faderExp = 7;
  if (inPhase)
  {
    dimFactHi = (pow (2, (faderExp*faderCtr)) - 1)/255;
    dimFactLo = (pow (2, (faderExp*(1-faderCtr))) - 1)/255;
  }
  else
  {
    dimFactLo = (pow (2, (faderExp*faderCtr)) - 1)/255;
    dimFactHi = (pow (2, (faderExp*(1-faderCtr))) - 1)/255;
  }
  
  uint8_t rPos, gPos, bPos;
  rPos = round(((posCol & 0x00FF0000) >> 16) * dimFactHi) & 0xFF;
  gPos = round(((posCol & 0x0000FF00) >> 8) * dimFactHi) & 0xFF;
  bPos = round((posCol & 0x000000FF) * dimFactHi) & 0xFF;
  if (negCol != 0)
  {
    rPos = rPos + (round(((negCol & 0x00FF0000) >> 16) * dimFactLo) & 0xFF);
    gPos = gPos + (round(((negCol & 0x0000FF00) >> 8) * dimFactLo) & 0xFF);
    bPos = bPos + (round((negCol & 0x000000FF) * dimFactLo) & 0xFF);
  }
  return Color(rPos, gPos, bPos);
}

void setLED(uint16_t ledNr, uint32_t newCol)
{
  if (pixCopy[ledNr] != newCol)
  {
    pixCopy[ledNr] = newCol;
    thisStrip.setPixelColor(ledNr, newCol);
    ledChg = true;
  }
}

uint16_t getServoPos(myServo * thisServo)
{
  return (thisServo->currPos >> 6) + 150;
}

void setServoPos(myServo * thisServo, uint16_t value) //150 <= value <= 1173
{
  value = min(1173, max(value, 150));
  thisServo->currPos = (value-150)<<6;
}

void relayFct(void * thisServo)  //this strip has color sequence b,g,r
{
  myServo * currServo = (myServo*)thisServo;
  uint32_t pos1Col=colorDark, pos2Col=relayOFF, pos3Col=colorDark;
  if (currServo->targetPos != currServo->currPos) //->currPos)
    pos2Col = relayON;
  else
    if (currServo->currPos == THROWN)
      pos1Col = relayThrown;
    else
      pos3Col = relayClosed;
  if (currServo->ledPos[0] > 0)
      setLED(currServo->ledPos[0]-1, pos1Col);
  if (currServo->ledPos[1] > 0)
      setLED(currServo->ledPos[1]-1, pos2Col);
  if (currServo->ledPos[2] > 0)
      setLED(currServo->ledPos[2]-1, pos3Col);
}

void servoFct(void * thisServo)
{
  myServo * currServo = (myServo*)thisServo;
  uint32_t newCol;
  if (currServo->targetPos != getServoPos(currServo))
    newCol = blinkColor(servoMove);
  else
    if (getServoPos(currServo) == SERVOMIN)
      newCol = servoMinPos;
    else
      newCol = servoMaxPos;
  if (currServo->ledPos[0] > 0)
    setLED(currServo->ledPos[0]-1, newCol);
}

void aspectFct(void * thisServo)
{
  myServo * currServo = (myServo*)thisServo;
  uint32_t newCol;
  int currAspect = round((getServoPos(currServo) - currServo->minPos) / ((currServo->maxPos - currServo->minPos) / 8));
  switch(currAspect)
  {
    case 0: newCol = aspectHalt; break; 
    case 1:;
    case 2: newCol = aspectSlow; break;
    case 3:; 
    case 4:; 
    case 5: newCol = fadeColor(aspectSlow); break; 
    default: newCol = aspectClear; break;
  }
  if (currServo->ledPos[0] > 0)
    setLED(currServo->ledPos[0]-1, newCol);
}

void levelCrossingRelay(void * thisServo)
{
  relayFct(thisServo);
  myServo * currServo = (myServo*)thisServo;
  if ((getServoPos(currServo) == THROWN) || (getServoPos(currServo) != currServo->targetPos))
  {
    if (currServo->ledPos[3] > 0)
      setLED(currServo->ledPos[3]-1, fadeColor(level1Col));
    if (currServo->ledPos[4] > 0)
      setLED(currServo->ledPos[4]-1, fadeColor(level1Col, 0, false));
  }
  else
  {
    if (currServo->ledPos[3] > 0)
      setLED(currServo->ledPos[3]-1, colorDark);
    if (currServo->ledPos[4] > 0)
      setLED(currServo->ledPos[4]-1, colorDark);
  }
}

void levelCrossingServo(void * thisServo)
{
  servoFct(thisServo);
  myServo * currServo = (myServo*)thisServo;
  if ((getServoPos(currServo) == SERVOMAX) || (getServoPos(currServo) != currServo->targetPos))
  {
    if (currServo->ledPos[3] > 0)
      setLED(currServo->ledPos[3]-1, blinkColor(level1Col));
    if (currServo->ledPos[4] > 0)
      setLED(currServo->ledPos[4]-1, blinkColor(level1Col, 0, false));
  }
  else
  {
    if (currServo->ledPos[3] > 0)
      setLED(currServo->ledPos[3]-1, colorDark);
    if (currServo->ledPos[4] > 0)
      setLED(currServo->ledPos[4]-1, colorDark);
  }
}

// This function is called whenever a normal DCC Turnout Packet is received and we're in Output Addressing Mode
void notifyDccAccTurnoutOutput(uint16_t Addr, uint8_t Direction, uint8_t OutputPower)
{
  Serial.print("notifyDccAccTurnoutOutput: ");
  Serial.print(Addr, DEC);
  Serial.print(',');
  Serial.print(Direction, DEC);
  Serial.print(',');
  Serial.println(OutputPower, HEX);

  for (int i = 0; i < numChannels; i++)
  {
    if (Addr == servoArray[i].dccAddr)
    {
      if (Direction == 0)
        servoArray[i].targetPos = (servoArray[i].driveType == SERVO) ? servoArray[i].minPos : THROWN;
      else
        servoArray[i].targetPos = (servoArray[i].driveType == SERVO) ? servoArray[i].maxPos : CLOSED;
      if (servoArray[i].driveType == RELAY)
        setServoPos(&servoArray[i], (servoArray[i].targetPos == CLOSED) ? THROWN : CLOSED);
      servoArray[i].currMoveMode = 0; //new position command, stop any current movement
      servoArray[i].nextMove = micros(); //initialize timer
    }
  }
}

// This function is called whenever a DCC Signal Aspect Packet is received
void notifyDccSigOutputState(uint16_t Addr, uint8_t State)
{
  Addr += 10000;
  Serial.print("notifyDccSigOutputState: ");
  Serial.print(Addr, DEC);
  Serial.print(',');
  Serial.println(State, HEX);
  for (int i = 0; i < numChannels; i++)
    if ((Addr == servoArray[i].dccAddr) && (servoArray[i].driveType == SERVO))
    {
      servoArray[i].targetPos = min(servoArray[i].maxPos, servoArray[i].minPos + (State * ((servoArray[i].maxPos - servoArray[i].minPos) / 8)));
      servoArray[i].currMoveMode = 0; //new position command, stop any current movement
      servoArray[i].nextMove = micros(); //initialize timer
    }
}

void processServo(myServo *thisServo)
{
//  uint32_t thisTime = micros();
  if (thisServo->currMoveMode == 0) //linear movement mode
  {
    uint16_t currPos = getServoPos(thisServo); //incr
    if (thisServo->targetPos != currPos)
    {
      if ((thisServo->nextMove < micros()))
      {
        //analyze the current status for further decision making
        bool moveUp = thisServo->targetPos > getServoPos(thisServo);
        uint8_t adjMode = moveUp ? (thisServo->moveConfig & 0x0F) : ((thisServo->moveConfig & 0xF0) >> 4);
        float linSpeed = moveUp ? (double) thisServo->moveDelayCL : (double)(-1) * thisServo->moveDelayTH; //set linSpeed to incr/s
        bool correctDir = (sgn(thisServo->currSpeed) == sgn(linSpeed)) || (thisServo->currSpeed == 0);
        bool softStop = ((adjMode & 0x03) == 1); //soft stop
        bool softStart = (adjMode & 04); //overshoot or soft stop
        bool beforeHesitate = adjMode & 0x08 ? (moveUp ? currPos < thisServo->hesitatePosition : currPos > thisServo->hesitatePosition) : false; 
        uint16_t moveEndPoint = beforeHesitate ? thisServo->hesitatePosition : thisServo->targetPos; //end point or hesitate
        float moveEndSpeed = (beforeHesitate ? (sgn(linSpeed) * thisServo->hesitateSpeed) : (adjMode & 0x02 ? (linSpeed) : (adjMode == 0 ? (linSpeed) : (0)))); //speed when arriving at end point, incr/s
        int32_t breakDistance = round(sq(linSpeed - moveEndSpeed) / (2 * (int32_t)SERVODECEL)); //s = v2 /2a
        uint16_t breakPoint = moveEndPoint - (sgn(linSpeed) * breakDistance);
        bool beforeBreakPoint = ((int)(breakPoint - currPos) * (int)sgn(linSpeed)) > 0;

        //calculate dynamic data for step duration and width 
        uint32_t stepDelay = thisServo->currSpeed == 0 ? refreshInterval : round(1000000 / abs(thisServo->currSpeed)); //calculating the duration of 1 step in micros/incr
        float stepFactor = thisServo->currSpeed == 0 ? 1 : 1 + (refreshInterval / stepDelay);  //calculate how many steps to take assuming 5ms cycle time

        if (correctDir)
        {
          if (linSpeed != 0)
          {
            if (beforeBreakPoint)
            {
              if (abs(thisServo->currSpeed) < abs(linSpeed))
              {
                if (softStart)
                //accelerate
                  thisServo->currSpeed += round((sgn(linSpeed) * stepFactor * (stepDelay * (int32_t)SERVOACCEL) / 1000000)); //v = v0 + at
                else
                  thisServo->currSpeed = linSpeed;
                if (abs(thisServo->currSpeed) > abs(linSpeed))
                  thisServo->currSpeed = linSpeed;
              }
              //keep moving
              thisServo->currPos += round(stepFactor * 64 * sgn(linSpeed)); //set the position
              thisServo->nextMove += round(stepFactor * stepDelay); //set the delay time
            }
            else
            {
              if (getServoPos(thisServo) == thisServo->targetPos) //final position
              {
                setServoPos(thisServo, thisServo->targetPos);
                thisServo->nextMove += refreshInterval; //1ms wait                
              }
              else
              {
              //accel-/decel to moveEndSpeed
                bool endAccel = false;
                if (abs(moveEndSpeed) > abs(thisServo->currSpeed))
                //accelerate
                {
                  //add comparison to target speed at position, adjust acceleration if needed
                  thisServo->currSpeed += round((sgn(linSpeed) * stepFactor * (stepDelay * (int32_t)SERVOACCEL) / 1000000)); //v = v0 + at
                  if (abs(moveEndSpeed) <= abs(thisServo->currSpeed))
                    thisServo->currSpeed = moveEndSpeed;
                  endAccel = thisServo->currSpeed == moveEndSpeed;
                }
                else
                //decelerate
                {
                  //add comparison to target speed at position, adjust deceleration if needed
                  int sgnSpeed = sgn(thisServo->currSpeed);
                  thisServo->currSpeed -= round((sgn(linSpeed) * stepFactor * (stepDelay * (int32_t)SERVODECEL) / 1000000)); //v = v0 - at
                  if (sgn(thisServo->currSpeed) != sgnSpeed)
                    thisServo->currSpeed = 0;
                  endAccel = thisServo->currSpeed == 0;
                }
                //advance to moveEndPoint
                if (endAccel)
                {
                  setServoPos(thisServo, moveEndPoint);
                  thisServo->nextMove += refreshInterval; //1ms wait                
                }
                else
                {
                  thisServo->currPos += round(stepFactor * 64 * sgn(linSpeed)); //set the position
                  thisServo->nextMove += round(stepFactor * stepDelay); //set the delay time
                }
              }
            }
            //when there, execute move end
            bool currMoveUp = thisServo->targetPos > getServoPos(thisServo);
            if ((getServoPos(thisServo) == thisServo->targetPos) || (moveUp != currMoveUp)) //overshooting when speed > 1 incr per cycle
              if ((adjMode & 0x03) > 1) //bounce back or overshooting
              {
                setServoPos(thisServo, thisServo->targetPos);
                thisServo->currMoveMode = 1; //enter oscillation phase
                thisServo->timeNull = micros();
              }
              else
                thisServo->currSpeed = 0;
          }
          else
          {
            setServoPos(thisServo, thisServo->targetPos);
            thisServo->nextMove = micros();
          }
        }
        else
        {
          if (softStop)
          {
            //decelerate and change direction. Speed is incr/s, accel/decel is incr/s2, time intervl is 1ms
            int sgnSpeed = sgn(thisServo->currSpeed);
            thisServo->currSpeed += round((sgn(linSpeed) * stepFactor * (stepDelay * (int32_t)SERVODECEL) / 1000000)); //v = v0 - at
            if (sgn(thisServo->currSpeed) != sgnSpeed)
              thisServo->currSpeed = 0;
          }
          else
            thisServo->currSpeed = 0;
          if (thisServo->currSpeed != 0)
          {
            thisServo->currPos += round(stepFactor * 64 * sgn(linSpeed)); //set the position
            thisServo->nextMove += round(stepFactor * stepDelay); //set the delay time
          }
          else
            thisServo->nextMove += refreshInterval; //standard 1ms wait
        }
//        printPosition(thisServo->currSpeed, getServoPos(thisServo));
        pwm.setPWM(thisServo->portA, 0, getServoPos(thisServo));
      }
    }
  }
  else //oscillator mode
  {
    bool moveUp = getServoPos(thisServo) == thisServo->maxPos;
    uint8_t adjMode = moveUp ? (thisServo->moveConfig & 0x0F) : ((thisServo->moveConfig & 0xF0) >> 4);
    float thisLambda = (float)((moveUp ? (thisServo->oscLambda & 0x0F) : ((thisServo->oscLambda & 0xF0) >> 4)) + 1) / 2;
    float thisFreq = (float)((moveUp ? (thisServo->oscFrequency & 0x0F) : ((thisServo->oscFrequency & 0xF0) >> 4)) + 1) / 2;
    float timePassed2 = (float)(micros() - thisServo->timeNull) / 1000;
    float timePassed = timePassed2 / 1000;
    //calculate y(t)
    float origAmpl = thisServo->currSpeed / (TWO_PI * thisFreq);
    float currAmpl = origAmpl  * exp(thisLambda * timePassed * -1);
    float currVal = round(currAmpl * sin(TWO_PI * thisFreq * timePassed));
    uint16_t pwmVal = getServoPos(thisServo); //moveUp ? (uint16_t)SERVOMAX : (uint16_t)SERVOMIN;

    if ((currAmpl / origAmpl) > 0.1) //stop oscillator if amplitude < 10% of original value
    {
      //PWM to targetPos
      if ((adjMode & 0x03) == 2) //bounce back
        //PWM to targetPos + newAmpl
        pwmVal = pwmVal + currVal;
      else //3, overshoot
        //PWM to targetPos - abs(newAmpl)
        pwmVal = moveUp ? pwmVal - abs(currVal) : pwmVal + abs(currVal);
//      printPosition(thisServo->currSpeed, pwmVal);
      pwm.setPWM(thisServo->portA, 0, pwmVal);
    } 
    else //done, back to linear mode
    {
      pwm.setPWM(thisServo->portA, 0, getServoPos(thisServo));
      thisServo->currMoveMode = 0; 
      thisServo->currSpeed = 0; 
    }
    thisServo->nextMove += refreshInterval; //standard 1ms wait
  }
}

void printPosition(float speedVal, uint16_t posVal)
{
  Serial.print(micros());
  Serial.print(", ");
  Serial.print(speedVal);
  Serial.print(", ");
  Serial.println(posVal);
}

void processServoSimple(myServo *thisServo)
{
  if (thisServo->targetPos != getServoPos(thisServo))
  {
    if ((thisServo->nextMove < micros()))
    {
      bool moveUp = thisServo->targetPos > getServoPos(thisServo);
      uint16_t stepSpeed = moveUp ? thisServo->moveDelayCL : thisServo->moveDelayTH; //incr/s 
      if (stepSpeed > 0) //valid speed settings
      {
        uint32_t stepDelay = stepSpeed > 0 ? round(1000000 / stepSpeed) : 0; //calculating the duration of 1 step
        float stepFactor = 1 + (refreshInterval / stepDelay);  //calculate how many steps to take assuming 5ms cycle time
        thisServo->currPos += round(stepFactor * 64 * (moveUp ? 1 : (-1))); //set the position
        thisServo->nextMove += round(stepFactor * stepDelay); //set the delay time
        bool nextDir = thisServo->targetPos > getServoPos(thisServo);
        if (moveUp != nextDir) //reached targetPos, so break the movement
        {
          setServoPos(thisServo, thisServo->targetPos);
          thisServo->nextMove = micros()+ refreshInterval;
        }
      }
      else //no settings, go with maximum speed to the target
      {
        setServoPos(thisServo, thisServo->targetPos);
        thisServo->nextMove = micros()+ refreshInterval;
      }
//      printPosition(thisServo->currSpeed, getServoPos(thisServo));
      pwm.setPWM(thisServo->portA, 0, getServoPos(thisServo)); //setting the PWM output
    }
  }
}

void processRelay(myServo *thisServo)
{
  switch (thisServo->currPos)
  {
  case 0:;
  case 1:
  {
    mcp.digitalWrite(thisServo->portA, thisServo->targetPos == CLOSED);
    mcp.digitalWrite(thisServo->portB, thisServo->targetPos == THROWN);
    thisServo->nextMove = millis() + ((thisServo->targetPos == CLOSED) ? thisServo->moveDelayCL : thisServo->moveDelayTH);
    thisServo->currPos = 0xFFFF;
    break;
  }
  case 0xFFFF: //moveDelayXX is milliseconds to wait before switching relay off
    if ((thisServo->nextMove < millis()) && ((thisServo->moveDelayTH > 0) || (thisServo->moveDelayCL > 0)))
    {
      thisServo->currPos = thisServo->targetPos;
      thisServo->currMoveMode = 0; //just to be sure, should be initialized like this
      mcp.digitalWrite(thisServo->portA, 1);
      mcp.digitalWrite(thisServo->portB, 1);
    }
    break;
  }
}

void processLED(myServo *thisServo)
{
  if (thisServo->ledProc != NULL)
    thisServo->ledProc(thisServo);
}

void processLocations()
{
  for (int i = 0; i < numChannels; i++)
  {
    if ((servoArray[i].targetPos != getServoPos(&servoArray[i])) || (servoArray[i].currMoveMode != 0))
    {
      switch (servoArray[i].driveType)
      {
      case SERVO:
        if (servoArray[i].moveConfig > 0)
          processServo(&servoArray[i]);
        else
          processServoSimple(&servoArray[i]);
        break;
      case RELAY:
        processRelay(&servoArray[i]);
        break;
      }
    }
    processLED(&servoArray[i]);
  }
}

void setup()
{
//  clock_prescale_set(clock_div_1); //make this a 32MHz machine????
  
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Configure the DCC CV Programing ACK pin for an output
  pinMode(DccAckPin, OUTPUT);
  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up
  Dcc.pin(DccIntrpt, DccDataPin, 1);

  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init(MAN_ID_DIY, 10, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE, 0);

  Serial.println("Init Done");

  for (int i = 0; i < numChannels; i++)
  {
    if (servoArray[i].driveType == SERVO)
      servoArray[i].currPos = (servoArray[i].currPos - 150)<<6;
  }  

  pwm.begin();

  pwm.setPWMFreq(92); // Analog servos run at ~50 - 200 Hz updates. 92 is the closest I get to 100Hz measured

  mcp.begin(); // use default address 0
  for (int i = 0; i < 16; i++)
  {
    mcp.pinMode(i, OUTPUT);
    mcp.digitalWrite(i, 1);
  }
  
  // Start up the LED counter
  thisStrip.begin();

  // Update the strip, to start they are all 'off'

  for (int i = 0; i < numPixels; i++)
  {
    thisStrip.setPixelColor(i, 0);
    pixCopy[i] = 0;
  }
  ledChg = true;
}

void loop()
{
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();
  processLocations();
  
  timeElapsed = millis() - ledBlinkTimer;
  if (timeElapsed > blinkLEDInterval)
  {
    blinkFlag = !blinkFlag; //flip flop flag
    ledBlinkTimer += blinkLEDInterval; 
    timeElapsed -= blinkLEDInterval;
  }
  if (blinkFlag)
    faderCtr = (float)timeElapsed/(float)blinkLEDInterval;  //positive slope ramp from 0 to 1
  else
    faderCtr = 1 - ((float)timeElapsed/(float)blinkLEDInterval); //negative slope ramp from 1 to 0

  if (ledChg)
  {
    thisStrip.show();
    ledChg = false;
  }

#ifdef measureMode
  if (loopTimer < millis())
  {
    Serial.println(loopCounter);
    loopCounter = 0;
    loopTimer += 1000;
  }
  loopCounter++;
#endif  
}
