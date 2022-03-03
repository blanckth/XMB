#include <Servo.h>

// Authur : Salar Muhammadi BS.Student in COMPUTER SCIENCE at RAZI University
//
// PROJECT (xmb)
//
// METHODS INIT
int AbsVal();
void ChangePos();
void SyncChPos();
void ArmCenterPos();
void LaserFadeOnOff();
void SRFScan();
void HelloMotion();
void ManMoveXY();
//
// VARIABLES
//
// BUTTONS PIN
const byte PIN_BUTT_A = 2; // #1 Pin For Buttun A status pressed
const byte PIN_BUTT_B = 3; // #2 Pin For Buttun B status pressed
const byte PIN_BUTT_C = 4; // #3 Pin For Buttun C status pressed
const byte PIN_BUTT_D = 5; // #4 Pin For Buttun D status pressed
const byte PIN_BUTT_E = 6; // #4 Pin For Buttun E status pressed
const byte PIN_BUTT_F = 7; // #4 Pin For Buttun F status pressed
const byte PIN_BUTT_K = 8; // #4 Pin For Buttun K status pressed
byte Buttons[] = {PIN_BUTT_A, PIN_BUTT_B, PIN_BUTT_C, PIN_BUTT_D, PIN_BUTT_E, PIN_BUTT_F, PIN_BUTT_K};
//
// SRF PIN
const byte PIN_TRIG = 10;
const byte PIN_ECHO = 9;
//
// // LASER PIN
const byte PIN_LASER = 11;
int LaserVal = 0;
//
// X values will be read from ANALOG pin A0 and Y from ANALOG pin A1 ( JOYSTICK )
const byte PIN_ANALOG_X = A0; // ANALOG INPUT JUST THE WAY
const byte PIN_ANALOG_Y = A1; // \\ // \\ // \\ // / /////
//
// ROTATION SERVO
const byte rotateSrvP = A5; // Rotation Servo PIN
Servo rotateSrv;            // // Servo ( LOWEST HEIGHT SERVO FOR ROTATING AROUND )
// ANGLE DEGREE variable with INITIAL VALUE (THE MAX HERE FOR SERVO IS 180 THEN WE PEAKED 90 FOR CENTERIZING calibration )
byte RSval = 90;
//
// BACK AND FORWARD SERVO
const byte BFSrvP = A4; // Back and Forward Servo PIN
Servo BFSrv;            // // Servo
byte BFSval = 90;       // Value
//
// UP AND DOWN SERVO
const byte UDSrvP = A3; //  PIN
Servo UDSrv;            // // Servo
byte UDSval = 90;       // Value
//
// GRIPPER SERVO
const byte GSrvP = A2; //  PIN
Servo GSrv;            // // Servo
byte GSval = 90;       // Value
//
// BUZZER
const byte BzrP = 9; //  PIN
int BuzFreq = 666;    // Value Frequency
//
// DC MOTOR Trans Switch
const byte Dcmotort = 12; // PIN
// BUILTIN LED
byte led = LED_BUILTIN; // PIN
// METHODS
// // //
// ABSOLUTE VALUE METHOD
int AbsVal(int vAl)
{
  if (vAl < 0)
  {
    return (vAl * -1);
  }
  else
  {
    return vAl;
  }
}
//
// SRF SCAN MASURMENTS METHOD
//
void SRFScan()
{
  //SRF //////
  // PULSE
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  // READ ECHO
  const unsigned long duration = pulseIn(PIN_ECHO, HIGH);
  int distance = duration / 29 / 2;
  if (duration == 0)
  {
    Serial.println("Warning: no pulse from sensor");
  }
  else
  {
    Serial.print("distance to nearest object:");
    Serial.println(distance);
    Serial.println(" cm");
  }
}
//
// HELLO MOTION ARM WAVES
void HelloMotion()
{
  // BEGINNING OF MOTION
  for (int x = 0; x < 2; x++)
  {
    SyncChPos(44, 90, 90, 166, 15);
    SyncChPos(111, 90, 90, 88, 15);
  }
  for (int x = 0; x < 2; x++)
  {
    SyncChPos(44, 111, 55, 166, 15);
    SyncChPos(111, 111, 55, 88, 15);
  }
  for (int x = 0; x < 2; x++)
  {
    SyncChPos(44, 111, 22, 166, 15);
    SyncChPos(111, 111, 22, 88, 15);
  }
  SyncChPos(66, 88, 55, 180, 8);
  SyncChPos(111, 99, 44, 90, 11);
  SyncChPos(66, 111, 33, 180, 8);
}
//
// MANUAL MOVE Servo By XY axis METHOD
// xy ==> x = 1 and y = 0
void ManMoveXY(Servo Srv, int xy)
{
  int XYinp;
  int LimitLow;
  int LimitHigh;
  if (xy)
  {
    XYinp = analogRead(PIN_ANALOG_X);
    LimitLow = 499;
    LimitHigh = 522;
  }
  else
  {
    XYinp = analogRead(PIN_ANALOG_Y);
    LimitLow = 499;
    LimitHigh = 512;
  }
  int Sval = Srv.read();
  int valC = 0;
  if (XYinp < LimitLow)
  {
    XYinp = AbsVal(XYinp - LimitLow - 1);
    valC = map(XYinp, 0, LimitLow - 1, 1, 3);
    if ((Sval + valC) < 180)
    {
      Sval += valC;
      Srv.write(Sval);
      delay(4);
    }
    else
    {
      tone(BzrP, BuzFreq);
    }
  }
  else if (XYinp > LimitHigh)
  {
    valC = map(XYinp, LimitHigh + 1, 1023, 1, 3);
    if ((Sval - valC) > 0)
    {
      Sval -= valC;
      Srv.write(Sval);
      delay(4);
    }
    else
    {
      tone(BzrP, BuzFreq);
    }
  }
}
//LASER FADE ON OFF METHOD
//
void LaserFadeOnOff()
{
  if (LaserVal == 0)
  {
    for (LaserVal = 0; LaserVal <= 255; LaserVal++)
    {
      analogWrite(PIN_LASER, LaserVal);
      delay(11);
    }
  }
  else
  {
    for (LaserVal = 255; LaserVal >= 0; LaserVal--)
    {
      analogWrite(PIN_LASER, LaserVal);
      delay(11);
    }
    LaserVal = 0;
  }
}
// SYNC CHANGE POSITION FOR 4 Servo Of MeArm with variable SPEED 1-100
void SyncChPos(int RSP, int BFSP, int UDSP, int GSP, int spd)
{
  // CURRENT POSITION
  int RSCP = rotateSrv.read();
  int BFSCP = BFSrv.read();
  int UDSCP = UDSrv.read();
  int GSCP = GSrv.read();
  // DIFFERENCES
  int maxDiff = 0; // Max Difference
  int RSD = RSP - RSCP;
  if (AbsVal(RSD) > maxDiff)
    maxDiff = AbsVal(RSD);
  int BFSD = BFSP - BFSCP;
  if (AbsVal(BFSD) > maxDiff)
    maxDiff = AbsVal(BFSD);
  int UDSD = UDSP - UDSCP;
  if (AbsVal(UDSD) > maxDiff)
    maxDiff = AbsVal(UDSD);
  int GSD = GSP - GSCP;
  if (AbsVal(GSD) > maxDiff)
    maxDiff = AbsVal(GSD);
  // IF has DIFF
  if (maxDiff > 0)
  {
    for (int x = 0; x < maxDiff; x++)
    {
      if ((x + 1) <= AbsVal(RSD))
      {
        if (RSD > 0)
        {
          rotateSrv.write(RSCP + x + 1);
        }
        else
        {
          rotateSrv.write(RSCP - x + 1);
        }
      }
      if ((x + 1) <= AbsVal(BFSD))
      {
        if (BFSD > 0)
        {
          BFSrv.write(BFSCP + x + 1);
        }
        else
        {
          BFSrv.write(BFSCP - x + 1);
        }
      }
      if ((x + 1) <= AbsVal(UDSD))
      {
        if (UDSD > 0)
        {
          UDSrv.write(UDSCP + x + 1);
        }
        else
        {
          UDSrv.write(UDSCP - x + 1);
        }
      }
      if ((x + 1) <= AbsVal(GSD))
      {
        if (GSD > 0)
        {
          GSrv.write(GSCP + x + 1);
        }
        else
        {
          GSrv.write(GSCP - x + 1);
        }
      }
      delay(100 / spd);
    }
  }
}
void ChangePos(Servo mSrv, int POS, int spd)
{
  int currPos = mSrv.read();
  int DiffPos;
  if (POS > currPos)
  {
    DiffPos = POS - currPos;
    for (int x = 0; x < DiffPos; x++)
    {
      mSrv.write(currPos + x + 1);
      delay(100 / spd);
    }
  }
  else if (POS != currPos)
  {
    DiffPos = currPos - POS;
    for (int x = 0; x < DiffPos; x++)
    {
      mSrv.write(currPos - x + 1);
      delay(100 / spd);
    }
  }
}
void ArmCenterPos()
{
  SyncChPos(90, 90, 90, 90, 8);
}

void setup()
{
  // put your setup code here, to run once:
  //
  // SRF
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  //
  // Bond First Conection wire to Buzzer FROM OUTPUT Pin
  pinMode(BzrP, OUTPUT);
  pinMode(Dcmotort, OUTPUT);
  digitalWrite(Dcmotort, LOW);
  // MAKE SOME SOUND IN THE RANGE 0 (OFF) - To 255 Divided, the 5 volts Comunication
  //analogWrite(BzrP,222);
  tone(BzrP, BuzFreq);
  // Bond First Conection to Buttons
  for (int x = 0; x < 7; x++)
  {
    pinMode(Buttons[x], INPUT);
    digitalWrite(Buttons[x], HIGH); // TURN ON
  }
  // ATTACH THE SERVO LIBRARY INSTANCE TO PINs
  rotateSrv.attach(rotateSrvP, 333, 2444);
  // SOME LIKE
  BFSrv.attach(BFSrvP);
  // SOME LIKE
  UDSrv.attach(UDSrvP);
  // SOME LIKE
  GSrv.attach(GSrvP);
  // RUN THE MOTOR AND GO TO INITIALIZE 90 DEGREE
  ArmCenterPos();
  // CONFIRM AN BLOCK SIZE FOR COMUNICATION MONITORING
  Serial.begin(9600); // SERIAL BEGINS ...
  delay(666);         // 1000 = 1 Second ( THIS DELAY WILL BE THE LAST OF FIRST BEEP OF BUZZER ) :)
  noTone(BzrP);       // BUZZ OFF
}
//
void loop()
{
  // put your main code here, to run repeatedly:

  if (!digitalRead(PIN_BUTT_K))
  {
    tone(BzrP,222);
    delay(66);
    noTone(BzrP);
  }
  if (!digitalRead(PIN_BUTT_D))
  {
    HelloMotion();
    // digitalWrite(Dcmotort, HIGH);
  }
  else
  {
    // digitalWrite(Dcmotort, LOW);
  }
  if (!digitalRead(PIN_BUTT_C))
  {
    ArmCenterPos();
  }
  if (!digitalRead(PIN_BUTT_B))
  {
    SyncChPos(66, 66, 44, 166, 8);
    // SRFScan();
  }
  if (!digitalRead(PIN_BUTT_A))
  {
    ManMoveXY(GSrv, 1);
    ManMoveXY(BFSrv, 0);
  }
  else
  {
    ManMoveXY(rotateSrv, 1);
    ManMoveXY(UDSrv, 0);
  }
  // Some delay to clearly observe your values on serial monitor.
  delay(6);
  noTone(BzrP);
}
