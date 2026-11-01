/*
 * TURBO Line Follower v3.4
 * Base: T.Baatarkhuu v2.1 | Updater: B. Batmend
 *
 * s/r start/stop
 * 1-5 no-reverse | 6-9,0 pivot | F G H pivot faster | B C E pivot + slowdown
 * l/m quick speed
 * p D h n a  Kp/Kd/base/Min/Max then u/d | o fan then u/d | z fan 110
 * c  corner assist on/off
 * y/q  slowdown on/off | b cornerSpeed | g slowErr, then u/d
 * ? params | x/k telemetry
 *
 * Kd is per ms. Left motor flipped in Motor(). TA6586 max 14V.
 */
#include <QTRSensors.h>
#include <Servo.h>

Servo myservo;

#define CONTROL_PERIOD_US  2000UL // 500 Hz
#define DT_MS              2.0f
#define ESC_PERIOD_MS      20UL // 50hz
#define TELEM_PERIOD_MS    100UL // 10hz

static uint32_t lastControlUs = 0;
static uint32_t lastEscMs     = 0;
static uint32_t lastTelemMs   = 0;
static uint16_t overrunCount  = 0;

#define ESC_PIN  10
#define AIN_2    5
#define AIN_1    7
#define BIN_1    8
#define BIN_2    6
#define LEFT     11
#define RIGHT    12

int   Servo_Speed = 40;
int   Max         = 110;
int   Min         = 0;
int   baseSpeed   = 85;
float Kp          = 0.07f;
float Kd          = 1.6f;
float dFilter     = 0.35f;

/* corner assist */
bool  cornerAssist   = false; // c = toggle
int   sharpThreshold = 2800;
int   pivotReverse   = -180;

/* dynamic slowdown */
bool  dynamicSpeed = false;
int   cornerSpeed  = 50;
int   slowErr      = 1500;
float baseSmooth   = 0.15f;
float dynBase      = 85.0f;

bool  Enable       = false;
bool  serialEnable = false;
short state        = 0;

int   Error = 0, lastError = 0;
float dFiltered = 0.0f;
float PID = 0;
int   incomingByte;
int   zuunHurd, baruunHurd;

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void printParams()
{
  Serial.print(F("Kp=")); Serial.print(Kp, 3);
  Serial.print(F(" Kd=")); Serial.print(Kd, 2);
  Serial.print(F(" dF=")); Serial.print(dFilter, 2);
  Serial.print(F(" base=")); Serial.print(baseSpeed);
  Serial.print(F(" Max=")); Serial.print(Max);
  Serial.print(F(" Min=")); Serial.print(Min);
  Serial.print(F(" CA=")); Serial.print(cornerAssist ? F("ON") : F("OFF"));
  if (dynamicSpeed) {
    Serial.print(F(" DYN cspd=")); Serial.print(cornerSpeed);
    Serial.print(F(" sErr=")); Serial.print(slowErr);
  } else {
    Serial.print(F(" FLAT"));
  }
  Serial.print(F(" Servo=")); Serial.print(Servo_Speed);
  Serial.print(F(" OVR=")); Serial.println(overrunCount);
}

void Motor(int zuun, int baruun)
{
  if (zuun   >  255) zuun   =  255;
  if (baruun >  255) baruun =  255;
  if (zuun   < -255) zuun   = -255;
  if (baruun < -255) baruun = -255;

  /* left, flipped */
  if (zuun < 0)      { analogWrite(AIN_2, abs(zuun));  digitalWrite(AIN_1, 0); }
  else if (zuun > 0) { analogWrite(AIN_2, 255 - zuun); digitalWrite(AIN_1, 1); }
  else               { digitalWrite(AIN_2, 0);         digitalWrite(AIN_1, 0); }

  /* right */
  if (baruun > 0)      { analogWrite(BIN_2, baruun);            digitalWrite(BIN_1, 0); }
  else if (baruun < 0) { analogWrite(BIN_2, 255 - abs(baruun)); digitalWrite(BIN_1, 1); }
  else                 { digitalWrite(BIN_2, 0);                digitalWrite(BIN_1, 0); }
}

void setup()
{
  Serial.begin(9600);
  delay(100);
  while (Serial.available()) Serial.read();

  ADCSRA = (ADCSRA & 0xF8) | 0x05;   // faster adc

  pinMode(AIN_2, OUTPUT);
  pinMode(AIN_1, OUTPUT);
  pinMode(BIN_1, OUTPUT);
  pinMode(BIN_2, OUTPUT);
  Motor(0, 0);

  pinMode(LEFT,  INPUT_PULLUP);
  pinMode(RIGHT, INPUT_PULLUP);

  myservo.attach(ESC_PIN);
  myservo.write(Servo_Speed);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A7, A6, A5, A4, A3, A2, A1, A0 }, SensorCount);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 300; i++) qtr.calibrate();
  digitalWrite(LED_BUILTIN, LOW);

  for (uint8_t i = 0; i < SensorCount; i++) { Serial.print(qtr.calibrationOn.minimum[i]); Serial.print(' '); }
  Serial.println();
  for (uint8_t i = 0; i < SensorCount; i++) { Serial.print(qtr.calibrationOn.maximum[i]); Serial.print(' '); }
  Serial.println();

  for (uint8_t i = 0; i < SensorCount; i++) {
    if ((qtr.calibrationOn.maximum[i] - qtr.calibrationOn.minimum[i]) < 150) {
      Serial.print(F("WARN weak sensor ")); Serial.println(i);
    }
  }

  Serial.println(F("Start v3.4"));
  lastControlUs = micros();
}

void handleSerial()
{
  incomingByte = Serial.read();

  if      (state == 1) { if (incomingByte=='u'){Kp+=0.02f;Serial.println(Kp,3);} else if (incomingByte=='d'){Kp-=0.02f;Serial.println(Kp,3);} }
  else if (state == 3) { if (incomingByte=='u'){Kd+=0.2f; Serial.println(Kd,2);} else if (incomingByte=='d'){Kd-=0.2f; Serial.println(Kd,2);} }
  else if (state == 4) { if (incomingByte=='u'){baseSpeed+=10;Max+=10;Serial.println(baseSpeed);} else if (incomingByte=='d'){baseSpeed-=10;Max-=10;Serial.println(baseSpeed);} }
  else if (state == 5) { if (incomingByte=='u'){Servo_Speed+=5;Serial.println(Servo_Speed);} else if (incomingByte=='d'){Servo_Speed-=5;Serial.println(Servo_Speed);} }
  else if (state == 6) { if (incomingByte=='u'){Min+=10;Serial.println(Min);} else if (incomingByte=='d'){Min-=10;Serial.println(Min);} }
  else if (state == 7) { if (incomingByte=='u'){Max+=10;Serial.println(Max);} else if (incomingByte=='d'){Max-=10;Serial.println(Max);} }
  else if (state == 8) { if (incomingByte=='u'){cornerSpeed+=10;Serial.println(cornerSpeed);} else if (incomingByte=='d'){cornerSpeed-=10;Serial.println(cornerSpeed);} }
  else if (state == 9) {
    if (incomingByte=='u')      { slowErr += 250; Serial.println(slowErr); }
    else if (incomingByte=='d') { slowErr -= 250; if (slowErr < 250) slowErr = 250; Serial.println(slowErr); }
  }

  switch (incomingByte)
  {
    case 'p': state = 1; Serial.println(F("P")); break;
    case 'D': state = 3; Serial.println(F("D")); break;
    case 'h': state = 4; Serial.println(F("BSPEED")); break;
    case 'o': state = 5; Serial.println(F("Servo")); break;
    case 'n': state = 6; Serial.println(F("MIN")); break;
    case 'a': state = 7; Serial.println(F("MAX")); break;
    case 'b': state = 8; Serial.println(F("CSPD")); break;
    case 'g': state = 9; Serial.println(F("SLOWERR")); break;

    case 'c': cornerAssist = !cornerAssist; Serial.println(cornerAssist ? F("CA ON") : F("CA OFF")); break;

    case 'y': dynamicSpeed = true;  dynBase = (float)baseSpeed; Serial.println(F("DYN ON"));  break;
    case 'q': dynamicSpeed = false;                             Serial.println(F("DYN OFF")); break;

    case 's': Enable = true; lastError = 0; dFiltered = 0; dynBase = (float)baseSpeed; overrunCount = 0; break;
    case 'r': Enable = false; Motor(0,0); Servo_Speed = 40; break;

    /* no reverse */
    case '1': Kp=0.07f; Kd=1.0f; baseSpeed=40; Max=90;  Min=0; dynamicSpeed=false; Serial.println(F("V1")); break;
    case '2': Kp=0.07f; Kd=2.0f; baseSpeed=50; Max=100; Min=0; dynamicSpeed=false; Serial.println(F("V2")); break;
    case '3': Kp=0.11f; Kd=2.0f; baseSpeed=60; Max=120; Min=0; dynamicSpeed=false; Serial.println(F("V3")); break;
    case '4': Kp=0.15f; Kd=2.0f; baseSpeed=60; Max=120; Min=0; dynamicSpeed=false; Serial.println(F("V4")); break;
    case '5': Kp=0.15f; Kd=2.6f; baseSpeed=70; Max=140; Min=0; dynamicSpeed=false; Serial.println(F("V5")); break;

    /* pivot */
    case '6': Kp=0.07f; Kd=2.4f; baseSpeed=60;  Max=90;  Min=-50;  dynamicSpeed=false; Serial.println(F("V6")); break;
    case '7': Kp=0.07f; Kd=2.4f; baseSpeed=70;  Max=100; Min=-70;  dynamicSpeed=false; Serial.println(F("V7")); break;
    case '8': Kp=0.07f; Kd=2.4f; baseSpeed=80;  Max=110; Min=-90;  dynamicSpeed=false; Serial.println(F("V8")); break;
    case '9': Kp=0.07f; Kd=2.6f; baseSpeed=90;  Max=130; Min=-110; dynamicSpeed=false; Serial.println(F("V9")); break;
    case '0': Kp=0.07f; Kd=2.8f; baseSpeed=100; Max=140; Min=-130; dynamicSpeed=false; Serial.println(F("V0")); break;

    case 'l': baseSpeed=115; Max=210; dynamicSpeed=false; Serial.println(F("L")); break;
    case 'm': baseSpeed=130; Max=230; dynamicSpeed=false; Serial.println(F("M")); break;

    /* pivot + slowdown */
    case 'B': Kp=0.08f; Kd=2.8f; baseSpeed=100; cornerSpeed=70; slowErr=1500; Max=140; Min=-90;  dynamicSpeed=true; dynBase=100; Serial.println(F("B-dyn")); break;
    case 'C': Kp=0.09f; Kd=3.2f; baseSpeed=115; cornerSpeed=75; slowErr=1400; Max=160; Min=-105; dynamicSpeed=true; dynBase=115; Serial.println(F("C-dyn")); break;
    case 'E': Kp=0.10f; Kd=3.6f; baseSpeed=130; cornerSpeed=80; slowErr=1300; Max=180; Min=-120; dynamicSpeed=true; dynBase=130; Serial.println(F("E-dyn")); break;

    /* pivot faster */
    case 'F': Kp=0.08f; Kd=2.8f; baseSpeed=100; Max=140; Min=-90;  dynamicSpeed=false; Serial.println(F("F")); break;
    case 'G': Kp=0.09f; Kd=3.2f; baseSpeed=115; Max=160; Min=-105; dynamicSpeed=false; Serial.println(F("G")); break;
    case 'H': Kp=0.10f; Kd=3.6f; baseSpeed=130; Max=180; Min=-120; dynamicSpeed=false; Serial.println(F("H")); break;

    case 'x': serialEnable = true;  break;
    case 'k': serialEnable = false; break;
    case 'z': Servo_Speed = 110; break;
    case '?': printParams(); break;
    default: break;
  }
  incomingByte = 0;
}

void loop()
{
  if (Serial.available() > 0) handleSerial();

  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastControlUs) < CONTROL_PERIOD_US) return;

  if ((uint32_t)(nowUs - lastControlUs) > (CONTROL_PERIOD_US * 2)) overrunCount++;
  lastControlUs += CONTROL_PERIOD_US;
  if ((uint32_t)(nowUs - lastControlUs) > CONTROL_PERIOD_US) lastControlUs = nowUs;

  uint16_t position = qtr.readLineBlack(sensorValues);
  Error = (int)position - 3500;

  float dRaw = (float)(Error - lastError) / DT_MS;
  dFiltered  = dFiltered + (1.0f - dFilter) * (dRaw - dFiltered);
  lastError  = Error;

  PID = (Kp * (float)Error) + (Kd * dFiltered);

  /* speed: flat or eased by error */
  int base;
  if (dynamicSpeed) {
    int absErr = Error < 0 ? -Error : Error;
    if (absErr > slowErr) absErr = slowErr;
    float target = (float)baseSpeed - ((float)(baseSpeed - cornerSpeed) * absErr / slowErr);
    dynBase += baseSmooth * (target - dynBase);
    base = (int)dynBase;
  } else {
    base = baseSpeed;
  }

  int steer = (int)PID;

  /* Min 0: speed gives way to steering */
  if (Min >= 0) {
    int absSteer = steer < 0 ? -steer : steer;
    int headroom = Max - absSteer;
    if (headroom < 0) headroom = 0;
    if (base > headroom) base = headroom;
  }

  zuunHurd   = base + steer;
  baruunHurd = base - steer;

  if (zuunHurd   > Max) zuunHurd   = Max;
  if (baruunHurd > Max) baruunHurd = Max;
  if (zuunHurd   < Min) zuunHurd   = Min;
  if (baruunHurd < Min) baruunHurd = Min;

  /* corner assist */
  if (cornerAssist) {
    if (Error > sharpThreshold)       { zuunHurd = Max;          baruunHurd = pivotReverse; }
    else if (Error < -sharpThreshold) { zuunHurd = pivotReverse; baruunHurd = Max; }
  }

  if (Enable) Motor(zuunHurd, baruunHurd);
  else        Motor(0, 0);

  uint32_t nowMs = millis();
  if (nowMs - lastEscMs >= ESC_PERIOD_MS) {
    lastEscMs = nowMs;
    myservo.write(Servo_Speed);
  }

  if (serialEnable && (nowMs - lastTelemMs >= TELEM_PERIOD_MS)) {
    lastTelemMs = nowMs;
    Serial.print(position);   Serial.print(F(" e="));
    Serial.print(Error);      Serial.print(F(" L="));
    Serial.print(zuunHurd);   Serial.print(F(" R="));
    Serial.println(baruunHurd);
  }
}