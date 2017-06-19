#include <Servo.h>
#include <EEPROM.h>
#include <avr/wdt.h>

#define EMERG_HIGH 2
#define EMERG_LOW 3
#define APS1 A0
#define APS2 A1
#define TPS1 A2
#define TPS2 A3
#define BPS A4

#define SHORT_ERR 1
#define LONG_ERR 2

#define SHORT_ERR_MAX 90
#define LONG_ERR_MAX 400

#define RAWTOFLOAT(RAW, MIN, MAX) (((float) RAW - (float) MIN)/((float) MAX - (float) MIN))
#define FLOATTORAW(FLOAT, MIN, MAX) ((int) ((((float) MAX - (float) MIN) * FLOAT) + (float) MIN))
int APS1_MAX = 1023;
int APS1_MIN = 0;
int APS1_RAW = 0;
float APS1_POS = 0;
int APS2_MAX = 0;
int APS2_MIN = 1023;
int APS2_RAW = 0;
float APS2_POS = 0;
int TPS1_MAX = 1023;
int TPS1_MIN = 0;
int TPS1_RAW = 0;
float TPS1_POS = 0;
int TPS2_MAX = 0;
int TPS2_MIN = 1023;
int TPS2_RAW = 0;
float TPS2_POS = 0;
int BPS_MIN = 0;
int BPS_MAX = 1023;
int BPS_RAW = 0;
int BPS_THRES = 100;
float BPS_POS = 0;
float BPS_POS_THRES = RAWTOFLOAT(BPS_THRES, BPS_MIN, BPS_MAX);

int THR_MAX = 2500;
int THR_MIN = 1500;
float THR_POS = 0;
float THR_IDLE = 0.05;
int THR_IDLE_RAW = FLOATTORAW(THR_IDLE, THR_MIN, THR_MAX);

float ACCEPT_DIFF = 0.05;

int ERROR_TYPE = 0;

int IN_ERROR = 0;
unsigned long LONG_ERROR_TIME = 0;
int LONG_ERROR_TIME_ACTIVE = 0;
unsigned long SHORT_ERROR_TIME = 0;
int SHORT_ERROR_TIME_ACTIVE = 0;

int WE_ARE_PANICKING = 0;

Servo throttle;

int BLIP = 2200;
int CUT = 1000;
int UNBLIP = 0;
unsigned long LASTNORM = 0;
unsigned long LASTBLIP = 0;
#define MAXTIME 500

int pos = 0;
int thr = 0;
void setup()
{
  wdt_reset();
  pinMode(EMERG_HIGH, OUTPUT);
  pinMode(EMERG_LOW, OUTPUT);
  pinMode(11, INPUT);
  pinMode(12, INPUT);
  digitalWrite(EMERG_HIGH, HIGH);
  digitalWrite(EMERG_LOW, LOW);
  throttle.attach(6);
  throttle.writeMicroseconds(THR_MIN);
  Serial.begin(57600);
  watchdogSetup();
}

void loop()
{
  APS1_RAW = analogRead(APS1);
  APS2_RAW = analogRead(APS2);
  TPS1_RAW = analogRead(TPS1);
  TPS2_RAW = analogRead(TPS2);
  BPS_RAW = analogRead(BPS);
  APS1_POS = RAWTOFLOAT(APS1_RAW, APS1_MIN, APS1_MAX);
  APS2_POS = RAWTOFLOAT(APS2_RAW, APS2_MIN, APS2_MAX);
  TPS1_POS = RAWTOFLOAT(TPS1_RAW, TPS1_MIN, TPS1_MAX);
  TPS2_POS = RAWTOFLOAT(TPS2_RAW, TPS2_MIN, TPS2_MAX);
  BPS_POS = RAWTOFLOAT(BPS_RAW, BPS_MIN, BPS_MAX);
  Serial.print(APS1_POS);
  Serial.print("|");
  Serial.print(APS2_POS);
  Serial.print("|");
  Serial.print(TPS1_POS);
  Serial.print("|");
  Serial.print(TPS2_POS);
  Serial.println("");
  Serial.print(BPS_RAW);
  Serial.print("|");
  Serial.println(BPS_THRES);
  if ((millis() - LASTNORM) > 450)
  {
    UNBLIP = 1;
  }
  else if ((millis() - LASTBLIP) > 550)
  {
    UNBLIP = 0;
  }
  if (digitalRead(11) && !UNBLIP)
  {
    throttle.writeMicroseconds(CUT);
    LASTBLIP = millis();
  }
  else if (digitalRead(12) && !UNBLIP)
  {
    throttle.writeMicroseconds(BLIP);
    LASTBLIP = millis();
  }
  else
  {
    throttle.writeMicroseconds((((APS1_POS + APS2_POS) / 2) * (THR_MAX - THR_MIN)) + THR_MIN);
    LASTNORM = millis();

  }
  errorcheck();
}

void errorcheck()
{
  int shorterror = 0;
  int longerror = 0;
  if (TPS1_RAW > max(TPS1_MAX, TPS1_MIN) || TPS1_RAW < min(TPS1_MIN, TPS1_MAX))
  {
    shorterror = 1;
    ERROR_TYPE |= 1;
  }
  else
  {
    ERROR_TYPE &= ~1;
  }
  if (TPS2_RAW > max(TPS2_MAX, TPS2_MIN) || TPS2_RAW < min(TPS2_MIN, TPS2_MAX))
  {
    shorterror = 1;
    ERROR_TYPE |= 2;
  }
  else
  {
    ERROR_TYPE &= ~2;
  }
  if (APS1_RAW > max(APS1_MAX, APS1_MAX) || APS1_RAW < min(APS1_MIN, APS1_MAX))
  {
    shorterror = 1;
    ERROR_TYPE |= 4;
  }
  else
  {
    ERROR_TYPE &= ~4;
  }
  if (APS2_RAW > max(APS2_MAX, APS2_MIN) || APS2_RAW < min(APS2_MIN, APS2_MAX))
  {
    shorterror = 1;
    ERROR_TYPE |= 8;
  }
  else
  {
    ERROR_TYPE &= ~8;
  }
  if (TPS1_POS > 1 || TPS1_POS < 0)
  {
    shorterror = 1;
    ERROR_TYPE |= 16;
  }
  else
  {
    ERROR_TYPE &= ~16;
  }
  if (TPS2_POS > 1 || TPS2_POS < 0)
  {
    shorterror = 1;
    ERROR_TYPE |= 32;
  }
  else
  {
    ERROR_TYPE &= ~32;
  }
  if (APS1_POS > 1 || APS1_POS < 0)
  {
    shorterror = 1;
    ERROR_TYPE |= 64;
  }
  else
  {
    ERROR_TYPE &= ~64;
  }
  if (APS2_POS > 1 || APS2_POS < 0)
  {
    shorterror = 1;
    ERROR_TYPE |= 128;
  }
  if (BPS_RAW > max(BPS_MAX, BPS_MIN) || BPS_RAW < min(BPS_MAX, BPS_MIN))
  {
    shorterror = 1;
    ERROR_TYPE &= ~128;
  }
  if (BPS_POS < 0 || BPS_POS > 1)
  {
    shorterror = 1;
    ERROR_TYPE |= 256;
  }
  else
  {
    ERROR_TYPE &= ~256;
  }
  if (abs(TPS1_POS - TPS2_POS) > ACCEPT_DIFF)
  {
    shorterror = 1;
    ERROR_TYPE |= 512;
  }
  else
  {
    ERROR_TYPE &= ~512;
  }
  if (abs(APS1_POS - APS2_POS) > ACCEPT_DIFF)
  {
    shorterror = 1;
    ERROR_TYPE |= 1024;
  }
  else
  {
    ERROR_TYPE &= ~1024;
  }
  if (shorterror)
  {
    IN_ERROR |= SHORT_ERR;
    if (SHORT_ERROR_TIME_ACTIVE == 0)
    {
      SHORT_ERROR_TIME = millis();
      SHORT_ERROR_TIME_ACTIVE = 1;
    }
  }
  else
  {
    IN_ERROR &= ~(SHORT_ERR);
    SHORT_ERROR_TIME_ACTIVE = 0;
  }
  checkpanic();
  if (abs(APS1_POS - TPS1_POS) > ACCEPT_DIFF || abs(APS2_POS - TPS1_POS) > ACCEPT_DIFF || abs(APS1_POS - TPS2_POS) > ACCEPT_DIFF || abs(APS2_POS - TPS2_POS) > ACCEPT_DIFF)
  {
    if ((APS1_POS < ACCEPT_DIFF || APS2_POS < ACCEPT_DIFF) && (TPS1_POS > (THR_IDLE + (THR_IDLE * ACCEPT_DIFF)) || TPS1_POS > (THR_IDLE + (THR_IDLE * ACCEPT_DIFF))))
    {
      longerror = 1;
      ERROR_TYPE |= 2048;
    }
    else
    {
      ERROR_TYPE &= ~2048;
    }
  }
  if (BPS_RAW > BPS_THRES && (TPS1_POS > (THR_IDLE + (THR_IDLE * ACCEPT_DIFF)) || TPS1_POS > (THR_IDLE + (THR_IDLE * ACCEPT_DIFF))))
  {
    longerror = 1;
    ERROR_TYPE |= 4096;
  }
  else
  {
    ERROR_TYPE &= ~4096;
  }
  if (longerror)
  {
    IN_ERROR |= LONG_ERR;
    if (LONG_ERROR_TIME_ACTIVE == 0)
    {
      LONG_ERROR_TIME = millis();
      LONG_ERROR_TIME_ACTIVE = 1;
    }
  }
  else
  {
    IN_ERROR &= ~(LONG_ERR);
    LONG_ERROR_TIME_ACTIVE = 0;
  }
  checkpanic();
}

void checkpanic()
{
  if (((IN_ERROR & SHORT_ERR) && ((millis() - SHORT_ERROR_TIME) > SHORT_ERR_MAX)) || ((IN_ERROR & LONG_ERR) && ((millis() - LONG_ERROR_TIME) > LONG_ERR_MAX)))
  {
    panic();
  }
  else
  {
    wdt_reset();
  }
}
void watchdogSetup(void)
{
  cli();
  wdt_reset();
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  WDTCSR = (1 << WDIE) | (1 << WDE) | (0 << WDP3) | (0 << WDP2) | (1 << WDP1) | (0 << WDP0); //64ms
  sei();
}

ISR(WDT_vect)
{
  panic();
}

void panic()
{
  wdt_reset();
  cli();
  WDTCSR |= (1 << WDCE);
  WDTCSR = 0;
  sei();
  WE_ARE_PANICKING = 1;
  throttle.writeMicroseconds(THR_IDLE_RAW);
  digitalWrite(EMERG_HIGH, LOW);
  digitalWrite(EMERG_LOW, HIGH);
  while (1)
  {
    APS1_RAW = analogRead(APS1);
    APS2_RAW = analogRead(APS2);
    TPS1_RAW = analogRead(TPS1);
    TPS2_RAW = analogRead(TPS2);
    BPS_RAW = analogRead(BPS);
    digitalWrite(EMERG_HIGH, LOW);
    digitalWrite(EMERG_LOW, HIGH);
    Serial.println("PANIC");
    Serial.println(ERROR_TYPE);
    Serial.print(RAWTOFLOAT(APS1_RAW, APS1_MIN, APS1_MAX));
    Serial.print("|");
    Serial.print(RAWTOFLOAT(APS2_RAW, APS2_MIN, APS2_MAX));
    Serial.print("|");
    Serial.print(RAWTOFLOAT(TPS1_RAW, TPS1_MIN, TPS1_MAX));
    Serial.print("|");
    Serial.print(RAWTOFLOAT(TPS2_RAW, TPS2_MIN, TPS2_MAX));
    Serial.println("");
    Serial.print(BPS_RAW);
    Serial.print("|");
    Serial.println(BPS_THRES);
    delay(500);
  }

}

void (*reset) (void) = 0;
