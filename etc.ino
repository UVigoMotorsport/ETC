#include <Servo.h>
#include <EEPROM.h>
#include <avr/wdt.h>

#define OFFSET 10
#define VERSION 1

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


#define SERVOMAXMAX 2500
#define SERVOMINMIN 500

#define RAWTOFLOAT(RAW, MIN, MAX) (((float) RAW - (float) MIN)/((float) MAX - (float) MIN))
#define FLOATTORAW(FLOAT, MIN, MAX) ((int) ((((float) MAX - (float) MIN) * FLOAT) + (float) MIN))
int WRITTEN = 0;
int addr_WRITTEN = OFFSET;
int VER = 0;
int addr_VER = addr_WRITTEN + sizeof(WRITTEN);
int APS1_MAX = 1023;
int addr_APS1_MAX = addr_VER + sizeof(VER);
int APS1_MIN = 0;
int addr_APS1_MIN = addr_APS1_MAX + sizeof(APS1_MAX);
int APS1_RAW = 0;
float APS1_POS = 0;
int APS2_MAX = 1023;
int addr_APS2_MAX = addr_APS1_MIN + sizeof(APS1_MIN);
int APS2_MIN = 0;
int addr_APS2_MIN = addr_APS2_MAX + sizeof(APS2_MAX);
int APS2_RAW = 0;
float APS2_POS = 0;
int TPS1_MAX = 1023;
int addr_TPS1_MAX = addr_APS2_MIN + sizeof(APS2_MIN);
int TPS1_MIN = 0;
int addr_TPS1_MIN = addr_TPS1_MAX + sizeof(TPS1_MAX);
int TPS1_RAW = 0;
float TPS1_POS = 0;
int TPS2_MAX = 1023;
int addr_TPS2_MAX = addr_TPS1_MIN + sizeof(TPS1_MIN);
int TPS2_MIN = 0;
int addr_TPS2_MIN = addr_TPS2_MAX + sizeof(TPS2_MAX);
int TPS2_RAW = 0;
float TPS2_POS = 0;
int BPS_MIN = 0;
int addr_BPS_MIN = addr_TPS2_MIN + sizeof(TPS2_MIN);
int BPS_MAX = 1023;
int addr_BPS_MAX = addr_BPS_MIN + sizeof(BPS_MIN);
int BPS_RAW = 0;
int BPS_THRES = 100;
int addr_BPS_THRES = addr_BPS_MAX + sizeof(BPS_MAX);
float BPS_POS = 0;
float BPS_POS_THRES = RAWTOFLOAT(BPS_THRES, BPS_MIN, BPS_MAX);

int THR_MAX = 2500;
int addr_THR_MAX = addr_BPS_THRES + sizeof(BPS_THRES);
int THR_MIN = 1500;
int addr_THR_MIN = addr_THR_MAX + sizeof(THR_MAX);
float THR_POS = 0;
float THR_IDLE = 0.05;
int addr_THR_IDLE = addr_THR_MIN + sizeof(THR_MIN);
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

int UNBLIP = 0;
unsigned long LASTNORM = 0;
unsigned long LASTBLIP = 0;
#define MAXTIME 500

int SETCHECK = 0;

int pos = 0;
int thr = 0;
void setup()
{
  EEPROM.get(addr_WRITTEN, WRITTEN);
  EEPROM.get(addr_VER, VER);
  if (WRITTEN != 1 || VER != VERSION)
  {
    EEPROM.put(addr_APS1_MAX, APS1_MAX);
    EEPROM.put(addr_APS1_MIN, APS1_MIN);
    EEPROM.put(addr_APS2_MAX, APS2_MAX);
    EEPROM.put(addr_APS2_MIN, APS1_MIN);
    EEPROM.put(addr_TPS1_MAX, TPS1_MAX);
    EEPROM.put(addr_TPS1_MIN, TPS1_MIN);
    EEPROM.put(addr_TPS2_MAX, TPS2_MAX);
    EEPROM.put(addr_TPS2_MIN, TPS2_MIN);
    EEPROM.put(addr_BPS_MIN, BPS_MIN);
    EEPROM.put(addr_BPS_MAX, BPS_MAX);
    EEPROM.put(addr_BPS_THRES, BPS_THRES);
    EEPROM.put(addr_THR_MAX, THR_MAX);
    EEPROM.put(addr_THR_MIN, THR_MIN);
    EEPROM.put(addr_THR_IDLE, THR_IDLE);
    EEPROM.put(addr_WRITTEN, 1);
    EEPROM.put(addr_VER, VERSION);
  }
  else
  {
    EEPROM.get(addr_APS1_MAX, APS1_MAX);
    EEPROM.get(addr_APS1_MIN, APS1_MIN);
    EEPROM.get(addr_APS2_MAX, APS2_MAX);
    EEPROM.get(addr_APS2_MIN, APS1_MIN);
    EEPROM.get(addr_TPS1_MAX, TPS1_MAX);
    EEPROM.get(addr_TPS1_MIN, TPS1_MIN);
    EEPROM.get(addr_TPS2_MAX, TPS2_MAX);
    EEPROM.get(addr_TPS2_MIN, TPS2_MIN);
    EEPROM.get(addr_BPS_MIN, BPS_MIN);
    EEPROM.get(addr_BPS_MAX, BPS_MAX);
    EEPROM.get(addr_BPS_THRES, BPS_THRES);
    EEPROM.get(addr_THR_MAX, THR_MAX);
    EEPROM.get(addr_THR_MIN, THR_MIN);
    EEPROM.get(addr_THR_IDLE, THR_IDLE);
    BPS_POS_THRES = RAWTOFLOAT(BPS_THRES, BPS_MIN, BPS_MAX);
    THR_IDLE_RAW = FLOATTORAW(THR_IDLE, THR_MIN, THR_MAX);
  }
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
  /*Serial.print(APS1_POS);
    Serial.print("|");
    Serial.print(APS2_POS);
    Serial.print("|");
    Serial.print(TPS1_POS);
    Serial.print("|");
    Serial.print(TPS2_POS);
    Serial.println("");*/
    Serial.print(BPS_POS);
    Serial.print("|");
    Serial.println(BPS_POS_THRES);
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
    throttle.writeMicroseconds(THR_MIN);
    LASTBLIP = millis();
  }
  else if (digitalRead(12) && !UNBLIP)
  {
    throttle.writeMicroseconds(THR_MAX);
    LASTBLIP = millis();
  }
  else
  {
    throttle.writeMicroseconds((int)(((APS1_POS + APS2_POS) / 2) * (THR_MAX - THR_IDLE_RAW)) + THR_IDLE_RAW);
    Serial.println((int)(((APS1_POS + APS2_POS) / 2) * (THR_MAX - THR_IDLE_RAW)) + THR_IDLE_RAW);
    LASTNORM = millis();
  }
  if (millis() - SETCHECK > 500)
  {
    int in = Serial.read();
    if (in == 's')
    {
      set();
    }
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
  if (BPS_POS > BPS_POS_THRES && (TPS1_POS > (THR_IDLE + (THR_IDLE * ACCEPT_DIFF)) || TPS1_POS > (THR_IDLE + (THR_IDLE * ACCEPT_DIFF))))
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
    BPS_POS = RAWTOFLOAT(BPS_RAW, BPS_MIN, BPS_MAX);
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
    Serial.print(BPS_POS);
    Serial.print("|");
    Serial.println(BPS_POS_THRES);
    delay(500);
  }

}

void (*reset) (void) = 0;

void set()
{
  int in = 0;
  Serial.println("Entering set mode");
  while (in != 'q')
  {
    in = Serial.read();
    int in2 = 0;
    wdt_reset();
    switch (in)
    {
      case 'i':
        Serial.println("Entering set idle mode");
        while (in2 != 'q')
        {
          in2 = Serial.read();
          switch (in2)
          {
            case '+':
              if (THR_IDLE < 0.499)
              {
                THR_IDLE += 0.001;
                THR_IDLE_RAW = FLOATTORAW(THR_IDLE, THR_MIN, THR_MAX);
                throttle.writeMicroseconds(THR_IDLE_RAW);
                Serial.print(THR_IDLE * 100);
                Serial.println("%");
              }
              break;
            case '-':
              if (THR_IDLE > 0.001)
              {
                THR_IDLE -= 0.001;
                THR_IDLE_RAW = FLOATTORAW(THR_IDLE, THR_MIN, THR_MAX);
                throttle.writeMicroseconds(THR_IDLE_RAW);
                Serial.print(THR_IDLE * 100);
                Serial.println("%");
              }
              break;
            default:
              break;
          }
          wdt_reset();
        }
        EEPROM.put(addr_THR_IDLE, THR_IDLE);
        Serial.println("Exiting set idle mode");
        break;
      case 'a':
        Serial.println("Entering APS set mode");
        Serial.println("Leave the pedal alone and press d");
        in2 = 0;
        while (in2 != 'd')
        {
          wdt_reset();
          in2 = Serial.read();
        }
        APS1_MIN = analogRead(APS1);
        APS2_MIN = analogRead(APS2);
        EEPROM.put(addr_APS1_MIN, APS1_MIN);
        EEPROM.put(addr_APS2_MIN, APS2_MIN);
        Serial.println("Press the pedal all the way and press d");
        in2 = 0;
        while (in2 != 'd')
        {
          wdt_reset();
          in2 = Serial.read();
        }
        APS1_MAX = analogRead(APS1);
        APS2_MAX = analogRead(APS2);
        EEPROM.put(addr_APS1_MAX, APS1_MAX);
        EEPROM.put(addr_APS2_MAX, APS2_MAX);
        Serial.println("Exiting APS set mode");
        break;
      case 't':
        Serial.println("Entering TPS set mode");
        Serial.println("Close the throttle all the way manually and press d");
        in2 = 0;
        while (in2 != 'd')
        {
          wdt_reset();
          in2 = Serial.read();
        }
        TPS1_MIN = analogRead(TPS1);
        TPS2_MIN = analogRead(TPS2);
        EEPROM.put(addr_TPS1_MIN, TPS1_MIN);
        EEPROM.put(addr_TPS2_MIN, TPS2_MIN);
        Serial.println("Open the throttle all the way manually and press d");
        in2 = 0;
        while (in2 != 'd')
        {
          wdt_reset();
          in2 = Serial.read();
        }
        TPS1_MAX = analogRead(TPS1);
        TPS2_MAX = analogRead(TPS2);
        EEPROM.put(addr_TPS1_MAX, TPS1_MAX);
        EEPROM.put(addr_TPS2_MAX, TPS2_MAX);
        Serial.println("Exiting TPS set mode");
        break;
      case 'p':
        Serial.println("Entering throttle position set mode");
        while (in2 != 'q')
        {
          in2 = Serial.read();
          int in3 = 0;
          switch (in2)
          {
            case '+':
              Serial.println("Entering set max");
              throttle.writeMicroseconds(THR_MAX);
              while (in3 != 'q')
              {
                in3 = Serial.read();
                switch (in3)
                {
                  case '+':
                    if (THR_MAX < SERVOMAXMAX)
                    {
                      THR_MAX += 1;
                      THR_IDLE_RAW = FLOATTORAW(THR_IDLE, THR_MIN, THR_MAX);
                      throttle.writeMicroseconds(THR_MAX);
                      Serial.print(THR_MAX);
                      Serial.println("ms");
                    }
                    break;
                  case '-':
                    if (THR_MAX > SERVOMINMIN)
                    {
                      THR_MAX -= 1;
                      THR_IDLE_RAW = FLOATTORAW(THR_IDLE, THR_MIN, THR_MAX);
                      throttle.writeMicroseconds(THR_MAX);
                      Serial.print(THR_MAX);
                      Serial.println("ms");
                    }
                    break;
                  default:
                    break;
                }
                wdt_reset();
              }
              EEPROM.put(addr_THR_MAX, THR_MAX);
              Serial.println("Exiting set max");
              break;
            case '-':
              Serial.println("Entering set min");
              throttle.writeMicroseconds(THR_MIN);
              while (in3 != 'q')
              {
                in3 = Serial.read();
                switch (in3)
                {
                  case '+':
                    if (THR_MIN < SERVOMAXMAX)
                    {
                      THR_MIN += 1;
                      THR_IDLE_RAW = FLOATTORAW(THR_IDLE, THR_MIN, THR_MAX);
                      throttle.writeMicroseconds(THR_MIN);
                      Serial.print(THR_MIN);
                      Serial.println("ms");
                    }
                    break;
                  case '-':
                    if (THR_MIN > SERVOMINMIN)
                    {
                      THR_MIN -= 1;
                      THR_IDLE_RAW = FLOATTORAW(THR_IDLE, THR_MIN, THR_MAX);
                      throttle.writeMicroseconds(THR_MIN);
                      Serial.print(THR_MIN);
                      Serial.println("ms");
                    }
                    break;
                  default:
                    break;
                }
                wdt_reset();
              }
              EEPROM.put(addr_THR_MIN, THR_MIN);
              Serial.println("Exiting set min");
              break;
            default:
              break;
          }
          wdt_reset();
        }
        Serial.println("Exiting throttle position set mode");
        break;
      case 'b':
        Serial.println("Entering brake set mode");
        Serial.println("Leave the brake pedal alone and press d");
        while (in2 != 'd')
        {
          in2 = Serial.read();
          wdt_reset();
        }
        in2 = 0;
        BPS_MIN = analogRead(BPS);
        EEPROM.put(addr_BPS_MIN, BPS_MIN);
        Serial.println("Press the brake pedal lightly and press d");
        while (in2 != 'd')
        {
          in2 = Serial.read();
          wdt_reset();
        }
        in2 = 0;
        BPS_THRES = analogRead(BPS);
        EEPROM.put(addr_BPS_THRES, BPS_THRES);
        Serial.println("Toni: Press the brake pedal as hard as possible and press d");
        while (in2 != 'd')
        {
          in2 = Serial.read();
          wdt_reset();
        }
        in2 = 0;
        BPS_MAX = analogRead(BPS);
        EEPROM.put(addr_BPS_MAX, BPS_MAX);
        BPS_POS_THRES = RAWTOFLOAT(BPS_THRES, BPS_MIN, BPS_MAX);
        Serial.println("Exiting brake set mode");
        break;
      default:
        break;
    }
  }
  Serial.println("Exiting set mode");
}

