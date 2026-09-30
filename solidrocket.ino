#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_BMP280.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_system.h>

#define PIN_SDA     0
#define PIN_SCL     1
#define PIN_MOSI    3
#define PIN_MISO    4
#define PIN_NSS     5
#define PIN_SCK     6
#define PIN_RST     7
#define PIN_SERVO  10
#define PIN_DIO0   18

#define ROCKET_ID       37
#define PKT_BIG_ENDIAN   1

#define LORA_FREQ   434500000L
#define LORA_BW         125E3
#define LORA_SF             7
#define LORA_CR             5
#define LORA_PREAMBLE       8
#define LORA_SYNC        0x12
#define TX_POWER           17
#define TX_PERIOD_MS      500

#define SERVO_LOCKED     20
#define SERVO_OPEN      110
#define SERVO_SELFTEST    0

#define SAMPLE_MS         20
#define STABILIZE_MS    4000
#define CAL_SAMPLES      120

#define LAUNCH_ALT_M      25.0f
#define LAUNCH_HOLD_MS     250
#define BOOST_LOCKOUT_MS  2500
#define MIN_PEAK_M        60.0f
#define APOGEE_DROP_M      2.5f
#define APOGEE_HOLD          5
#define BACKUP_DEPLOY_MS 18000
#define LAND_TOL_M         3.0f
#define LAND_HOLD_MS      5000

enum { ST_PAD, ST_ARMED, ST_BOOST, ST_COAST, ST_DESCENT, ST_LANDED };

Adafruit_BMP280 bmp;
Servo chute;

float groundPa   = 0;
float alt        = 0;
float peakAlt    = 0;
float apogeeAlt  = 0;

uint8_t  state   = ST_PAD;
uint16_t seq     = 0;
bool     deployed = false;

unsigned long tSample   = 0;
unsigned long tTx       = 0;
unsigned long tBoot     = 0;
unsigned long tLaunch   = 0;
unsigned long tCand     = 0;
unsigned long tLandRef  = 0;
bool  cand      = false;
int   dropCount = 0;
float landRefAlt = 0;
bool  altPacket  = true;


float altitudeFromPa(float pa) {
  return 44330.0f * (1.0f - powf(pa / groundPa, 0.1902949f));
}

void bootReason() {
  esp_reset_reason_t r = esp_reset_reason();
  Serial.print("reset: ");
  if (r == ESP_RST_POWERON)       Serial.println("POWERON");
  else if (r == ESP_RST_BROWNOUT) Serial.println("BROWNOUT  <-- fix the supply");
  else if (r == ESP_RST_PANIC)    Serial.println("PANIC");
  else if (r == ESP_RST_SW)       Serial.println("SW");
  else { Serial.print("code "); Serial.println((int)r); }
}

void sendPacket(uint8_t type, float metres) {
  long v = lroundf(metres * 10.0f);
  if (v < 0) v = 0;
  if (v > 65535) v = 65535;
  uint16_t a = (uint16_t)v;

  seq++;

  uint8_t p[6];
  p[0] = ROCKET_ID;
  p[1] = type;
#if PKT_BIG_ENDIAN
  p[2] = seq >> 8;   p[3] = seq & 0xFF;
  p[4] = a >> 8;     p[5] = a & 0xFF;
#else
  p[2] = seq & 0xFF; p[3] = seq >> 8;
  p[4] = a & 0xFF;   p[5] = a >> 8;
#endif

  LoRa.beginPacket();
  LoRa.write(p, 6);
  LoRa.endPacket();
}

void deploy(const char *why) {
  if (deployed) return;
  deployed = true;
  chute.write(SERVO_OPEN);
  Serial.print("DEPLOY ");
  Serial.print(why);
  Serial.print("  peak=");
  Serial.println(peakAlt, 1);
}

void calibrate() {
  float sum = 0;
  for (int i = 0; i < CAL_SAMPLES; i++) {
    sum += bmp.readPressure();
    delay(15);
  }
  groundPa = sum / CAL_SAMPLES;
  Serial.print("ground = ");
  Serial.print(groundPa / 100.0f, 2);
  Serial.println(" hPa");
}


void setup() {
  Serial.begin(115200);
  delay(400);
  WiFi.mode(WIFI_OFF);

  Serial.println();
  Serial.println("solid fuel flight computer");
  bootReason();

  chute.setPeriodHertz(50);
  chute.attach(PIN_SERVO, 500, 2400);
  chute.write(SERVO_LOCKED);

#if SERVO_SELFTEST
  delay(500);
  chute.write(SERVO_OPEN);
  delay(900);
  chute.write(SERVO_LOCKED);
  delay(500);
#endif

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 not found");
    while (1) delay(500);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X1,
                  Adafruit_BMP280::SAMPLING_X4,
                  Adafruit_BMP280::FILTER_X4,
                  Adafruit_BMP280::STANDBY_MS_1);
  Serial.println("BMP280 ok");

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);
  LoRa.setPins(PIN_NSS, PIN_RST, PIN_DIO0);
  LoRa.setSPIFrequency(8E6);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("RA-02 not found");
    while (1) delay(500);
  }
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setPreambleLength(LORA_PREAMBLE);
  LoRa.setSyncWord(LORA_SYNC);
  LoRa.setTxPower(TX_POWER);
  LoRa.enableCrc();
  Serial.println("RA-02 ok  434.500 MHz SF7 BW125 CR4/5");

  Serial.print("settling ");
  Serial.print(STABILIZE_MS / 1000);
  Serial.println("s - keep the rocket still");
  delay(STABILIZE_MS);

  calibrate();

  tBoot   = millis();
  tSample = millis();
  tTx     = millis();
  state   = ST_ARMED;
  Serial.println("ARMED");
}


void loop() {
  unsigned long now = millis();

  if (now - tSample >= SAMPLE_MS) {
    tSample = now;
    alt = altitudeFromPa(bmp.readPressure());

    if (state >= ST_BOOST && alt > peakAlt) peakAlt = alt;

    switch (state) {

      case ST_ARMED:
        if (alt > LAUNCH_ALT_M) {
          if (!cand) { cand = true; tCand = now; }
          else if (now - tCand >= LAUNCH_HOLD_MS) {
            state   = ST_BOOST;
            tLaunch = now;
            peakAlt = alt;
            Serial.println("LAUNCH");
          }
        } else {
          cand = false;
        }
        break;

      case ST_BOOST:
        if (now - tLaunch >= BOOST_LOCKOUT_MS) {
          state = ST_COAST;
          Serial.println("COAST");
        }
        break;

      case ST_COAST:
        if (peakAlt >= MIN_PEAK_M && (peakAlt - alt) >= APOGEE_DROP_M) {
          dropCount++;
          if (dropCount >= APOGEE_HOLD) {
            apogeeAlt = peakAlt;
            deploy("apogee");
            state      = ST_DESCENT;
            landRefAlt = alt;
            tLandRef   = now;
            Serial.print("APOGEE ");
            Serial.println(apogeeAlt, 1);
          }
        } else {
          dropCount = 0;
        }

        if (!deployed && now - tLaunch >= BACKUP_DEPLOY_MS) {
          apogeeAlt = peakAlt;
          deploy("backup timer");
          state      = ST_DESCENT;
          landRefAlt = alt;
          tLandRef   = now;
        }
        break;

      case ST_DESCENT:
        if (fabsf(alt - landRefAlt) > LAND_TOL_M) {
          landRefAlt = alt;
          tLandRef   = now;
        } else if (now - tLandRef >= LAND_HOLD_MS) {
          state = ST_LANDED;
          Serial.println("LANDED");
        }
        break;
    }
  }

  if (now - tTx >= TX_PERIOD_MS) {
    tTx = now;
    if (state >= ST_DESCENT) {
      if (altPacket) sendPacket(0x01, alt);
      else           sendPacket(0x02, apogeeAlt);
      altPacket = !altPacket;
    } else {
      sendPacket(0x01, alt);
    }

    Serial.print(seq);
    Serial.print("  alt ");
    Serial.print(alt, 1);
    Serial.print("  peak ");
    Serial.print(peakAlt, 1);
    Serial.print("  st ");
    Serial.println(state);
  }
}
