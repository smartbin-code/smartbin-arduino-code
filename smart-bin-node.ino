#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// define ultrasonic sensor pins
#define TRIG_PIN 5 // Trigger pin for HC-SR04
#define ECHO_PIN 4 // Echo pin for HC-SR04

// define bin parameters
#define BIN_DEPTH 100.0 // Bin depth in cm (adjust based on your bin)
#define BIN_ID 0xA0FE5980 // Hardcoded Bin ID (4 bytes, from your comment: a0 fe 59 80)
const float LATITUDE = 40.7128;   // Hardcoded latitude (e.g., New York City)
const float LONGITUDE = -74.0060; // Hardcoded longitude (e.g., New York City)

// define the pins for the Dragino LoRa Shield
const lmic_pinmap lmic_pins = {
    .nss = 10,    // RFM95_CS
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,     // RFM95_RST
    .dio = {2, 6, 7}, // DIO0, DIO1, DIO2
};

// LoRaWAN device EUI and application key
static const u1_t PROGMEM DEVEUI [8] = {0xd8, 0x07, 0xf3, 0xaf, 0x5a, 0x59, 0xd1, 0xde};
static const u1_t PROGMEM APPKEY[16] = {0xC8, 0xC2, 0x4C, 0x34, 0xF6, 0x26, 0xF1, 0xFF, 0x5A, 0x94, 0xE5, 0x73, 0x80, 0x5D, 0x4D, 0x76};

// Alternative DevEUI and AppKey from your comment (uncomment if needed)
/*
static const u1_t PROGMEM DEVEUI [8] = {0xcc, 0x0b, 0x2c, 0x9b, 0x80, 0x59, 0xfe, 0xa0};
static const u1_t PROGMEM APPKEY [16] = {0xb7, 0xa1, 0xd5, 0xf8, 0x6f, 0x27, 0x82, 0x99, 0x6e, 0xae, 0x46, 0x1f, 0x9e, 0x27, 0xad, 0x9d};
*/
static osjob_t sendjob;

void os_getArtEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevEui(u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey(u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

// Function to measure distance using HC-SR04
float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = duration * 0.034 / 2;  // Distance in cm
  if (distance > BIN_DEPTH || distance <= 0) {
    distance = BIN_DEPTH;  // Handle out-of-range readings
  }
  return distance;
}

// Function for sending data
void do_send(osjob_t* j) {
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println("OP_TXRXPEND, not sending");
  } else {
    // Measure distance and calculate fullness
    float distance = getDistance();
    int fullness = (int)((1.0 - (distance / BIN_DEPTH)) * 100);  // Fullness as percentage
    if (fullness < 0) fullness = 0;
    if (fullness > 100) fullness = 100;

    // Print fullness to Serial Monitor
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(" cm, Fullness: ");
    Serial.print(fullness);
    Serial.println(" %");

    // Prepare payload
    uint8_t payload[13];
    // Fullness (1 byte)
    payload[0] = (uint8_t)fullness;
    // Bin ID (4 bytes)
    payload[1] = (BIN_ID >> 24) & 0xFF;
    payload[2] = (BIN_ID >> 16) & 0xFF;
    payload[3] = (BIN_ID >> 8) & 0xFF;
    payload[4] = BIN_ID & 0xFF;
    // Latitude (4 bytes, float)
    uint8_t* latPtr = (uint8_t*)&LATITUDE;
    for (int i = 0; i < 4; i++) payload[5 + i] = latPtr[i];
    // Longitude (4 bytes, float)
    uint8_t* lonPtr = (uint8_t*)&LONGITUDE;
    for (int i = 0; i < 4; i++) payload[9 + i] = lonPtr[i];

    // Send payload
    LMIC_setTxData2(1, payload, sizeof(payload), 0);
    Serial.print("Payload size (bytes): ");
    Serial.println(sizeof(payload));
    Serial.println("Sending bin data...");
  }
}

void onEvent(ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission (every 2 minutes)
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(60*2), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      Serial.println(F("EV_RXCOMPLETE"));	
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("Starting smart bin node"));

  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  #ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  delay(60000);
  #endif

  // LMIC init
  os_init();
  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 10);
  LMIC_setLinkCheckMode(0);
  LMIC.dn2Dr = DR_SF9;
  LMIC_setDrTxpow(DR_SF7, 14);

  // Start job
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}