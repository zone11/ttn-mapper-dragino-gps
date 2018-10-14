#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const u1_t NWKSKEY[16] = { 0x60, 0x86, 0xFE, 0x39, 0x3F, 0x4F, 0x84, 0xF1, 0x9A, 0xD0, 0x5C, 0xB8, 0x17, 0x22, 0x64, 0xE6 };
static const u1_t APPSKEY[16] = { 0x86, 0x0A, 0xFC, 0x3A, 0x5D, 0xBC, 0x08, 0xC4, 0x67, 0xD5, 0x2C, 0x38, 0xE0, 0x8B, 0x5D, 0x5D };
static const u4_t DEVADDR = 0x26051340;

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }


// SoftwareSerial pins and speed for GPS
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// LMIC pins for DraginoShield
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 6, 7},
};

String toLog;
uint8_t txBuffer[9];
uint8_t txBufferSimple[1];
uint32_t LatitudeBinary, LongitudeBinary;
uint16_t altitudeGps;
uint8_t hdopGps;

// LED related
unsigned long ledstart = 0;
boolean ledLORA = false;
boolean ledGPS = false;

// Mode related
boolean simpleMode = false;
unsigned int TX_INTERVAL = 10;

// IO definitions
#define BATTERY A0
#define LEDLORAPIN A1
#define LEDGPSPIN A2
#define LEDALERT A3
#define SWSPEED A4
#define SWMODE A5

// GPS support
TinyGPSPlus gps;

// SoftwareSerial for GPS
SoftwareSerial ss(RXPin, TXPin);

// Some stateds for GPS data and sendjob
bool validDataTime = false;
bool validDataDate = false;
bool validDataPos = false;
static osjob_t sendjob;

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));

      // use SF12 for maximum range
      LMIC_setDrTxpow(DR_SF9, 14);

      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
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
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    SetGPSData();

  if (simpleMode) {
    txBufferSimple[0] = 0xAB;
    LMIC_setTxData2(2, txBufferSimple, sizeof(txBufferSimple), 0);
    ledLORA = true;
  } else {
    if (validDataTime == true && validDataPos == true)
    {
      // Prepare upstream data transmission at the next possible time.
      LMIC_setTxData2(1, txBuffer, sizeof(txBuffer), 0);
      Serial.println(F("Packet queued"));
      ledLORA = true;
    }
    else
    {
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
    }
  }
  }
}

void SetGPSData()
{
  validDataTime = false;
  validDataDate = false;
  validDataPos = false;
  ledGPS = false;

  if (gps.time.isValid()) {
    validDataTime = true;
  }
  
  if (gps.date.isValid()) {
    validDataDate = true;
  }

  if (gps.location.isValid()) {
    LatitudeBinary = ((gps.location.lat() + 90) / 180.0) * 16777215;
    LongitudeBinary = ((gps.location.lng() + 180) / 360.0) * 16777215;

    txBuffer[0] = ( LatitudeBinary >> 16 ) & 0xFF;
    txBuffer[1] = ( LatitudeBinary >> 8 ) & 0xFF;
    txBuffer[2] = LatitudeBinary & 0xFF;

    txBuffer[3] = ( LongitudeBinary >> 16 ) & 0xFF;
    txBuffer[4] = ( LongitudeBinary >> 8 ) & 0xFF;
    txBuffer[5] = LongitudeBinary & 0xFF;

    altitudeGps = gps.altitude.meters();
    txBuffer[6] = ( altitudeGps >> 8 ) & 0xFF;
    txBuffer[7] = altitudeGps & 0xFF;

    hdopGps = gps.hdop.value() / 10;
    txBuffer[8] = hdopGps & 0xFF;

    toLog = "";
    for (size_t i = 0; i < sizeof(txBuffer); i++) {
      char buffer[3];
      sprintf(buffer, "%02x", txBuffer[i]);
      toLog = toLog + String(buffer);
    }

    validDataPos = true;
    ledGPS = true;
  }

  displayInfo();
}

void displayInfo() {
  Serial.print(F("SimpleMode: "));
  Serial.print(simpleMode);

  Serial.print(F("  TXsec: "));
  Serial.print(TX_INTERVAL);
  
  Serial.print(F("  Location: "));
  if (validDataPos)
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (validDataDate)
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (validDataTime)
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    //Serial.print(F("."));
    //if (gps.time.centisecond() < 10) Serial.print(F("0"));
    //Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  ss.begin(9600);
  Serial.println(F("Starting"));

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  delay(500);

  // Set static session parameters.
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);

  // Start job
  do_send(&sendjob);

  // LED
  pinMode(LEDLORAPIN, OUTPUT);
  pinMode(LEDGPSPIN, OUTPUT);
  pinMode(LEDALERT, OUTPUT);
  digitalWrite(LEDLORAPIN, LOW);
  digitalWrite(LEDGPSPIN, LOW);
  digitalWrite(LEDALERT, LOW);

  // Switches
  pinMode(SWMODE, INPUT);
  pinMode(SWMODE, INPUT_PULLUP);
  pinMode(SWSPEED, INPUT);
  pinMode(SWSPEED, INPUT_PULLUP);

}

void loop() {
  // LMIC
  os_runloop_once();

  // GPS
  while (ss.available()) {
    gps.encode(ss.read());
  }

  // Switch SPEED
  if(!digitalRead(SWSPEED)) {
    TX_INTERVAL = 30;
  } else {
    TX_INTERVAL = 5;
  }
  
  // Switch MODE
  if(!digitalRead(SWMODE)) {
    simpleMode = false;
  } else {
    simpleMode = true;
  }
    
  // LED LORA
  if (ledLORA == true) {
    ledstart = millis();
    ledLORA = false;
    digitalWrite(LEDLORAPIN, HIGH);
  } else {
    if (millis() - ledstart > 400) {
      digitalWrite(LEDLORAPIN, LOW);
      ledstart = 0;
    }
  }

  // LED GPS
  if(ledGPS == true) {
    digitalWrite(LEDGPSPIN,HIGH);
  } else {
    digitalWrite(LEDGPSPIN,LOW);
  }
}
