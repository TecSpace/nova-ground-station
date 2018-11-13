#include <ArduinoJson.h>
#include <Base64.h>
#include <RH_RF95.h>
#include <SPI.h>

#define RFM95_CS  10
#define RFM95_RST 9
#define RFM95_INT 2

RH_RF95 rf95(RFM95_CS, RFM95_INT);

uint8_t buf[35]        = {};
uint8_t bufHex[71]     = {};
uint8_t len            = sizeof(buf);

const size_t bufferSize = JSON_OBJECT_SIZE(18);
DynamicJsonBuffer jsonBuffer(bufferSize);
JsonObject& root = jsonBuffer.createObject();

const char* START_MODULE   = "Starting module";
const char* SUCCESS_MODULE = "Module initialized";
const char* FAILURE_MODULE = "Could not initialize module";

void printTimeLead(){
  Serial.print("[");
  Serial.print(millis()/1000.0, 3);
  Serial.print("] ");
}

void debug(const char* str){
  printTimeLead();
  Serial.println(str);
}


void moduleStatusMessage(const char* name, const char* msg){
  printTimeLead();
  Serial.print(name);
  Serial.print(": ");
  Serial.println(msg);
}

int bufToHex(uint8_t *src, size_t len, uint8_t *dst){
  const char* alphabet = "0123456789ABCDEF";
  for(size_t i = 0; i < len; i++) {
    dst[i * 2]        = alphabet[(src[i] >> 4) & 0xF]; //Higher nibble first
    dst[(i * 2) + 1]  = alphabet[src[i] & 0xF];
  }
  dst[len*2] = 0; //Null terminator
}

void initRadioSubsystem() {
  moduleStatusMessage("Radio", START_MODULE);
 
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  if(!rf95.init()) {
    debug("Radio: module init failed - halting");
    while(1) {;}
  }

  if(!rf95.setFrequency(915.00)) {
    debug("Radio:   Could not set frequency to 915.00 MHz");
    while(1) {;}
  } else {
    debug("Radio:   Frequency: 915.00 MHz");
  }
  
  //rf95.setModemConfig(2);
  //rf95.spiWrite(0x1d, 0x78);  //BW: 62.5 kHz, CR: 4/8, explicit header
  //rf95.spiWrite(0x1e, 0x94);  //SF: 9
  //rf95.spiWrite(0x26, 0x0c);  //TBD

  rf95.setPreambleLength(6);  //Awful waste of air time, but whatever...
  rf95.setModeRx();

  debug("Radio:   BW: 62.5 kHz, CR: 4/8, SF: 10, AGC: on, CRC: off");
  debug("Radio:   Preamble length: 6 symbols");

  moduleStatusMessage("Radio", SUCCESS_MODULE);
}

void setup() {
  Serial.begin(115200);
  debug("Hello world! This is your local friendly ground station!");
  initRadioSubsystem();
}

void loop() {
  if(rf95.recv(buf, &len)){
    uint32_t packet_ctr  = buf[0]  | ((uint32_t) buf[1]  << 8);
    uint32_t time_boot   = buf[2]  | ((uint32_t) buf[3]  << 8) | ((uint32_t) buf[4]  << 16);
    uint16_t temperature = buf[5]  | ((uint32_t) buf[6]  << 8);
    uint32_t pressure    = buf[7]  | ((uint32_t) buf[8]  << 8) | ((uint32_t) buf[9]  << 16);
    int16_t  accX        = buf[10] | ((uint32_t) buf[11] << 8);
    int16_t  accY        = buf[12] | ((uint32_t) buf[13] << 8);
    int16_t  accZ        = buf[14] | ((uint32_t) buf[15] << 8);
    int32_t  latitude    = buf[16] | ((uint32_t) buf[17] << 8) | ((uint32_t) buf[18] << 16) | ((uint32_t) buf[19] << 24);
    int32_t  longitude   = buf[20] | ((uint32_t) buf[21] << 8) | ((uint32_t) buf[22] << 16) | ((uint32_t) buf[23] << 24);
    int32_t  altitude    = buf[24] | ((uint32_t) buf[25] << 8) | ((uint32_t) buf[26] << 16);
    uint32_t speed       = buf[27] | ((uint32_t) buf[28] << 8) | ((uint32_t) buf[29] << 16);
    int16_t  magX        = 0;
    int16_t  magY        = 0;
    int16_t  magZ        = 0;
    uint8_t  satellites  = (buf[34] >> 4) & 0xF;
    bool     isValidLoc  = buf[34] >> 7;
    int64_t  lastSNR     = rf95._lastRssi;

    bufToHex(buf, len, bufHex);

    root["packet_ctr"]  = packet_ctr;
    root["time_boot"]   = time_boot;
    root["temperature"] = temperature;
    root["pressure"]    = pressure;
    root["accX"]        = accX;
    root["accY"]        = accY;
    root["accZ"]        = accZ;
    root["latitude"]    = latitude;
    root["longitude"]   = longitude;
    root["altitude"]    = altitude;
    root["speed"]       = speed;
    root["magX"]        = magX;
    root["magY"]        = magY;
    root["magZ"]        = magZ;
    root["satellites"]  = satellites;
    root["isValidLoc"]  = isValidLoc;
    root["lastSNR"]     = lastSNR;
    root["bufHex"]      = bufHex;
    root.printTo(Serial);
    Serial.println();
  } 
}
