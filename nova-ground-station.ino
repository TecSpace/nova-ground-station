#include <ArduinoJson.h>
#include <Base64.h>
#include <RH_RF95.h>
#include <SPI.h>

#define RFM95_CS  10
#define RFM95_RST 9
#define RFM95_INT 2

RH_RF95 rf95(RFM95_CS, RFM95_INT);

uint8_t buf[20]        = {};
uint8_t bufHex[100]     = "";
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
  return;
  printTimeLead();
  Serial.println(str);
}

void moduleStatusMessage(const char* name, const char* msg){
  return;
  printTimeLead();
  Serial.print(name);
  Serial.print(": ");
  Serial.println(msg);
}

int bufToHex(uint8_t *src, uint8_t len, uint8_t *dst){
  memset(dst, 0, 100);
  const char* alphabet = "0123456789ABCDEF";
  for(size_t i = 0; i < len; i++) {
    char snum[5];
    itoa(src[i], snum, 16);
    strcat(dst, snum);
  }
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

  //rf95.setPreambleLength(6);  //Awful waste of air time, but whatever...
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
    uint32_t packetCtr   = buf[0]  | ((uint32_t) buf[1]  << 8);
    uint32_t timeBoot    = buf[2]  | ((uint32_t) buf[3]  << 8) | ((uint32_t) buf[4]  << 16);
    int32_t  latitude    = buf[5]  | ((uint32_t) buf[6] << 8) | ((uint32_t) buf[7] << 16) | ((uint32_t) buf[8] << 24);
    int32_t  longitude   = buf[9]  | ((uint32_t) buf[10] << 8) | ((uint32_t) buf[11] << 16) | ((uint32_t) buf[12] << 24);
    int32_t  altitude    = buf[13] | ((uint32_t) buf[14] << 8) | ((uint32_t) buf[15] << 16);
    uint32_t speed       = buf[16] | ((uint32_t) buf[17] << 8) | ((uint32_t) buf[18] << 16);
    uint8_t  satellites  = (buf[19] >> 4) & 0xF;
    bool     isValidLoc  = buf[19] >> 7;
    int16_t  lastSNR     = -133 + rf95.spiRead(0x1b);

    bufToHex(buf, len, bufHex);

    root["packetCtr"]   = packetCtr;
    root["timeBoot"]    = timeBoot;
    root["latitude"]    = latitude;
    root["longitude"]   = longitude;
    root["altitude"]    = altitude;
    root["speed"]       = speed;
    root["satellites"]  = satellites;
    root["isValidLoc"]  = isValidLoc;
    root["lastSNR"]     = lastSNR;
    root["bufHex"]      = bufHex;

    root.printTo(Serial);
    Serial.println();
  } 
}
