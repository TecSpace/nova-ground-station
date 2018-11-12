#include <RH_RF95.h>
#include <SPI.h>

#define RFM95_CS  10
#define RFM95_RST 9
#define RFM95_INT 2

RH_RF95 rf95(RFM95_CS, RFM95_INT);

uint8_t buf[35] = {};
uint8_t len     = sizeof(buf);

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

bool customRecv(uint8_t* buf, uint8_t* len) {
  if(rf95.available() && buf && len){
    ATOMIC_BLOCK_START;
      memcpy(buf, rf95._buf, *len);
    ATOMIC_BLOCK_END;
    rf95.clearRxBuf();
    return true;
  } else {
    return false;
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

  rf95.spiWrite(0x1d, 0x68);  //BW: 62.5 kHz, CR: 4/8, explicit header
  rf95.spiWrite(0x1e, 0x94);  //SF: 9
  rf95.spiWrite(0x26, 0x0c);  //TBD

  rf95.setPreambleLength(6);  //Awful waste of air time, but whatever...
  rf95.setModeRx();

  debug("Radio:   BW: 125 kHz, CR: 4/8, SF: 10, AGC: on, CRC: off");
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
    uint32_t packet_ctr  = buf[0]  | (buf[1] << 8);
    uint32_t time_boot   = buf[2]  | (buf[3] << 8) | (buf[4] << 16);
    uint16_t temperature = buf[5]  | (buf[6] << 8);
    uint32_t pressure    = buf[7]  | (buf[8] << 8) | (buf[9] << 16);
    int16_t  accX        = buf[10] | (buf[11] << 8);
    int16_t  accY        = buf[12] | (buf[13] << 8);
    int16_t  accZ        = buf[14] | (buf[15] << 8);
    int32_t  latitude    = buf[16] | (buf[17] << 8) | (buf[18] << 16) | (buf[19] << 24);
    int32_t  longitude   = buf[20] | (buf[21] << 8) | (buf[22] << 16) | (buf[23] << 24);
    int32_t  altitude    = buf[24] | (buf[25] << 8) | (buf[26] << 16);
    uint32_t speed       = buf[27] | (buf[28] << 8) | (buf[29] << 16);
  } else {
    //Serial.println("Nothing");
  }
}
