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

void debug(const char* str){
  Serial.print("[");
  Serial.print(millis()/1000.0, 3);
  Serial.print("] ");
  Serial.println(str);
}

void moduleStatusMessage(const char* name, const char* msg){
  Serial.print("[");
  Serial.print(millis()/1000.0, 3);
  Serial.print("] ");
  Serial.print(name);
  Serial.print(": ");
  Serial.println(msg);
}

bool customRecv(uint8_t* buf, uint8_t* len) {
  if(true || rf95.available() && buf && len){
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

  rf95.setModeRx();
  rf95.spiWrite(0x1d, 0x39);  //BW: 125 kHz, CR: 4/8, implicit header
  //rf95.spiWrite(0x1e, 0xa4);  //SF: 10, no CRC
  //rf95.spiWrite(0x26, 0x0c);  //Mobile node, AGC on
  rf95.setPreambleLength(6);  //Awful waste of air time, but whatever...
  debug("Radio:   BW: 125 kHz, CR: 4/8, SF: 10,  AGC: on, CRC: off");
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
    uint32_t packet_ctr = buf[0] | (buf[1] << 8);
    Serial.println(packet_ctr);
  } else {
    //Serial.println("Nothing");
  }
}
