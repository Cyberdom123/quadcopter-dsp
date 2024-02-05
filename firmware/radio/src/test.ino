#include <RF24.h>

RF24 radio(9, 10); // CE, CSN

typedef union {
  float floatingPoint[6];
  byte binary[24];
} telemetry;


void setup(){
    Serial.begin(57600);
    Serial.setTimeout(1); 

    radio.begin();
    radio.setChannel(88);

    radio.setCRCLength(RF24_CRC_8);
    radio.setAutoAck(true);
    radio.setPALevel(RF24_PA_MAX, 1);

    radio.enableDynamicPayloads();
    radio.enableAckPayload();
    
    Serial.print('0');

    /* address for pipe 0 must be the same as tx addr 
        when ack payload are used */
    radio.openWritingPipe(0xc2c2c2c2c2LL);
    //Pipe in which the ACK payload will be stored -> pipe0

    /* Put radio in Tx mode */
    radio.stopListening();
    //radio.printDetails();
}

char txt_buff[20] = ""; 
uint8_t msg[8] = {0};
telemetry tele;
uint8_t pipe;
int timeout = 0;

void loop(){

    while (Serial.available() != 6); //check if bytes available for read 
    Serial.readBytes(msg, 6);
    //Serial.write(msg, 6);
    radio.write((char *) &msg, 8);

    //ACKpayload
    while(!radio.available(&pipe)){
        timeout++;
        if(timeout > 10){
            radio.flush_rx();
            break;
        }
    }
    timeout = 0;

    radio.read(tele.binary, 24);
    Serial.write(tele.binary, 24);
}
