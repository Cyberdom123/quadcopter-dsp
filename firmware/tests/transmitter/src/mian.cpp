
// #include "Arduino.h"
// #include <SPI.h>
// #include <RF24.h>
// #include "nRF24L01.h"
// #include "printf.h"

// RF24 radio(9,10);

// #define Size_of_Payload 18

// void setup(void)
// {

//   Serial.begin(57600);
//   printf_begin();

//   radio.begin();
//   radio.setChannel(23);
//   radio.setPayloadSize(Size_of_Payload);
//   //radio.setAutoAck(false);
//   radio.setCRCLength(RF24_CRC_8);
//   radio.openReadingPipe(0, 0xE7E7E7E7E7LL);

//   radio.startListening();

//   radio.printDetails();
// }

// void loop(void)
// {
//   if (radio.available()) {
//     char text[Size_of_Payload] = "";
//     radio.read(&text, sizeof(text));
//     Serial.println(text);
//   }
// }
#include "Arduino.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "printf.h"

RF24 radio(9, 10); // CE, CSN

uint16_t cnt = 0;

void setup() {
  Serial.begin(57600);
  Serial.setTimeout(1);
  printf_begin();

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

char msg[8] = "";
char text[33] = "";
uint8_t payload_size = 0;
uint8_t pipe;
void loop() {

  if(Serial.available()){
    Serial.readBytes(msg, 6);
    //Serial.write(msg, 6);
  }
  
  if(!radio.write(msg, 8))
  {
    //delay(1000);
    //radio.printDetails();
  }

  //ACKpayload
  if (radio.available(&pipe)) {
    radio.read(&text, sizeof(text));
    Serial.println(text);
  }
  // else{
  //   Serial.write("Received: an empty ACK packet"); 
  // }

  delay(10);
}