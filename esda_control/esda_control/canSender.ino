                                                                                                                                                                                                                                                                                                                                                                              //==================================================================================//

#include <CAN.h>

#define TX_GPIO_NUM   21  // Connects to CTX
#define RX_GPIO_NUM   22  // Connects to CRX

//==================================================================================//

void setup() {
  Serial.begin (115200);
  while (!Serial);
  delay (1000);

  Serial.println ("CAN Receiver/Receiver");

  // Set the pins
  CAN.setPins (RX_GPIO_NUM, TX_GPIO_NUM);
            
  // start the CAN bus at 500 kbps
  if (!CAN.begin (500E3)) {
    Serial.println ("Starting CAN failed!");
    while (1);
  }
  else {
    Serial.println ("CAN Initialized");
  }
}

//==================================================================================//

void loop() {
  // canForwarderSender();
  canSender();
  // delay(1000);
  // canReceiver();
}

//==================================================================================//
void canForwarderSender() {
  if(Serial.available() > 0){
    // a = Serial.read()
    CAN.beginPacket(Serial.read());
    
    
    while(Serial.available() > 0) {
      CAN.write (Serial.read());
    }
    CAN.endPacket();
  }
}

void canBackwardReceiver(){
  



}

void canSender() {
  // send packet: id is 11 bits, packet can contain up to 8 bytes of data
  Serial.print ("Sending packet ... ");

  // This identifier can't been found on receiving python side 
  CAN.beginPacket(0x12);

  byte ID = 0b00000011;
  float data = 123.456;

  byte buffer[sizeof(byte) + sizeof(float)];

  memcpy(buffer, &ID, sizeof(byte));
  memcpy(buffer + sizeof(byte), &data, sizeof(float));

  CAN.write(buffer, sizeof(buffer));

  CAN.endPacket();

  //RTR packet with a requested data length
  // CAN.beginPacket (0x12, 3, true);
  // CAN.endPacket();

  Serial.println ("done");
  Serial.print(sizeof(buffer));
  Serial.print("Buffer content: ");
  for (size_t i = 0; i < sizeof(buffer); i++) {
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  Serial.println();

  delay (1000);
}

//==================================================================================//

void canReceiver() {
  // try to parse packet
  int packetSize = CAN.parsePacket();

  if (packetSize) {
    // received a packet
    Serial.print ("Received ");

    if (CAN.packetExtended()) {
      Serial.print ("extended ");
    }

    if (CAN.packetRtr()) {
      // Remote transmission request, packet contains no data
      Serial.print ("RTR ");
    }

    Serial.print ("packet with id 0x");
    Serial.print (CAN.packetId(), HEX);

    if (CAN.packetRtr()) {
      Serial.print (" and requested length ");
      Serial.println (CAN.packetDlc());
    } else {
      Serial.print (" and length ");
      Serial.println (packetSize);

      // only print packet data for non-RTR packets
      while (CAN.available()) {
        Serial.print ((char) CAN.read());
      }
      Serial.println();
    }

    Serial.println();
  }
}

//==================================================================================//
