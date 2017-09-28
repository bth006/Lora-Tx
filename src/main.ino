// LoRa 9x_TX
//esp8266
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example LoRa9x_RX

#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 4 //4=d2
#define RFM95_RST 15 //=D8  boots when discon
#define RFM95_INT 5 //=D1
//on esp :
 // gpio12=d6 = miso
 //gpio13=d7=mosi
 //gpio14=d5= clk
 //gpio 15=d8=cs


// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 433.0
#define RH_HAVE_SERIAL  //th debug only

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup()
{
yield();
delay(1000);

  //while (!Serial){
  Serial.begin(115200);
  yield();
  delay(1000);

  Serial.println("Arduino LoRa TX Test!"); yield();
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(1000);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
yield();

  delay(2000);
Serial.println("about to init");
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    yield();delay(500);yield();
  }
Serial.println("after init");
yield(); delay(10); yield();
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    yield(); delay(10); yield();
  }
yield(); delay(10); yield();
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(13, false);//th
  //rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096);//th
  rf95.printRegisters(); //th
}

int16_t packetnum = 0;  // packet counter, we increment per xmission

void loop()
{
yield(); delay(1); yield();
  Serial.println("Sending to rf95_server");
  // Send a message to rf95_server

  char radiopacket[20] = "Hello World #      ";
  itoa(packetnum++, radiopacket+13, 10);
  Serial.print("Sending "); Serial.println(radiopacket);
  radiopacket[19] = 0;

  Serial.println("Sending..."); delay(10);
  yield(); delay(1); yield();
  rf95.send((uint8_t *)radiopacket, 20);
yield(); delay(1); yield();
  Serial.println("Waiting for packet to complete..."); delay(10);
yield(); delay(1); yield();
  rf95.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  Serial.println("Waiting for reply..."); delay(10);
  yield(); delay(1); yield();
  if (rf95.waitAvailableTimeout(3000))//th was 1000
  {
    // Should be a reply message for us now
    yield(); delay(1); yield();
    if (rf95.recv(buf, &len))
   {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      yield(); delay(1); yield();
      Serial.println(rf95.lastRssi(), DEC);
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  else
  {
    Serial.println("No reply, is there a listener around?");
  }
  delay(1000);
}
