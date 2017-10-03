// rf95_client.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf95_server
// Tested with Anarduino MiniWirelessLoRa, Rocket Scream Mini Ultra Pro with
// the RFM95W, Adafruit Feather M0 with RFM95
//#define DEBUG   //If you comment this line, the DPRINT & DPRINTLN lines are defined as blank.
//test
//#define DEBUG
#ifdef DEBUG    //Macros are usually in all capital letters.
  #define DPRINT(...)    Serial.print(__VA_ARGS__)     //DPRINT is a macro, debug print
  #define DPRINTln(...)  Serial.println(__VA_ARGS__)   //DPRINTLN is a macro, debug print with new line
#else
  #define DPRINT(...)     //now defines a blank line
  #define DPRINTln(...)   //now defines a blank line
#endif



#include <SPI.h>
#include <RH_RF95.h>
#include "LowPower.h"
#include "avr/power.h" //to adjust clock speed

// Singleton instance of the radio driver
//RH_RF95 rf95;
RH_RF95 rf95(10, 2); // Select, interupt
//RH_RF95 rf95(8, 3); // Adafruit Feather M0 with RFM95

// Need this on Arduino Zero with SerialUSB port (eg RocketScream Mini Ultra Pro)
//#define Serial SerialUSB

//Function prototypes
long readVcc(void);
double GetTemp(void);
byte batteryVoltageCompress (long batvoltage);
const byte nodeID=1;
boolean ack =0;
const int sleepDivSixteen = 2; //sleep time divided by 16 (seconds)  75=20minutes
struct payloadDataStruct{
  byte nodeID;
  byte rssi;
  byte voltage;
}rxpayload;

payloadDataStruct txpayload;


byte tx_buf[sizeof(txpayload)] = {0};

void setup()
{
  txpayload.nodeID=nodeID;
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);  //reset pin

  delay(5000);
  clock_prescale_set(clock_div_2); // This divides the clock by 2


  Serial.begin(115200);//note if the main clock speed is slowed the baud will change
  while (!Serial) ; // Wait for serial port to be available
  if (!rf95.init())
    DPRINTln("init failed");
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:

rf95.setTxPower(15, false);
DPRINTln("init ok");
rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096);//th

rf95.printRegisters(); //th
}

void loop()
{
//DPRINT(millis()/1000);
//DPRINT(" voltage="); DPRINT( readVcc(), DEC );
 //DPRINT(" temp=");DPRINT(GetTemp(),1);

  //DPRINT("  Sending...   ");
  // Send a message to rf95_server
  delay(10);
  byte absrssi = abs(rf95.lastRssi());
  txpayload.rssi = absrssi;
  txpayload.voltage=batteryVoltageCompressed(readVcc());
  memcpy(tx_buf, &txpayload, sizeof(txpayload) );
  byte zize=sizeof(txpayload);
  DPRINT("sizeof data =  ");DPRINT(sizeof(txpayload));
  rf95.send((uint8_t *)tx_buf, zize);

//uint8_t data[] = "Hello";
//uint8_t data[3];
//data[0]= (uint8_t)"2";


  delay(500);
//  rf95.send(data, sizeof(data));

  rf95.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  //DPRINT("available "); DPRINTln(rf95.available());//th
  if (rf95.waitAvailableTimeout(4000))
  {
    // Should be a reply message for us now
    if (rf95.recv(buf, &len))
   {
      DPRINT(" got reply ");
      //rf95.printBuffer("Got:", buf, len);
      memcpy(&rxpayload, buf, sizeof(rxpayload));
      DPRINT(" remote voltage = ");DPRINT(rxpayload.voltage);
      DPRINT(" remote rssi = ");DPRINT(rxpayload.rssi);
      DPRINT("    local RSSI = ");
      DPRINT(rf95.lastRssi(), DEC);
      DPRINT("    local snr = ");DPRINTln(rf95.lastSNR(), DEC);
       //DPRINTln(rf95.frequencyError());

    }
    else
    {
      DPRINTln("recv failed");
    }
  }
  else
  {
    DPRINTln("No reply");

  }
  //delay(10000);
  rf95.sleep();
  delay(10);

  for (int i=0; i < sleepDivSixteen; i++){
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_ON);//main clock slowed, therefore 16s
  //LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_ON);
  }
}


long readVcc() { long result; // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert while (bit_is_set(ADCSRA,ADSC));
  result = ADCL; result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

double GetTemp(void)
{
  unsigned int wADC;
  double t;

  // The internal temperature has to be used
  // with the internal reference of 1.1V.
  // Channel 8 can not be selected with
  // the analogRead function yet.

  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);  // enable the ADC

  delay(20);            // wait for voltages to become stable.

  ADCSRA |= _BV(ADSC);  // Start the ADC

  // Detect end-of-conversion
  while (bit_is_set(ADCSRA,ADSC));

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  wADC = ADCW;

  // The offset of 324.31 could be wrong. It is just an indication.
  t = (wADC - 324.31 ) / 1.22;

  // The returned temperature is in degrees Celsius.
  return (t);
}

byte batteryVoltageCompress (long batvoltage) {
long result2;
result2 =  (batvoltage - 1000L)/10;
return (byte)(result2);
}
