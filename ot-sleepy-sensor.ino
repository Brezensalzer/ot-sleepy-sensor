//---------------------------------------------------------------------------
// ot-sleepy-sensor.ino
//---------------------------------------------------------------------------
// we want a sleepy end device
#define OPENTHREAD_MTD 1
#define OPENTHREAD_MTD_SED 1
#define OPENTHREAD_FTD 0
#define MAIN_STACK_SIZE 2048

#include <bluefruit.h>
#include <UdpSocket.h>
#include <IPAddress.h>
#include <OpenThread.h>

// save power without serial interface...
const boolean DEBUG = true;

const uint8_t CHANNEL = 11;
const char PSK[] = "J01NME";
const uint8_t EXTPANID[] = {0x11, 0x11, 0x11, 0x11, 0x22, 0x22, 0x22, 0x22};
//const uint8_t EXTPANID[] = {0xDE, 0xAD, 0x00, 0xBE, 0xEF, 0x00, 0xCA, 0xFE};

// Mesh-local multicast address ff03::1
// we make sure the message reaches the thread border router
IPAddress server(0xff03, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0001); // multicast address

UDPSocket Udp;

const size_t MAX_PAYLOAD_LEN = 40;
const unsigned short DEST_PORT  = 4711;
const int INTERVAL = 5;

String message;
char recvBuffer[MAX_PAYLOAD_LEN];
long lastsend = INTERVAL * -1000;
uint16_t seq_id = 0;

//--- BMP85 temperature and pressure sensor ---
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
Adafruit_BMP085 bmp;

//---- Button for JOIN Mode ------------------------------
int buttonPin = 4;  // on board switch

//---- ADC Battery Monitoring ----------------------------
const int adcPin = A5;

// Create a SoftwareTimer that will drive our i2c sensor.
SoftwareTimer i2c_timer;

//---------------------------------------------------------------------------
void signalLED() {
//---------------------------------------------------------------------------
  digitalWrite(3, HIGH);
  delay(2000);
  digitalWrite(3, LOW);
}

//------------------------------------------------------------------------------
uint8_t batteryLevel()
//------------------------------------------------------------------------------
{
  int adcValue = analogRead(adcPin);
  // 1024 = 3.6V, 768 = 2.7V cut off voltage for LiFePO4 batteries
  uint8_t value = map(adcValue, 768, 1024, 0, 100);
  return value;  
}

//---------------------------------------------------------------------------
void setup() {
//---------------------------------------------------------------------------
  if (DEBUG) {
    Serial.begin(115200);
    while(!Serial) delay(10);
  }
  if (DEBUG) Serial.println("-------------------------------");
  if (DEBUG) Serial.println("Starting OpenThread Sensor Node");
  if (DEBUG) Serial.println("-------------------------------");

  // Initialize switch for joining
  pinMode(buttonPin, INPUT_PULLUP);
  signalLED();
  boolean JOIN_MODE = !digitalRead(buttonPin);
  if (DEBUG) Serial.print("join mode active: ");
  if (DEBUG) Serial.println(JOIN_MODE);
  if (DEBUG) Serial.flush();
  
  // Initialize OpenThread ----------------------
  OpenThread.init();
  if (DEBUG) Serial.println("init finished");
  if (DEBUG) Serial.flush();
  OpenThread.begin();
  if (JOIN_MODE) {
    OpenThread.panid(0xFFFF);
    OpenThread.channel(CHANNEL);
    OpenThread.extpanid(EXTPANID);
  } 
  //OpenThread.routereligible.disable();
  OpenThread.txpower(8);
  OpenThread.ifconfig.up();

  if (DEBUG) {
    Serial.print("channel  = ");
    Serial.println(OpenThread.channel() );
    Serial.print("extpanid = 0x");
    Serial.println(OpenThread.extpanid() );
    Serial.print("eui64 = ");
    Serial.println(OpenThread.eui64());
  }

  OTErr err;

  if (JOIN_MODE) {
    do {
      if (DEBUG) Serial.println("starting joiner...");
      if (DEBUG) Serial.flush();
      err = OpenThread.joiner.start(PSK);
  
      if (err) {
        if (DEBUG) Serial.print("...");
        if (DEBUG) Serial.print(err);
        delay(100);
        continue;
      }
  
      if (DEBUG) Serial.println("starting openthread...");
      if (DEBUG) Serial.flush();
      err = OpenThread.thread.start();
  
      if(err) {
        if (DEBUG) Serial.print("Thread process can't start: ");
        if (DEBUG) Serial.println(err);
        delay(100);
        continue;
      }
    } while(err != 0);
  }
  else {
    if (DEBUG) Serial.println("starting openthread...");
    err = OpenThread.thread.start();    
  }
    err = OpenThread.thread.start();    

  if (DEBUG) {
    Serial.println("-----------------------");
    Serial.println("network joined");
    Serial.println("-----------------------");
  }

  //--- setup timer ---------------------------------------------------
  // Set up a repeating timer that fires every 60 seconds (60000ms) to read the i2c sensor.
  i2c_timer.begin(30000, timer_callback);
  i2c_timer.start();

  if (DEBUG) Serial.println("started timer, suspending loop.");
  // Since loop() is empty, suspend its task so that the system never runs it
  // and can go to sleep properly.
  suspendLoop();
  
} // end setup()

//---------------------------------------------------------------------------
// the timer callback should be as short as possible
// Serial.print is a rather bad idea...
void timer_callback(TimerHandle_t _handle) {
//---------------------------------------------------------------------------
  /* start sensor */
  if (DEBUG) Serial.print(".");
  if (!bmp.begin(BMP085_ULTRAHIGHRES)) {
    if (DEBUG) Serial.println("error initializing i2c sensor");
  }

  float pressure    = bmp.readPressure();
  float temperature = bmp.readTemperature();
/*    
  if (DEBUG) {
    Serial.print("Pressure: "); Serial.println(pressure);
    Serial.print("Temperature: "); Serial.println(temperature);
  }
*/
  int bat = batteryLevel(); // ignore first measurement

  // format JSONmessage
  message = "{ID: sleepyItsyBitsy,";
  message.concat("CurrentTemperature: ");
  message.concat(temperature);
  message.concat(",CurrentPressure: ");
  message.concat(pressure);
  message.concat(",BatteryLevel: ");
  message.concat(batteryLevel());
  message.concat(",Alive: ");
  message.concat(++seq_id);
  message.concat("}");
/*
  if (DEBUG) {
    Serial.print("Send to [");
    Serial.print(server);
    Serial.printf("]:%d -> '", DEST_PORT);
    Serial.print(message);
    Serial.println("'");
  }
*/
  message.concat("\n");
  
  // send packet
  Udp.beginPacket(server, DEST_PORT);
  Udp.write(message.c_str(), message.length() );
  Udp.endPacket();

    /* sensor sleep */
  bmp.begin(BMP085_ULTRALOWPOWER);
}

// orphaned loop
void loop() {}
