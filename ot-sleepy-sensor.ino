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
//#define DEBUG

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
  #ifdef DEBUG
    Serial.begin(115200);
    while(!Serial) delay(10);

    Serial.println("-------------------------------");
    Serial.println("Starting OpenThread Sensor Node");
    Serial.println("-------------------------------");
  #endif
  
  // Initialize switch for joining
  pinMode(buttonPin, INPUT_PULLUP);
  signalLED();
  boolean JOIN_MODE = !digitalRead(buttonPin);
  #ifdef DEBUG
    Serial.print("join mode active: ");
    Serial.println(JOIN_MODE);
    Serial.flush();
  #endif
  
  // Initialize OpenThread ----------------------
  OpenThread.init();
  #ifdef DEBUG
    Serial.println("init finished");
    Serial.flush();
  #endif
  OpenThread.begin();
  if (JOIN_MODE) {
    OpenThread.panid(0xFFFF);
    OpenThread.channel(CHANNEL);
    OpenThread.extpanid(EXTPANID);
  } 
  //OpenThread.routereligible.disable();
  OpenThread.txpower(8);
  OpenThread.ifconfig.up();

  #ifdef DEBUG
    Serial.print("channel  = ");
    Serial.println(OpenThread.channel() );
    Serial.print("extpanid = 0x");
    Serial.println(OpenThread.extpanid() );
    Serial.print("eui64 = ");
    Serial.println(OpenThread.eui64());
  #endif

  OTErr err;

  if (JOIN_MODE) {
    do {
      #ifdef DEBUG
        Serial.println("starting joiner...");
        Serial.flush();
      #endif
      err = OpenThread.joiner.start(PSK);
  
      if (err) {
        #ifdef DEBUG
          Serial.print("...");
          Serial.print(err);
        #endif
        delay(100);
        continue;
      }

      #ifdef DEBUG
        Serial.println("starting openthread...");
        Serial.flush();
      #endif
      err = OpenThread.thread.start();
  
      if(err) {
        #ifdef DEBUG
          Serial.print("Thread process can't start: ");
          Serial.println(err);
        #endif
        delay(100);
        continue;
      }
    } while(err != 0);
  }
  else {
    #ifdef DEBUG 
      Serial.println("starting openthread...");
    #endif
    err = OpenThread.thread.start();    
  }
    err = OpenThread.thread.start();    

  #ifdef DEBUG
    Serial.println("-----------------------");
    Serial.println("network joined");
    Serial.println("-----------------------");
  #endif

  //--- setup timer ---------------------------------------------------
  // Set up a repeating timer that fires every 60 seconds (60000ms) to read the i2c sensor.
  i2c_timer.begin(30000, timer_callback);
  i2c_timer.start();

  #ifdef DEBUG 
    Serial.println("started timer, suspending loop.");
  #endif
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
  #ifdef DEBUG
    Serial.print(".");
  #endif
  if (!bmp.begin(BMP085_ULTRAHIGHRES)) {
    #ifdef DEBUG
      if (DEBUG) Serial.println("error initializing i2c sensor");
    #endif
  }

  float pressure    = bmp.readPressure();
  float temperature = bmp.readTemperature();
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
