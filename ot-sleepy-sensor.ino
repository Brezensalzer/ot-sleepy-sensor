//---------------------------------------------------------------------------
// ot-sleepy-sensor.ino
//---------------------------------------------------------------------------
// we want a sleepy end device
#define OPENTHREAD_MTD 1
#define OPENTHREAD_MTD_SED 1

#include <bluefruit.h>
#include <UdpSocket.h>
#include <IPAddress.h>
#include <OpenThread.h>

OTErr err;

// save power without serial interface...
//#define DEBUG

const uint8_t CHANNEL = 11;
const char PSK[] = "J01NME";
const uint8_t EXTPANID[] = {0x11, 0x11, 0x11, 0x11, 0x22, 0x22, 0x22, 0x22};

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

//--- MS8607 temperature, humidity and pressure sensor ---
#include <Wire.h>
#include <Adafruit_MS8607.h>
#include <Adafruit_Sensor.h>
// define the GPIO pin used as i2c vcc
#define VCC_I2C 10

//---- ADC Battery Monitoring ----------------------------
const int adcPin = A5;

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
void initOpenThread()
//---------------------------------------------------------------------------
{
  boolean JOIN_MODE = false;
  
  // Initialize OpenThread ----------------------
  OpenThread.init();
  #ifdef DEBUG
    Serial.println("init finished");
  #endif
  OpenThread.begin();
  OpenThread.txpower(8);  // max power for max range

  // -----------------------------------------------
  // Configure Sleepy End Device, see Device Modes: 
  // https://software-dl.ti.com/simplelink/esd/simplelink_cc26x2_sdk/1.60.00.43/exports/docs/thread/html/thread/ot-stack-overview.html
  // -----------------------------------------------
  otLinkModeConfig cfg;
  cfg.mRxOnWhenIdle = false;        // no "r"
  cfg.mSecureDataRequests = true;   // "s"
  cfg.mDeviceType = false;          // no "d"
  cfg.mNetworkData = false;         // no "n"
  OpenThread.mode(cfg);             // sleepy end device
  
  OpenThread.ifconfig.up();

  #ifdef DEBUG
    Serial.print("channel  = ");
    Serial.println(OpenThread.channel() );
    Serial.print("extpanid = 0x");
    Serial.println(OpenThread.extpanid() );
    Serial.print("extaddr = ");
    Serial.println(OpenThread.extaddr() );
    Serial.print("eui64 = ");
    Serial.println(OpenThread.eui64());
    Serial.print("networkname = ");
    Serial.println(OpenThread.networkname());
    cfg = OpenThread.mode();
    Serial.print("mode.mRxOnWhenIdle = ");
    Serial.println(cfg.mRxOnWhenIdle);
    Serial.print("mode.mSecureDataRequests = ");
    Serial.println(cfg.mSecureDataRequests);
    Serial.print("mode.mDeviceType = ");
    Serial.println(cfg.mDeviceType);
    Serial.print("mode.mNetworkData = ");
    Serial.println(cfg.mNetworkData);
  #endif

  // Check if we have joined a network. If false, 
  // the default networkname is "OpenThread"
  if (strcmp(OpenThread.networkname(),"OpenThread") == 0) {
    JOIN_MODE = true;
    #ifdef DEBUG
      Serial.print("join mode active: ");
      Serial.println(JOIN_MODE);
    #endif
  }
  
  // code for joining a Thread network
  // we need an active commissioner on the network!
  if (JOIN_MODE) {
    OpenThread.panid(0xFFFF);
    OpenThread.channel(CHANNEL);
    OpenThread.extpanid(EXTPANID);
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
    Serial.println("-----------------------");
      Serial.println("starting openthread...");
    #endif
    err = OpenThread.thread.start();    
  }
    err = OpenThread.thread.start();    

  #ifdef DEBUG
    Serial.println("network joined");
  #endif

  // check for child state, otherwise we are not ready
  while (OpenThread.state.child() == 0)
    delay(10);
}

//---------------------------------------------------------------------------
void setup()
//---------------------------------------------------------------------------
{
  #ifdef DEBUG
    Serial.begin(115200);
    while(!Serial) delay(10);

    Serial.println("-------------------------------");
    Serial.println("Starting OpenThread Sensor Node");
    Serial.println("-------------------------------");
  #endif
}

//---------------------------------------------------------------------------
void loop()
//---------------------------------------------------------------------------
{
  // losing 2.8 µA
  delay(29000);  // 29s

  //--- power on i2c vcc --------------------------------------------
  pinMode(VCC_I2C, OUTPUT);
  digitalWrite(VCC_I2C, HIGH);
  #ifdef DEBUG
    Serial.println("setting pin VCC_I2C to HIGH");
  #endif
  delay(200);

  // Initialize sensor --------------------------
  Wire.begin();
  Adafruit_MS8607 ms8607;
  
  if (!ms8607.begin()) {
    #ifdef DEBUG
      Serial.println("Failed to find MS8607 chip");
    #endif
    while (1) { delay(10); }
  }
  #ifdef DEBUG
    Serial.println("MS8607 Found!");
  #endif
  
  // read sensor data
  sensors_event_t temp, pressure, humidity;
  Adafruit_Sensor *pressure_sensor = ms8607.getPressureSensor();
  Adafruit_Sensor *temp_sensor = ms8607.getTemperatureSensor();
  Adafruit_Sensor *humidity_sensor = ms8607.getHumiditySensor();

  temp_sensor->getEvent(&temp);
  pressure_sensor->getEvent(&pressure);
  humidity_sensor->getEvent(&humidity);

  int bat = batteryLevel(); // ignore first measurement

  // sensor sleep 
  Wire.end();
  
  //--- power off i2c vcc --------------------------------------------
  digitalWrite(VCC_I2C, LOW);
  pinMode(VCC_I2C, INPUT);
  #ifdef DEBUG
    Serial.println("setting pin VCC_I2C to LOW");
  #endif

  // connect to network
  initOpenThread();

  // format JSONmessage
  message = "{\"ID\": \"itsybitsy-01\"";
  //message.concat(String(OpenThread.eui64.extaddr()));
  message.concat(",\"CurrentTemperature\": ");
  message.concat(temp.temperature);
  message.concat(",\"CurrentRelativeHumidity\": ");
  message.concat(humidity.relative_humidity);
  message.concat(",\"CurrentPressure\": ");
  message.concat(pressure.pressure); 
  message.concat(",\"BatteryLevel\": ");
  message.concat(batteryLevel());
  message.concat("}");
  message.concat("\n");
  
  #ifdef DEBUG
    Serial.print(message);
  #endif

  delay(1000); // this is needed, otherwise no udp packet is sent!
  // send packet immediately
  Udp.beginPacket(server, DEST_PORT);
  Udp.write(message.c_str(), message.length() );
  Udp.endPacket();
  Udp.flush();
  #ifdef DEBUG
    Serial.println("message sent!");
    Serial.flush();
  #endif
  delay(1000); // this is needed, otherwise no udp packet is sent!

  err = OpenThread.thread.stop();
  while (err)
    err = OpenThread.thread.stop();
  
  //--- brute force ----------------
  // otherwise we lose 274 µA
  NVIC_SystemReset();  
}
