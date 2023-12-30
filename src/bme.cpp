#include <ESP32_LoRaWAN.h>
#include <CayenneLPP.h>
#include <SoftwareSerial.h>
#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <TinyGPS++.h>




/******************************* LORAWAN **********************************************/

//
//  These must be public
//
/* OTAA para*/
uint8_t DevEui[8] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x05, 0x74, 0x14 };
uint8_t AppEui[8] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0xAA, 0xB3 };
uint8_t AppKey[16] = { 0x30, 0x6B, 0x14, 0x59, 0xD6, 0x0C, 0xF8, 0xF2, 0xF6, 0x96, 0x6B, 0xF7, 0x66, 0xA3, 0x4F, 0x89 };
uint8_t NwkSKey [] = /* ABP  */ { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda, 0x85 };
uint8_t AppSKey [] = /* ABP  */ { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef, 0x67 };
uint32_t DevAddr   = /* ABP  */ (uint32_t) 0x007e6ae1;
uint16_t userChannelsMask [6] = { 0x00ff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };
LoRaMacRegion_t loraWanRegion = LORAMAC_REGION_EU868 /*LORAMAC_REGION_EU868*/;
DeviceClass_t loraWanClass = CLASS_A;
bool overTheAirActivation = true;
bool loraWanAdr = true;
bool isTxConfirmed = true;
uint8_t confirmedNbTrials = 8;
uint8_t appPort = 2;

//
//  These can be static, however, you need to edit ./.pio/libdeps/heltec_wifi_lora_32_V2/ESP32_LoRaWAN/src/ESP32_LoRaWAN.h
//  and remove the "extern uint32_t appTxDutyCycle;" line. Typical HelTec amateur hour crap. They don't use this anywhere
//  in their code.
//
/*license for Heltec ESP32 LoRaWan, quary your ChipID relevant license: http://resource.heltec.cn/search */
uint32_t license[4] = {0x9C5AA386,0x967D0549,0x333F6D6F,0x807DB52F}; // (uint32_t) LORAWAN_LICENSE;

static uint8_t debugLevel = 0;
static uint32_t appTxDutyCycle = 60000;


// Sensor values

float pm25 = 0.0;
float pm10 = 0.0;
float temperature = 0.0;
float humidity = 0.0;
float airpressure = 0.0;
float gas = 0.0;


//
//  Set up the data to be sent to 
//  TTN
//

CayenneLPP lpp(128);


/******************************* LORAWAN **********************************************/

/******************************* GPS     **********************************************/

// The TinyGPS++ object
TinyGPSPlus gps;

#define GPSTX 37              //pin number for TX output from ESP32 - RX into GPS
#define GPSRX 36              //pin number for RX input into ESP32 - TX from GPS

#define GPSserial Serial2     //define GPSserial as ESP32 Serial2 

static const uint32_t GPSBaud = 9600;

///////////////////////////////////////////////////
//Some utilities for going into low power mode
TimerEvent_t sleepTimer;
//Records whether our sleep/low power timer expired
bool sleepTimerExpired;

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }

  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}

static void wakeUp()
{
  sleepTimerExpired=true;
}


static void lowPowerSleep(uint32_t sleeptime)
{
  sleepTimerExpired=false;
  TimerInit( &sleepTimer, &wakeUp );
  TimerSetValue( &sleepTimer, sleeptime );
  TimerStart( &sleepTimer );
  //Low power handler also gets interrupted by other timers
  //So wait until our timer had expired
//  while (!sleepTimerExpired) lowPowerHandler();
  TimerStop( &sleepTimer );
}

static void read_gps() {
    static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

  Serial.println(F("by Mikal Hart"));
  Serial.println();
  Serial.println(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  Serial.println(F("           (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
  Serial.println(F("----------------------------------------------------------------------------------------------------------------------------------------"));

  while (Serial2.available() > 0){
      gps.encode(Serial2.read());
    }

  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);

  unsigned long distanceKmToLondon =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON) / 1000;
  printInt(distanceKmToLondon, gps.location.isValid(), 9);

  double courseToLondon =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON);

  printFloat(courseToLondon, gps.location.isValid(), 7, 2);

  const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon);

  printStr(gps.location.isValid() ? cardinalToLondon : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  Serial.println();
  
  smartDelay(1000);

}

/******************************* GPS     **********************************************/

static void prepareTxFrame( uint8_t port )
{
  CayenneLPP *lpp = new CayenneLPP( LORAWAN_APP_DATA_MAX_SIZE );

  lpp->reset();
  lpp->addTemperature(0, temperature);
  lpp->addBarometricPressure(0, airpressure);
  lpp->addRelativeHumidity(0, humidity);
  lpp->addAnalogInput( 1, pm25 );
  lpp->addAnalogInput( 2, pm10);
  lpp->addAnalogInput( 3, gas);
  lpp->addGPS(0, gps.location.lat(), gps.location.lng(), gps.altitude.meters());

  appDataSize = lpp->copy( appData );

  delete lpp;
}
/******************************* Air Quality Sensor **********************************************/

#define SDSTX 38              //pin number for TX output from ESP32 - RX into GPS
#define SDSRX 39              //pin number for RX input into ESP32 - TX from GPS

#define SDSserial Serial1     //define GPSserial as ESP32 Serial2 

void run_aq_sensor() {
}


/******************************* Air Quality Sensor **********************************************/


/******************************* BME680 Sensor **********************************************/

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme;

void run_bme_sensor() {

  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");

  Serial.print("Gas = ");
  Serial.print(bme.gas_resistance);
  Serial.println(" KOhms");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  temperature = bme.temperature;
  humidity = bme.humidity;
  gas = bme.gas_resistance;
  airpressure = bme.pressure / 100.0;


  Serial.println();
}

/******************************* BME680 Sensor **********************************************/
void setup ()
{
  Serial.begin(115200);
  while (!Serial);
  GPSserial.begin(9600, SERIAL_8N1, GPSTX, GPSRX);           //format is baud, mode, UART RX data, UART TX data 
  SDSserial.begin(9600, SERIAL_8N1, SDSTX, SDSRX);           //format is baud, mode, UART RX data, UART TX data 

  Wire1.begin(SDA, SCL);


SPI.begin(SCK,MISO,MOSI,SS);
//  if(mcuStarted==0)
//  {
//    LoRaWAN.displayMcuInit();
//  }
  Mcu.init(SS,RST_LoRa,DIO0,DIO1,license);
  deviceState = DEVICE_STATE_INIT;

  if (!bme.begin(0x76, &Wire1)) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    while (1);
  } else {
      Serial.println("Found BME680 at 0x76");
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
                              //
}


void loop ()
{
    switch (deviceState)
    {
        case DEVICE_STATE_INIT :
            {
                Serial.println ("DEVICE_STATE_INIT");
                LoRaWAN.init (loraWanClass, loraWanRegion);
            }
            break;

        case DEVICE_STATE_JOIN :
            {
                Serial.println ("DEVICE_STATE_JOIN");
                LoRaWAN.displayJoining();
                LoRaWAN.join();
            }
            break;

        case DEVICE_STATE_SEND :
            {
               // LoRaWAN.displaySending();

                run_aq_sensor();
                run_bme_sensor();
                read_gps();
                prepareTxFrame( appPort );
                LoRaWAN.send(loraWanClass);
                deviceState = DEVICE_STATE_CYCLE;
            }
            break;

        case DEVICE_STATE_CYCLE :
            {
                txDutyCycleTime = appTxDutyCycle + randr (-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
                LoRaWAN.cycle (txDutyCycleTime);
                deviceState = DEVICE_STATE_SLEEP;
            }
            break;

        case DEVICE_STATE_SLEEP :
            {
                LoRaWAN.sleep (loraWanClass, debugLevel);
            }
            break;

        default:
            {
                deviceState = DEVICE_STATE_INIT;
                break;
            }
    }

}
