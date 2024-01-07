#include <ESP32_LoRaWAN.h>
#include <CayenneLPP.h>
#include <SoftwareSerial.h>
#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <TinyGPS++.h>
#include <esp_sds011.h>




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

int ext_pm25 = 0;
int ext_pm10 = 0;
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
    while (Serial2.available())
      gps.encode(Serial2.read());
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
  smartDelay(20);
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
  smartDelay(20);
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
  smartDelay(20);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(20);
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
  lpp->addAnalogInput( 1, ext_pm25 );
  lpp->addAnalogInput( 2, ext_pm10);
  lpp->addAnalogInput( 3, gas);
  lpp->addGPS(0, gps.location.lat(), gps.location.lng(), gps.altitude.meters());

  appDataSize = lpp->copy( appData );

  delete lpp;
}
/******************************* Air Quality Sensor **********************************************/

#define SDSTX 12              //pin number for TX output from ESP32 - RX into SDS011
#define SDSRX 13              //pin number for RX input into ESP32 - TX from SDS011

//#define SDSserial Serial1     //define SDSserial as ESP32 Serial1 
HardwareSerial& SDSserial(Serial1);
Sds011Async< HardwareSerial > sds011(SDSserial);

// The example stops the sensor for 210s, then runs it for 30s, then repeats.
// At tablesizes 20 and below, the tables get filled during duty cycle
// and then measurement completes.
// At tablesizes above 20, the tables do not get completely filled
// during the 30s total runtime, and the rampup / 4 timeout trips,
// thus completing the measurement at whatever number of data points
// were recorded in the tables.
constexpr int pm_tablesize = 20;
int pm25_table[pm_tablesize];
int pm10_table[pm_tablesize];
                              //
bool is_SDS_running = true;

void start_SDS() {
    Serial.println(F("Start wakeup SDS011"));

    if (sds011.set_sleep(false)) { is_SDS_running = true; }

    Serial.println(F("End wakeup SDS011"));
}

void stop_SDS() {
    Serial.println(F("Start sleep SDS011"));

    if (sds011.set_sleep(true)) { is_SDS_running = false; }

    Serial.println(F("End sleep SDS011"));
}

void run_aq_sensor() {
        // Per manufacturer specification, place the sensor in standby to prolong service life.
    // At an user-determined interval (here 210s down plus 30s duty = 4m), run the sensor for 30s.
    // Quick response time is given as 10s by the manufacturer, thus the library drops the
    // measurements obtained during the first 10s of each run.

    constexpr uint32_t down_s = 210;

    stop_SDS();
    Serial.print(F("stopped SDS011 (is running = "));
    Serial.print(is_SDS_running);
    Serial.println(')');

    uint32_t deadline = millis() + down_s * 1000;
    while (static_cast<int32_t>(deadline - millis()) > 0) {
        delay(1000);
        Serial.println(static_cast<int32_t>(deadline - millis()) / 1000);
        sds011.perform_work();
    }

    constexpr uint32_t duty_s = 30;

    start_SDS();
    Serial.print(F("started SDS011 (is running = "));
    Serial.print(is_SDS_running);
    Serial.println(')');

    sds011.on_query_data_auto_completed([](int n) {
        Serial.println(F("Begin Handling SDS011 query data"));
        Serial.print(F("n = ")); Serial.println(n);
        int pm25 = 0;
        int pm10 = 0;
        if (sds011.filter_data(n, pm25_table, pm10_table, pm25, pm10) &&
            !isnan(pm10) && !isnan(pm25)) {
            Serial.print(F("PM10: "));
            Serial.println(float(pm10) / 10);
            ext_pm10 = pm10;
            Serial.print(F("PM2.5: "));
            Serial.println(float(pm25) / 10);
            ext_pm25 = pm25;
        }
        Serial.println(F("End Handling SDS011 query data"));
        });

    if (!sds011.query_data_auto_async(pm_tablesize, pm25_table, pm10_table)) {
        Serial.println(F("SDS011 measurement capture start failed"));
    }

    deadline = millis() + duty_s * 1000;
    while (static_cast<int32_t>(deadline - millis()) > 0) {
        delay(1000);
        Serial.println(static_cast<int32_t>(deadline - millis()) / 1000);
        sds011.perform_work();
    }
}


/******************************* Air Quality Sensor **********************************************/


/******************************* BME680 Sensor **********************************************/

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme;

void run_bme_sensor() {

  if (! bme.performReading()) {
    Serial.println("Failed to perform reading of BME680 :(");
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

  Wire1.begin(SDA, SCL); // SDA = 21, SCL = 22


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
/************* SDS011 ************************/
    SDSserial.begin(9600, SERIAL_8N1, SDSRX, SDSTX); 
    delay(100);

    Serial.println(F("SDS011 start/stop and reporting sample"));

    start_SDS();
    Serial.print(F("SDS011 is running = "));
    Serial.println(is_SDS_running);

    String firmware_version;
    uint16_t device_id;
    if (!sds011.device_info(firmware_version, device_id)) {
        Serial.println(F("Sds011::firmware_version() failed"));
    }
    else
    {
        Serial.print(F("Sds011 firmware version: "));
        Serial.println(firmware_version);
        Serial.print(F("Sds011 device id: "));
        Serial.println(device_id, 16);
    }

    Sds011::Report_mode report_mode;
    if (!sds011.get_data_reporting_mode(report_mode)) {
        Serial.println(F("Sds011::get_data_reporting_mode() failed"));
    }
    if (Sds011::REPORT_ACTIVE != report_mode) {
        Serial.println(F("Turning on Sds011::REPORT_ACTIVE reporting mode"));
        if (!sds011.set_data_reporting_mode(Sds011::REPORT_ACTIVE)) {
            Serial.println(F("Sds011::set_data_reporting_mode(Sds011::REPORT_ACTIVE) failed"));
        }
    }
    /*************** END SDS011 **********************/
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
              //  LoRaWAN.displaySending();

                run_aq_sensor();
                run_bme_sensor();
                read_gps();
                prepareTxFrame( appPort );
                Serial.println("=== AQ ===");
                Serial.print("pm 2.5: ");
                Serial.println(ext_pm25);
                Serial.print("pm 10: ");
                Serial.println(ext_pm10);
                Serial.println("=== END AQ ===");
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
