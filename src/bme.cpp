#include <ESP32_LoRaWAN.h>
#include <CayenneLPP.h>
#include <SoftwareSerial.h>
#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"



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

  appDataSize = lpp->copy( appData );

  delete lpp;
}
/******************************* LORAWAN **********************************************/


/******************************* Air Quality Sensor **********************************************/

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
  Serial.print(bme.gas_resistance / 1000.0);
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
