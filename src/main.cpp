#include <ESP32_LoRaWAN.h>
#include "Arduino.h"
#include<CayenneLPP.h>


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
                                                                     //#if 0
                                                                     //static uint32_t license [4] = { 0x9c5aa386, 0x967d0549, 0x333f6d6f, 0x807db52f }; // Matthew
                                                                     //#else
                                                                     //static uint32_t license [4] = { 0x6a3f7214, 0x0bcb2c9b, 0xfe0a5c7d, 0x3f7de4d3 }; // JCW
                                                                     //#endif
static uint8_t debugLevel = 0;
static uint32_t appTxDutyCycle = 60000;


// Sensor values

int pm25 = 0;
int pm10 = 0;
int temperature = 0;
int humidity = 0;
int gas = 0;


//
//  Set up the data to be sent to 
//  TTN
//

CayenneLPP lpp(128);


//static void prepareTxFrame (uint8_t port)
//{
//    appDataSize = 5;
//    appData[0] = pm25;
//    appData[1] = pm10;
//    appData[2] = temperature;
//    appData[3] = humidity;
//    appData[4] = gas;
//
//     appDataSize = lpp.getSize();
//      appData = lpp.getBuffer();
//}
static void prepareTxFrame( uint8_t port )
{
  CayenneLPP *lpp = new CayenneLPP( LORAWAN_APP_DATA_MAX_SIZE );

  lpp->reset();
  lpp->addTemperature(0, temperature);
  lpp->addRelativeHumidity(1, humidity);
  lpp->addAnalogInput( 2, pm25 );
  lpp->addAnalogInput( 3, pm10);
  lpp->addAnalogInput( 4, gas);

  appDataSize = lpp->copy( appData );

  delete lpp;
}

void setup ()
{
  if(mcuStarted==0)
  {
    LoRaWAN.displayMcuInit();
  }
  Serial.begin(115200);
  while (!Serial);
  SPI.begin(SCK,MISO,MOSI,SS);
  Mcu.init(SS,RST_LoRa,DIO0,DIO1,license);
  deviceState = DEVICE_STATE_INIT;
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
                LoRaWAN.displaySending();
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
