/*******************************************************************************
 * Copyright (c) 2020 OpenWave inc, and Shoichi Owashi
 * 
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED. 
 * 
 * Required Library: 
 *    * https://github.com/adafruit/DHT-sensor-library
 *    * https://github.com/adafruit/Adafruit_Sensor
 *    * https://github.com/openwave-co-jp/arduino-lmic-master-for-lg308
 *    * https://github.com/myDevicesIoT/CayenneLPP
 *    * https://github.com/rocketscream/Low-Power
 *    
 * Require Hardware:
 *    * LoRaMini + DHT11
 *    * LG308-JP
 *    
 *    このサンプルは、DHT11の温度、湿度のデータをLoRaWANで送信します。
 *    
 *******************************************************************************/
 

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "DHT.h"
#include "CayenneLPP.h"
#include <LowPower.h>

#define SLEEP_S 100 // Sleep time. Set in units of 10 seconds.

#define dht_dpin A0 // Use A0 pin as Data pin for DHT11. 
#define DHTTYPE DHT11   // DHT 11 

// Device EUI ( Little Endian)
static const u1_t PROGMEM DEVEUI[8]={ 0x41, 0xA8, 0x7E, 0xD8, 0xDF, 0x34, 0x28, 0x00 };
// App EUI ( Little Endian)
static const u1_t PROGMEM APPEUI[8]={ 0xE2, 0xF7, 0x01, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
// App Key
static const u1_t PROGMEM APPKEY[16] ={ 0xAD, 0x8D, 0x3D, 0xA0, 0xF7, 0x71, 0x77, 0xAE, 0x4D, 0xB2, 0x56, 0x25, 0x77, 0xB6, 0x5C, 0xCF };

void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 0;

DHT dht(dht_dpin, DHTTYPE);

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

void do_send(osjob_t* j){

    pinMode(A2, OUTPUT);
    digitalWrite(A2, HIGH);

    delay(5000);

    // Cpu Voltage
    double value = cpuVcc();
    
    //Serial.println(value);

    float h = dht.readHumidity();
    float t = dht.readTemperature();

    CayenneLPP lpp(51);            // create a buffer of 51 bytes to store the payload

    lpp.reset();                   // clear the buffer
    lpp.addTemperature(1, t);      // on channel 1, add temperature, value 22.5°C
    lpp.addRelativeHumidity(2, h); // channel 2, pressure
    lpp.addAnalogInput(3, value);

    // Set data rate and transmit power (note: txpow seems to be ignored by the library)
    // LMIC_setDrTxpow(AS923_DR_SF10,13);
    
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println("OP_TXRXPEND, not sending");
    } else {
        Serial.println(F("LMIC SET DN"));
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
        Serial.println("Packet queued");
        //Serial.println(LMIC.freq);
    }
    // Next TX is scheduled after TX_COMPLETE event.

    digitalWrite(A2, LOW);
    
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            
            for (int i=0; i<SLEEP_S / 10 ; i++) {
              // Use library from https://github.com/rocketscream/Low-Power
                LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
                LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
            }
            do_send(&sendjob);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void setup() {
    
    Serial.begin(9600);
    Serial.println("Starting");

    delay(1000);

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    Serial.println(F("LMIC SET TX"));

    dht.begin();

    LMIC_setAdrMode(1);
    LMIC_setLinkCheckMode(1);
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    do_send(&sendjob);

}

void loop() {
    os_runloop_once();
}

//**** CPU TEMP & VCC ***************
//****************************
float cpuTemp(){                     // CPU温度測定関数
//****************************
  long sum=0;
  adcSetup(0xC8);                    // Vref=1.1V, input=ch8
  for(int n=0; n < 100; n++){
    sum = sum + adc();               // adcの値を読んで積分
  }
  return (sum * 1.1/102.4)- 342.5;   // 温度を計算して戻り値にする。-342.5は要調整
}
 
//****************************
float cpuVcc(){                      // 電源電圧(AVCC)測定関数
//****************************
  long sum=0;
  adcSetup(0x4E);                    // Vref=AVcc, input=internal1.1V
  for(int n=0; n < 10; n++){
    sum = sum + adc();               // adcの値を読んで積分
  }
  return (1.1 * 10240.0)/ sum;       // 電圧を計算して戻り値にする
}
 
//****************************
void adcSetup(byte data){            // ADコンバーターの設定
//****************************
  ADMUX = data;                      // ADC Multiplexer Select Reg.
  ADCSRA |= ( 1 << ADEN);            // ADC イネーブル
  ADCSRA |= 0x07;                    // AD変換クロック CK/128
  delay(10);                         // 安定するまで待つ
}
 
//****************************
unsigned int adc(){                  // ADCの値を読む
//****************************
  unsigned int dL, dH;
  ADCSRA |= ( 1 << ADSC);            // AD変換開始
  while(ADCSRA & ( 1 << ADSC) ){     // 変換完了待ち
  }
  dL = ADCL;                         // LSB側読み出し 
  dH = ADCH;                         // MSB側
  return dL | (dH << 8);             // 10ビットに合成した値を返す
}
