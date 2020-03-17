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
 * Require Hardware:
 *    * Arduino + DHT11
 *    
 *    このサンプルは、DHT11の温度、湿度のデータを出力します。
 *    
 *******************************************************************************/
 
#include "DHT.h"

#define dht_dpin A0 // Use A0 pin as Data pin for DHT11. 
#define DHTTYPE DHT11   // DHT 11
DHT dht(dht_dpin, DHTTYPE);

void setup() {
  dht.begin();
}

void loop() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  Serial.print("Tempeperature:");
  Serial.println(t);
  Serial.print("Humidity:");
  Serial.println(h);
  delay(3000);
}
