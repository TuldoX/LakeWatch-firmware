#include <DallasTemperature.h>
#include <OneWire.h>

#define tempPin 32 // zmeniť podľa potreby

OneWire oneWire(tempPin);
DallasTemperature sensor(&oneWire);

float measureTemperature(DallasTemperature sensor){
   sensor.requestTemperatures();
   return sensor.getTempCByIndex(0);
}

void setup() {
  Serial.begin(115200);
  sensor.begin();

  //stabilizácia TEMP
  sensor.requestTemperatures();
  sensor.getTempCByIndex(0);

  delay(1000);
}

void loop() {
  Serial.print(measureTemperature(sensor));
  Serial.print("\n");
  delay(3000);
}
