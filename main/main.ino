#include <DallasTemperature.h>
#include <OneWire.h>

#define tempPin 32 // zmeniť podľa potreby
#define tdsPin 33

#define VREF 3.3  // 3.3 - ESP32, 5 -Arduino
#define SCOUNT 30 // počet vzoriek

int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;

class Response {
  public:
    float temperature;
    String temperatureMSG = "OK";
    int tds;
    String tdsMSG = "OK";
};

void printValues(Response response){
    Serial.print(response.temperatureMSG + " " + response.temperature + "°C\n");
    Serial.print(response.tdsMSG + " " + response.tds + " ppm\n");
    Serial.print("\n");
}

int getMedianNum(int bArray[], int iFilterLen){
  int bTab[iFilterLen];
  for (byte i = 0; i<iFilterLen; i++)
  bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0){
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

int measureTDS(float temperature){
    if(temperature == -1.0) temperature = 25.0;

    int samples[SCOUNT];
    for(int i = 0; i < SCOUNT; i++){
        samples[i] = analogRead(tdsPin);
        delay(5); // small delay to stabilize ADC
    }

    int medianADC = getMedianNum(samples, SCOUNT);
    
    if(medianADC <= 0 || medianADC >= 4095) return -1; // disconnected/floating

    float voltage = medianADC * (float)VREF / 4096.0;
    float compensation = voltage / (1.0 + 0.02 * (temperature - 25.0));
    
    float tdsValue = (133.42 * compensation * compensation * compensation 
                     - 255.86 * compensation * compensation 
                     + 857.39 * compensation) * 0.5;

    if(tdsValue <= 0) return -1;

    return (int)tdsValue;
}

OneWire oneWire(tempPin);
DallasTemperature sensor(&oneWire);

float measureTemperature(DallasTemperature &sensor){
   sensor.requestTemperatures();
   if (sensor.getTempCByIndex(0) == -127.0) return -1.0;
   return sensor.getTempCByIndex(0);
}

void setup() {
  Serial.begin(115200);

  //stabilizácia senzorov
  sensor.begin();
  float temp = measureTemperature(sensor);

  pinMode(tdsPin,INPUT);
  measureTDS(temp);

  delay(2000);
}

void loop() {
  Response response;

  float temperature = measureTemperature(sensor);
  response.temperature = temperature;

  if(temperature == -1.0){
    response.temperatureMSG = "Temperature sensor disconnected";
  }

  int tds = measureTDS(temperature);
  response.tds = tds;

  if(tds == -1){
    response.tdsMSG = "TDS sensor disconnected";
  }

  printValues(response);
  delay(2000);
}
