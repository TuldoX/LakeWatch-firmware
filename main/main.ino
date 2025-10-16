#include <DallasTemperature.h>
#include <OneWire.h>

#define tempPin 32
#define tdsPin 33
#define PH_PIN 34
#define VREF 3.3
#define ADC_RES 4095
#define NUM_SAMPLES 60
#define SAMPLE_DELAY 100
#define SCOUNT 30

const float SLOPE = -30.0;
const float OFFSET = 73.0;

int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;

class Response {
  public:
    float temperature;
    String temperatureMSG = "OK";
    int tds;
    String tdsMSG = "OK";
    float pH;
    String pHMSG = "OK";
};

int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++) bTab[i] = bArray[i];
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
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}

int measureTDS(float temperature) {
  if (temperature == -1.0) temperature = 25.0;
  int samples[SCOUNT];
  for (int i = 0; i < SCOUNT; i++) {
    samples[i] = analogRead(tdsPin);
    delay(5);
  }
  int medianADC = getMedianNum(samples, SCOUNT);
  if (medianADC <= 0 || medianADC >= 4095) return -1;
  float voltage = medianADC * (float)VREF / 4096.0;
  float compensation = voltage / (1.0 + 0.02 * (temperature - 25.0));
  float tdsValue = (133.42 * compensation * compensation * compensation
                   - 255.86 * compensation * compensation
                   + 857.39 * compensation) * 0.5;
  if (tdsValue <= 0) return -1;
  return (int)tdsValue;
}

OneWire oneWire(tempPin);
DallasTemperature sensor(&oneWire);

float measureTemperature(DallasTemperature &sensor) {
  sensor.requestTemperatures();
  if (sensor.getTempCByIndex(0) == -127.0) return -1.0;
  return sensor.getTempCByIndex(0);
}

float measurePH() {
  float totalVoltage = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    int rawValue = analogRead(PH_PIN);
    float voltage = (rawValue * VREF) / ADC_RES;
    totalVoltage += voltage;
    delay(SAMPLE_DELAY);
  }
  float avgVoltage = totalVoltage / NUM_SAMPLES;
  return (SLOPE * avgVoltage + OFFSET);
}

void setup() {
  Serial.begin(115200);
  sensor.begin();
  pinMode(tdsPin, INPUT);
  Serial.println("Initializing sensors...");
  delay(3000);
  Serial.println("Sensors stabilized.\n");
}

void loop() {
  Response response;

  float temperature = measureTemperature(sensor);
  response.temperature = temperature;
  if (temperature == -1.0) response.temperatureMSG = "Temperature sensor disconnected";

  int tds = measureTDS(temperature);
  response.tds = tds;
  if (tds == -1) response.tdsMSG = "TDS sensor disconnected";

  float pH = measurePH();
  response.pH = pH;
  if (pH < 0 || pH > 14) response.pHMSG = "pH out of range";

  Serial.print("\r");  // return to line start (overwrites)
  Serial.print("T: ");
  Serial.print(response.temperature, 2);
  Serial.print(" °C | TDS: ");
  Serial.print(response.tds);
  Serial.print(" ppm | pH: ");
  Serial.print(response.pH, 2);
  Serial.print("     "); // clears leftovers from previous line
  delay(2000);
}
