#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>

#define TEMP 32
#define TDS 34
#define PH 35
#define OXYGEN 25
#define uS_TO_S_FACTOR 1000000ULL
#define TIME_TO_SLEEP  10
#define VREF 3300
#define ADC_RES 4096
#define CAL1_V (800)
#define CAL1_T (22)
#define V_REF 3.3
#define SCOUNT  30

OneWire oneWire(TEMP);
DallasTemperature sensors(&oneWire);

int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;
float averageVoltage = 0;
float tdsValue = 0;

const float v_pH7 = 2.8;
const float pH7 = 7.0;
const float pH_step = -0.67;

const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410
};

int average(const int arr[], int size) {
  long sum = 0;
  for (int i = 0; i < size; i++) sum += arr[i];
  return (int)((float)sum / size + 0.5);
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

float measureTDS(float temperature = 21.0){
  for(int i = 0; i < SCOUNT; i++){
    analogBuffer[i] = analogRead(TDS);
    delay(40);
  }
  
  for(copyIndex = 0; copyIndex < SCOUNT; copyIndex++){
    analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
  }
  
  averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)V_REF / 4096.0;
  
  float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
  float compensationVoltage = averageVoltage / compensationCoefficient;
  
  tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage 
              - 255.86 * compensationVoltage * compensationVoltage 
              + 857.39 * compensationVoltage) * 0.5;
  
  return tdsValue;
}

float measurePH() {
  int measure[30];

  for (int i = 0; i < 30; i++) {
    measure[i] = analogRead(PH);
    delay(10);
  }

  int measureAverage = average(measure, 30);
  float voltage = measureAverage * 3.3 / 4096.0;

  // Correct formula
  float pH = pH7 + (voltage - v_pH7) / pH_step;

  return pH;
}

int measureDO(float temperature = 21.0) {
  uint8_t temp_c = (uint8_t)temperature;
  uint16_t adc_raw = analogRead(OXYGEN);
  uint16_t adc_voltage = (uint32_t)VREF * adc_raw / ADC_RES;
  uint16_t v_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temp_c - (uint32_t)CAL1_T * 35;
  int do_value = (adc_voltage * DO_Table[temp_c]) / v_saturation;
  return do_value;
}

void setup(){
  Serial.begin(115200);
  delay(2000);
  sensors.begin();

  analogSetPinAttenuation(PH, ADC_11db);
  analogSetPinAttenuation(TDS, ADC_11db);
  analogSetPinAttenuation(OXYGEN, ADC_11db);
  analogReadResolution(12);

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  sensors.requestTemperatures(); 
  float temperature = sensors.getTempCByIndex(0);
  delay(1000);

  pinMode(TDS, INPUT);
  float tds = measureTDS(temperature);
  delay(1000);

  float ph = measurePH();
  delay(1000);

  int do_value_ugL = measureDO(temperature);
  float oxygen = do_value_ugL / 1000.0; //mg/L
  delay(1000);

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println("°C");

  Serial.print("TDS: ");
  Serial.print(tds);
  Serial.println(" ppm");

  Serial.print("pH: ");
  Serial.println(ph);

  Serial.print("Oxygen: ");
  Serial.print(oxygen);
  Serial.println(" mg/L");

  delay(5000);
  esp_deep_sleep_start();
}

void loop(){}