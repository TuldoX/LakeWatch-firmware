#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>


#define TEMP 32
#define TDS 34
#define PH 36
#define OXYGEN 39
#define BATTERY 35

#define uS_TO_S_FACTOR 1000000ULL
#define TIME_TO_SLEEP  3600

#define VREF 3300
#define ADC_RES 4096
#define CAL1_V (800)
#define CAL1_T (22)

#define V_REF 3.3
#define SCOUNT  30

#define SIM_PIN ""
#define NETWORK_APN "internet"

#define LILYGO_T_A7670
#define TINY_GSM_MODEM_A7670

#include <TinyGsmClient.h>

#if defined(LILYGO_T_A7670)
  #define MODEM_BAUDRATE        115200
  #define MODEM_DTR_PIN         25
  #define MODEM_TX_PIN          26
  #define MODEM_RX_PIN          27
  #define BOARD_PWRKEY_PIN      4
  #define BOARD_POWERON_PIN     12
  #define MODEM_RING_PIN        33
  #define MODEM_RESET_PIN       5
  #define MODEM_RESET_LEVEL     HIGH
  #define SerialAT              Serial1
#endif

TinyGsm modem(SerialAT);
Preferences prefs;

const char *API_URL = "https://httpbin.org/post";
String JWT_TOKEN;

OneWire oneWire(TEMP);
DallasTemperature sensors(&oneWire);

int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
float averageVoltage = 0;
int tdsValue = 0;

const float v_pH7 = 2.8;
const float pH7 = 7.0;
const float pH_step = -0.67;

const uint16_t DO_Table[41] = {
  14460,14220,13820,13440,13090,12740,12420,12110,11810,11530,
  11260,11010,10770,10530,10300,10080,9860,9660,9460,9270,
  9080,8900,8730,8570,8410,8250,8110,7960,7820,7690,
  7560,7430,7300,7180,7070,6950,6840,6730,6630,6530,6410
};

int average(const int arr[], int size) {
  long sum = 0;
  for (int i = 0; i < size; i++) sum += arr[i];
  return sum / size;
}

int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  memcpy(bTab, bArray, iFilterLen * sizeof(int));
  for (int j = 0; j < iFilterLen - 1; j++) {
    for (int i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        int t = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = t;
      }
    }
  }
  return (iFilterLen & 1) ? bTab[iFilterLen / 2]
                          : (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
}

int measureTDS(float temperature) {
  for (int i = 0; i < SCOUNT; i++) {
    analogBuffer[i] = analogRead(TDS);
    delay(40);
  }
  memcpy(analogBufferTemp, analogBuffer, sizeof(analogBuffer));
  averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * V_REF / ADC_RES;
  float compensation = 1.0 + 0.02 * (temperature - 25.0);
  float voltage = averageVoltage / compensation;
  float result = (133.42 * voltage * voltage * voltage - 255.86 * voltage * voltage + 857.39 * voltage) * 0.5;
  return (int)round(result);
}

float measurePH() {
  int buf[30];
  for (int i = 0; i < 30; i++) {
    buf[i] = analogRead(PH);
    delay(10);
  }
  float voltage = average(buf, 30) * 3.3 / ADC_RES;
  return pH7 + (voltage - v_pH7) / pH_step;
}

float measureDO(float temperature) {
  uint8_t t = (uint8_t)temperature;
  uint16_t adc = analogRead(OXYGEN);
  uint16_t mv = VREF * adc / ADC_RES;
  uint16_t vsat = CAL1_V + 35 * t - CAL1_T * 35;
  return (mv * DO_Table[t]) / vsat / 1000.0;
}

String constructJSON(float t, int tds, float ph, float o2,int percentage) {
  return "{"
    "\"id\":" + String(1) + ","
    "\"batteryLife\":" + String(percentage) + ","
    "\"temperature\":" + String(t, 1) + ","
    "\"tds\":" + String(tds)+ ","
    "\"ph\":" + String(ph, 1) + ","
    "\"oxygen\":" + String(o2, 1) +
  "}";
}

void initModem() {
  pinMode(BOARD_POWERON_PIN, OUTPUT);
  digitalWrite(BOARD_POWERON_PIN, HIGH);

  pinMode(MODEM_RESET_PIN, OUTPUT);
  digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
  delay(100);
  digitalWrite(MODEM_RESET_PIN, MODEM_RESET_LEVEL);
  delay(2600);

  pinMode(BOARD_PWRKEY_PIN, OUTPUT);
  digitalWrite(BOARD_PWRKEY_PIN, HIGH);
  delay(500);

  SerialAT.begin(MODEM_BAUDRATE, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

  while (!modem.testAT()) delay(500);
}

bool connectNetwork() {
  while (modem.getSimStatus() != SIM_READY) delay(1000);

  modem.sendAT("+CGDCONT=1,\"IP\",\"", NETWORK_APN, "\"");
  modem.waitResponse();

  while (!modem.isNetworkConnected()) delay(1000);
  modem.setNetworkActive();

  modem.https_begin();
  return true;
}

bool sendDataToAPI(float t, int tds, float ph, float o2, String token,int percent) {
  String payload = constructJSON(t, tds, ph, o2,percent);
  Serial.println(payload);

  if (!modem.https_set_url(API_URL)) return false;

  modem.sendAT("+HTTPPARA=\"CONTENT\",\"application/json\"");
  modem.waitResponse();

  modem.sendAT("+HTTPPARA=\"USERDATA\",\"Authorization: Bearer ", token, "\"");
  modem.waitResponse();

  int code = modem.https_post(payload);
  Serial.println(code);
  Serial.println(modem.https_body());

  return (code == 200 || code == 201);
}

float readBatteryVoltage() {
  long sum = 0;
  for (int i = 0; i < 15; i++) {
    sum += analogRead(BATTERY);
    delay(3);
  }

  float raw = sum / 15.0;
  float adcV = (raw / 4095.0) * 3.3;
  return adcV * 2.0;   // voltage divider on LILYGO
}

int batteryPercent(float v) {
  if (v >= 4.20) return 100;
  if (v >= 4.10) return 90;
  if (v >= 4.00) return 80;
  if (v >= 3.90) return 70;
  if (v >= 3.80) return 60;
  if (v >= 3.70) return 50;
  if (v >= 3.60) return 35;
  if (v >= 3.50) return 20;
  if (v >= 3.40) return 10;
  return 0;
}

void setup() {
  Serial.begin(115200);
  sensors.begin();

  prefs.begin("app", false);
  String token = prefs.getString("token", "DEFAULT");

  analogSetPinAttenuation(PH, ADC_11db);
  analogSetPinAttenuation(TDS, ADC_11db);
  analogSetPinAttenuation(OXYGEN, ADC_11db);
  analogSetPinAttenuation(BATTERY, ADC_11db);
  analogReadResolution(12);

  sensors.requestTemperatures();
  float temp = sensors.getTempCByIndex(0);
  int tds = measureTDS(temp);
  float ph = measurePH();
  float o2 = measureDO(temp);

  float voltage = readBatteryVoltage();
  int percent = batteryPercent(voltage);

  initModem();
  connectNetwork();
  sendDataToAPI(temp, tds, ph, o2,token,percent);

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

void loop() {}