#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define DHTPIN 2 // Pin D4
#define DHTTYPE DHT22

#define ONE_WIRE_BUS 4 // Pin D2

#define TRIG 5  // Pin D1
#define ECHO 16 // Pin D0

#define VREF 5000    // VREF (mv)
#define ADC_RES 1024 // ADC Resolution
#define TWO_POINT_CALIBRATION 0
#define READ_TEMP (25)
#define CAL1_V (131)  // mv
#define CAL1_T (25)   //℃
#define CAL2_V (1300) // mv
#define CAL2_T (15)   //℃
const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};

uint8_t Temperaturet;
uint16_t ADC_Raw;
uint16_t ADC_Voltage;
uint16_t DO;

int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c)
{
#if TWO_POINT_CALIBRATION == 00
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#else
  uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#endif
}
// Setup Mqtt server (Localhost)
#define mqtt_Server "192.168.1.50"
// Setup Mqtt port (Default port 1883)
#define mqtt_Port 1883
#define MQTT_PUB_TEMP "sensor/dht/temp"
#define MQTT_PUB_HUM "sensor/dht/hum"
#define MQTT_PUB_DS "sensor/ds/temp"
#define MQTT_PUB_HCSR04 "sensor/HCSR04/cm"
#define MQTT_PUB_PH "sensor/PH/ph"
#define MQTT_PUB_DO "sensor/DO/do"

int DO_PIN = A0; // DO sensor

int pHSense = 2; // PH sensor
float calibration_value = 21.34 - 0.7;
int buffer_arr[10], tempPH;
float ph_act;
int phval = 0;
unsigned long int avgval;
int samples = 10;
float adc_resolution = 1024.0;
// Setup Delay Time
int Wifi_period = 500;  // 500 ms
int Mqtt_period = 3000; // 3 Sec
int DHT_period = 3000;
int DS_period = 2000;
int HCSR04_period = 3000;
int PH_period = 3000;
int DO_period = 2000;
int random_period = 1000;
unsigned long time_now = 0;

DHT dht(DHTPIN, DHTTYPE);
float temp, humi;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float temp_DS;

long duration, cm;

float random_Num; // Random sensor val
WiFiClient Wifi_Client;
PubSubClient mqtt_client(Wifi_Client);
void setup()
{
  // Setup Serial baudrate
  Serial.begin(115200);
  dht.begin();
  sensors.begin();
  // Connect to Wifi
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    if (millis() >= time_now + Wifi_period)
    {
      time_now += Wifi_period;
      Serial.print(".");
    }
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Setup mqtt_Server and Port
  mqtt_client.setServer(mqtt_Server, mqtt_Port);
}

void reconnect()
{
  // Reconnect to Mqtt server
  while (!mqtt_client.connected())
  {
    Serial.print("MQTT connecting...");
    // Attempt to connect
    if (mqtt_client.connect("Client"))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed, reconnect=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 3 seconds");
      if (millis() >= time_now + Mqtt_period)
      {
        time_now += Mqtt_period;
      }
    }
  }
}

void dht_read()
{
  if (millis() >= time_now + DHT_period)
  {
    time_now += DHT_period;
    delay(100);
    humi = dht.readHumidity();
    temp = dht.readTemperature();
    if (isnan(humi) || isnan(temp))
    {
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }
    mqtt_client.publish(MQTT_PUB_TEMP, String(temp).c_str());
    mqtt_client.publish(MQTT_PUB_HUM, String(humi).c_str());
    Serial.printf("Temp: %.2f \n", temp);
    Serial.printf("Humi: %.2f \n", humi);
  }
}

void DS18B20()
{
  if (millis() >= time_now + DS_period)
  {
    sensors.requestTemperatures(); //อ่านข้อมูลจาก library
    mqtt_client.publish(MQTT_PUB_DS, String(sensors.getTempCByIndex(0)).c_str());
    Serial.printf("Temp_DS: %.2f \n", sensors.getTempCByIndex(0));
    // temp_DS = sensors.getTempCByIndex(0);
  }
}
long microsecondsToCentimeters(long microseconds)
{
  // ความเร็วเสียงในอากาศประมาณ 340 เมตร/วินาที หรือ 29 ไมโครวินาที/เซนติเมตร
  // ระยะทางที่ส่งเสียงออกไปจนเสียงสะท้อนกลับมาสามารถใช้หาระยะทางของวัตถุได้
  // เวลาที่ใช้คือ ระยะทางไปกลับ ดังนั้นระยะทางคือ ครึ่งหนึ่งของที่วัดได้
  return microseconds / 29 / 2;
}
void HCSR04()
{
  if (millis() >= time_now + HCSR04_period)
  {
    pinMode(TRIG, OUTPUT);
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(5);
    digitalWrite(TRIG, LOW);
    pinMode(ECHO, INPUT);
    duration = pulseIn(ECHO, HIGH);
    cm = microsecondsToCentimeters(duration);
    mqtt_client.publish(MQTT_PUB_HCSR04, String(cm).c_str());
    Serial.print(cm);
    Serial.print("cm");
    Serial.println();
  }
}

float ph(float voltage)
{
  return 7 + ((2.5 - voltage) / 0.18);
}

void ph_read()
{
  for (int i = 0; i < 10; i++)
  {
    buffer_arr[i] = analogRead(A0);
    delay(30);
  }
  for (int i = 0; i < 9; i++)
  {
    for (int j = i + 1; j < 10; j++)
    {
      if (buffer_arr[i] > buffer_arr[j])
      {
        tempPH = buffer_arr[i];
        buffer_arr[i] = buffer_arr[j];
        buffer_arr[j] = tempPH;
      }
    }
  }
  avgval = 0;
  for (int i = 2; i < 8; i++)
    avgval += buffer_arr[i];
  float volt = (float)avgval * 5.0 / 1024 / 6;
  ph_act = -5.70 * volt + calibration_value;
  delay(1000);
  if (millis() >= time_now + PH_period)
  {
    mqtt_client.publish(MQTT_PUB_PH, String(ph(ph_act)).c_str());
    Serial.print("pH= ");
    Serial.println(ph_act);
  }
}

void read_DO()
{
  if (millis() >= time_now + HCSR04_period)
  {
    Temperaturet = (uint8_t)READ_TEMP;
    ADC_Raw = analogRead(DO_PIN);
    ADC_Voltage = uint32_t(VREF) * ADC_Raw / ADC_RES;

    Serial.print("Temperaturet:\t" + String(Temperaturet) + "\t");
    Serial.print("ADC RAW:\t" + String(ADC_Raw) + "\t");
    Serial.print("ADC Voltage:\t" + String(ADC_Voltage) + "\t");
    mqtt_client.publish(MQTT_PUB_DO, String(readDO(ADC_Voltage, Temperaturet)).c_str());
    Serial.println("DO:\t" + String(readDO(ADC_Voltage, Temperaturet)) + "\t");
  }
}

void loop()
{
  // Check mqtt connection
  if (!mqtt_client.connected())
  {
    reconnect();
  }
  // Make Mqtt Keep Alive
  dht_read(); //
  DS18B20();  //
  HCSR04();   //
  ph_read();
  read_DO();
}