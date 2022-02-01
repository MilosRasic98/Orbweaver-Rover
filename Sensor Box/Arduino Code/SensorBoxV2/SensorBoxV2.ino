// Code for the Sensor Box

// Libraries
#include "ArduinoLowPower.h"
#include "DHT.h"
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <ds3231.h>
#include <SPI.h>
#include <LoRa.h>

// ID of this sensor box - unique for each box
#define BOX_ID 1

// MOSFET Pin
#define MOSFET 3

// DHT22
#define DHTPIN 2
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
float temperature;
float humidity;
float heatIndex;

// Pressure sensor
Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
double pressure = 0;

// RTC module
struct ts t;
String current_time;
String current_date;

// Rain sensor
int rain_sensor = 0;

// LoRa counter
int msg_counter = 0;

// Battery voltage
float battery_voltage = 3.0;
float voltage;

void setup() {

  // Starting serial
  Serial.begin(9600);

  // Setting up pins
  pinMode(MOSFET, OUTPUT);

  // Setting up the ADC
  analogReadResolution(10);

  // DHT22
  dht.begin();

  // Pressure Sensor
  unsigned status;
  status = bmp.begin(0x76, 0x58);

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  bmp_temp->printSensorDetails();

  // RTC
  Wire.begin();
  DS3231_init(DS3231_CONTROL_INTCN);

  // LoRa
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  
}

void loop() {
  // First we need to get all of the data from the sensors
  // List of things we need to read:
  // 1 - DHT22    - temperature, humidity, heat_index
  // 2 - BMP280   - pressure
  // 3 - RTC      - time and date
  // 4 - Battery  - battery voltage
  // 5 - Rain     - rain sensor reading

  // DHT22
  temperature = dht.readTemperature();
  humidity    = dht.readHumidity();
  heatIndex   = dht.computeHeatIndex(temperature, humidity, false);

  // BMP280
  sensors_event_t pressure_event;
  bmp_pressure -> getEvent(&pressure_event);
  pressure = pressure_event.pressure;

  // RTC
  DS3231_get(&t);
  current_date = String(t.mday) + "/" + String(t.mon) + "/" + String(t.year);
  current_time = String(t.hour) + ":" + String(t.min) + "/" + String(t.sec);

  // Battery voltage
  analogReference(AR_INTERNAL1V0);
  voltage = analogRead(ADC_BATTERY) * (battery_voltage / 1023.0);
  analogReference(AR_DEFAULT);

  // Rain sensor
  digitalWrite(MOSFET, HIGH);
  delay(100);
  rain_sensor = analogRead(A1);
  digitalWrite(MOSFET, LOW);

  /* After gathering all of the necessary data, we need to format it into our message
  // Each field is enclosed in a start flag (&S...) and end flag (&E...)
  // List of all flags:
  # Data Contained in the Sensor Message forwarded by the GCS
  # Numb  - Name          - Start         - Stop          - Description
  # 1     - Message ID    - &SID          - &EID          - ID of the message packet
  # 2     - Box ID        - &SBOXID       - &SBOXID       - ID of the sensor box - Arduino MKR WAN 1300
  # 3     - Date          - &SDATE        - &EDATE        - Date when the message was sent
  # 4     - Time          - &STIME        - &ETIME        - Timestamp when the message was sent
  # 5     - Temperature   - &STEMP        - &ETEMP        - Temperature measured by the DHT22
  # 6     - Heat Index    - &SHEATINDEX   - &EHEATINDEX   - Heat index calculated using the measurements made by the DHT22
  # 7     - Humidity      - &SHUMIDITY    - &EHUMIDITY    - Humidity measured by the DHT22
  # 8     - Pressure      - &SPRESSURE    - &EPRESSURE    - Pressure measured by the BMP280
  # 9     - Battery       - &SBAT         - &EBAT         - Voltage of the batteries powering the sensor box
  # 10    - Rain sensor   - &SRAIN        - &ERAIN        - Measurement of the Arduino Rain Sensor
  */

  // 1  - Message ID
  String msg1 = "&SID" + String(msg_counter) + "&EID";

  // 2  - Box ID
  String msg2 = "&SBOXID" + String(BOX_ID) + "&EBOXID";

  // 3  - Date
  String msg3 = "&SDATE" + current_date + "&EDATE";

  // 4  - Time
  String msg4 = "&STIME" + current_time + "&ETIME";

  // 5  - Temperature
  String msg5 = "&STEMP" + String(temperature) + "&ETEMP";

  // 6  - Heat Index
  String msg6 = "&SHEATINDEX" + String(heatIndex) + "&EHEATINDEX";

  // 7  - Humidity
  String msg7 = "&SHUMIDITY" + String(humidity) + "&EHUMIDITY";

  // 8  - Pressure
  String msg8 = "&SPRESSURE" + String(pressure) + "&EPRESSURE";

  // 9  - Battery
  String msg9 = "&SBAT" + String(voltage) + "&EBAT";

  // 10 - Rain Sensor
  String msg10 = "&SRAIN" + String(rain_sensor) + "&ERAIN";

  // To form the final message, we need to combine all 10 messages into one single message
  String lora_message = msg1 + msg2 + msg3 + msg4 + msg5 + msg6 + msg7 + msg8 + msg9 + msg10;

  // Incrementing the msg_counter
  msg_counter++;

  // Sending the LoRa message
  LoRa.beginPacket();
  LoRa.print(lora_message);
  LoRa.endPacket();

  LowPower.sleep(5000);
  
}
