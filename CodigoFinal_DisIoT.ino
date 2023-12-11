#include <WiFi.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
 
Adafruit_INA219 ina219;
float busvoltage = 0;
float current_mA = 0;
float potencia = 0;

//habilitamos el puerto serial 2 de la ESP32
HardwareSerial serial(2);
const uint8_t r2Pin = 16;
const uint8_t t2Pin = 17;
int rollx_low = 0, rollx_high = 0;
int pitchx_high = 0, pitchx_low = 0;
int yawx_high = 0, yawx_low = 0;
float pitch, yaw, roll;
//declaramos el nombre de la red wifi donde nos conectaremos y su contraseña
const char* ssid = "COSMO";
const char* password = "3158746409";
const char* mqtt_server = "broker.emqx.io";


WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;


void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.publish("outTopic", "T1");
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  uint32_t currentFrequency;
  // Iniciar el INA219
  ina219.begin();  //por defecto, inicia a 32V y 2A
  //cambiamos la sensibilidad del sensor
  ina219.setCalibration_32V_1A();

  serial.begin(115200); //se inicia el puerto serial 2
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  delay(1000);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();

  if (now - lastMsg > 300) {
    lastMsg = now;
  //Obtenemos valores del acelerometro
  if (serial.available()) 
  {
    if (serial.read() == 'U') 
    {
      while (!serial.available());
      {
        char dato = serial.read();
        if (dato == 'S') 
        {
          while (!serial.available());
          //roll
          rollx_low = int(serial.read());
          while (!serial.available());
          rollx_high = int(serial.read());
          roll = ((rollx_high << 8 ) | rollx_low) / 32768.0 * 180.0;
          Serial.print("roll = ");
          Serial.println(roll);
          //delay(100);
          while (!serial.available());
          //pitch
          pitchx_low = int(serial.read());
          while (!serial.available());
          pitchx_high = int(serial.read());
          pitch = ((pitchx_high << 8 ) | pitchx_low) / 32768.0 * 180.0;
          Serial.print("pitch = ");
          Serial.println(pitch);
          //delay(100);
          while (!serial.available());
          //yaw
          yawx_low = int(serial.read());
          while (!serial.available());
          yawx_high = int(serial.read());
          yaw = ((yawx_high << 8 ) | yawx_low) / 32768.0 * 180.0;
          Serial.print("yaw = ");
          Serial.println(yaw);
          //delay(100);
        }
      }
    }
  }
  //obtenemos valores del ina219
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    potencia = busvoltage / (current_mA/-1);
    delay(300);
   

    char pitchStr[10];
    char yawStr[10];
    char rollStr[10];

    char busvoltageStr[10];
    char current_mAStr[10];
    char potenciaStr[10];

    
    dtostrf(pitch, 5, 2, pitchStr);
    dtostrf(yaw, 5, 2, yawStr);
    dtostrf(roll, 5, 2, rollStr);

    dtostrf(busvoltage, 5, 2, busvoltageStr);
    dtostrf(current_mA, 5, 2, current_mAStr);
    dtostrf(potencia, 5, 2, potenciaStr);

    //publicamos variables del  acelerometro
    client.publish("pitch_channel", pitchStr);
    Serial.print("Pitch: ");
    Serial.println(pitch);
    client.publish("yaw_channel", yawStr);
    Serial.print("Yaw: ");
    Serial.println(yaw);
    client.publish("roll_channel", rollStr);
    Serial.print("Roll: ");
    Serial.println(roll);
    // publicamos variables del ina219
    client.publish("voltage_channel", busvoltageStr);
    Serial.print("Voltage: ");
    Serial.println(busvoltage);
    client.publish("current_channel", current_mAStr);
    Serial.print("Corriente: ");
    Serial.println(current_mA);
    client.publish("power_channel", potenciaStr);
    Serial.print("Potencia: ");
    Serial.println(potencia);

  }
}

