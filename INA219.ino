#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;
float potencia;

void setup(void) 
{
  Serial.begin(115200);
  uint32_t currentFrequency;
  
  // Iniciar el INA219
  ina219.begin();  //por defecto, inicia a 32V y 2A

  // Opcionalmente, cambiar la sensibilidad del sensor
  ina219.setCalibration_32V_1A();
  //ina219.setCalibration_16V_400mA();

  Serial.println("INA219 iniciado...");
}

void loop(void) 
{
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  // Obtener mediciones
  //shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  potencia = busvoltage / (current_mA/-1);
  //power_mW = ina219.getPower_mW();
  //loadvoltage = busvoltage + (shuntvoltage / 1000);
  
  // Mostrar mediciones
  Serial.print("Bus Voltaje:   "); Serial.print(busvoltage); Serial.println(" V");
  //Serial.print("Shunt Voltaje: "); Serial.print(shuntvoltage); Serial.println(" mV");
  //Serial.print("Load Voltaje:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Corriente:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Potencia:         "); Serial.print(potencia); Serial.println(" W");
  Serial.println("");

  delay(2000);
}