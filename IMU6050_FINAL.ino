#include <HardwareSerial.h>
HardwareSerial serial(2);//definimos un serial para UART2 
const uint8_t r2Pin = 16;
const uint8_t t2Pin = 17;

int rollx_low = 0, rollx_high = 0;
int pitchx_high = 0, pitchx_low = 0;
int yawx_high = 0, yawx_low = 0;
int Wx_low = 0, Wx_high = 0;
int Wz_low = 0, Wz_high = 0;
int Wy_low = 0, Wy_high = 0;
int Ax_low = 0, Ax_high = 0;
int Ay_low = 0, Ay_high = 0;
int Az_low = 0, Az_high = 0;


void setup() {
  // put your setup code here, to run once:
  serial.begin(115200);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  //IMU6050
  //Ángulo de salida
  if (serial.available())
  {
    if (serial.read() == 'U') // Angle Output
    {    
      while (!serial.available());    
      {
        char dato = serial.read();
        if (dato == 'S')
        {
          while (!serial.available());
          //roll x
          rollx_low = int(serial.read());
          while (!serial.available());
          rollx_high = int(serial.read());
//          Serial.println((rollx_high << 8) | rollx_low);
          Serial.print("roll = ");
          Serial.println(((rollx_high << 8 ) | rollx_low) / 32768.0 * 180.0);
          delay(100);
          while (!serial.available());
          //pitch x
          pitchx_low = int(serial.read());
          while (!serial.available());
          pitchx_high = int(serial.read());
          Serial.print("pitch = ");
          Serial.println(((pitchx_high << 8 ) | pitchx_low) / 32768.0 * 180.0);
          delay(100);
           while (!serial.available());
          //yaw x
          yawx_low = int(serial.read());
          while (!serial.available());
          yawx_high = int(serial.read());
          Serial.print("yaw = ");
          Serial.println(((yawx_high << 8 ) | yawx_low) / 32768.0 * 180.0);
          delay(100);
        }
      }
    }
  }
  //Velocidad angular
  if (serial.available())
  {
    if (serial.read() == 'U') // Angle Output
    {    
      while (!serial.available());    
      {
        char dato = serial.read();
        if (dato == 'R')
        {
          while (!serial.available());
          // X-axis
          Wx_low = int(serial.read());
          while (!serial.available());
          Wx_high = int(serial.read());

          Serial.print(" V_Eje X = ");
          Serial.println(((Wx_high << 8 ) | Wx_low) / 32768.0 * 2000.0);
          delay(100);
          while (!serial.available());
          //Y-axis
          Wy_low = int(serial.read());
          while (!serial.available());
          Wy_high = int(serial.read());
          Serial.print("V_Eje Y = ");
          Serial.println(((Wy_high << 8 ) | Wy_low) / 32768.0 * 2000.0);
          delay(100);
           while (!serial.available());
          //Z-axis
          Wz_low = int(serial.read());
          while (!serial.available());
          Wz_high = int(serial.read());
          Serial.print("V_Eje Z = ");
          Serial.println(((Wz_high << 8 ) | Wz_low) / 32768.0 * 2000.0);
          delay(100);
        }
      }
    }
  }
  // Aceleración
  if (serial.available())
  {
    if (serial.read() == 'U') // Angle Output
    {    
      while (!serial.available());    
      {
        char dato = serial.read();
        if (dato == 'Q')
        {
          while (!serial.available());
          // X-axis
          Ax_low = int(serial.read());
          while (!serial.available());
          Ax_high = int(serial.read());

          Serial.print(" A_Eje X = ");
          Serial.println(((Ax_high << 8 ) | Ax_low) / 32768.0 * (16.0*9.8));
          delay(100);
          while (!serial.available());
          //Y-axis
          Ay_low = int(serial.read());
          while (!serial.available());
          Ay_high = int(serial.read());
          Serial.print("A_Eje Y = ");
          Serial.println(((Ay_high << 8 ) | Ay_low) / 32768.0 * (16.0*9.8));
          delay(100);
           while (!serial.available());
          //Z-axis
          Az_low = int(serial.read());
          while (!serial.available());
          Az_high = int(serial.read());
          Serial.print("V_Eje Z = ");
          Serial.println(((Az_high << 8 ) | Az_low) / 32768.0 * (16.0*9.8));
          delay(100);
        }
      }
    }
  }
}
