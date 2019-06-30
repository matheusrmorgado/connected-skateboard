//dmp com wifi

//GND - GND
//VCC - VCC
//SDA - Pin D2
//SCL - Pin D1
//INT - Pin D7

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

WiFiUDP Udp;

const char* ssid = "WIFI-SSID";
const char* password = "WIFI-PASSWORD";
int contconexion = 0;

//Direccion I2C de la IMU
uint8_t MPU = 0x68;

size_t SIZE = 6;


#define INTERRUPT_PIN D7

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]
VectorInt16 aa;         // [x, y, z]
VectorInt16 aaReal;     // [x, y, z]
VectorInt16 aaWorld;    // [x, y, z]
VectorFloat gravity;    // [x, y, z]
float ypr[3];           // [yaw, pitch, roll]

String valores;

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

    //Iniciar WiFi
  WiFi.mode(WIFI_STA); //para que no inicie el SoftAP en el modo normal
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED and contconexion <50) { //Cuenta hasta 50 si no se puede conectar lo cancela
      ++contconexion;
      delay(500);
      Serial.print(".");
    }
    if (contconexion <50) {
        //para usar con ip fija
        IPAddress Ip(XXX,XXX,XX,XXX);  //esp8266 IP
        IPAddress Gateway(XXX,XXX,XX,X);  //ipconfig gateway padrão
        IPAddress Subnet(XXX,XXX,XXX,X); //ipconfig sub-rede
        WiFi.config(Ip, Gateway, Subnet); 
      
        Serial.println("");
        Serial.println("WiFi conectado");
        Serial.println(WiFi.localIP());
    }
    else { 
        Serial.println("");
        Serial.println("Error de conexion");
    }

    // Iniciar MPU6050
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // Comprobar  conexion
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // Iniciar DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // Valores de calibracion
    mpu.setXGyroOffset(3);
    mpu.setYGyroOffset(-7);
    mpu.setZGyroOffset(27);
    mpu.setZAccelOffset(1125);


    // Activar DMP
    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // Activar interrupcion
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void loop() {
    // Si fallo al iniciar, parar programa
    if (!dmpReady) return;

    // Ejecutar mientras no hay interrupcion
    while (!mpuInterrupt && fifoCount < packetSize) {
        // AQUI EL RESTO DEL CODIGO DE TU PROGRRAMA

    //Mostrar los valores por consola
      valores = String(ypr[2] * 180/M_PI) + "," + String(ypr[1] * 180/M_PI) + "," + String(ypr[0] * 180/M_PI);

    //Armar los paquetes UDP  
      Udp.beginPacket("XXX.XXX.XX.XX", 245);  //ipconfig endereço ipv4
      for(int i=0; i<valores.length();i++){
        Udp.write(byte(valores[i]));  
      }
      Udp.endPacket();
      Serial.println(valores);

    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // Obtener datos del FIFO
    fifoCount = mpu.getFIFOCount();

    // Controlar overflow
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } 
    else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

  // Mostrar Yaw, Pitch, Roll
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
}
