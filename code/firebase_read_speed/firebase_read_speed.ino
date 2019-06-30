#include <ESP8266WiFi.h>
#include <Adafruit_NeoPixel.h>
#include <FirebaseArduino.h>
#include <Ticker.h>

#define FIREBASE_HOST "your-project.firebaseio.com"
#define FIREBASE_AUTH "your-project-password"
#define WIFI_SSID "wifi-ssid"
#define WIFI_PASSWORD "wifi-password"

int contconexion = 0;

#define LED_PIN D1
#define HALL_PIN D2

#define NUMPIXELS 8

// Publique a cada 5 s
#define PUBLISH_INTERVAL 1000*5

// Verifique a cada 1 s
#define LED_INTERVAL 1000*1

Ticker tickerPublish, tickerLamp;
int contaZero;
float avgSpeed, travelDistance;
bool publishNewState, publishNewLamp, movimentoSkate, led_last_state;
float wheelCirc = 0.16;
unsigned long lastRefresh;   // Para trabalhar com o millis(), use long, e não float.
volatile unsigned long loopCount;   // Contador de giros

Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

void speedCalc()
{
  loopCount++;
}

void publish(){
  publishNewState = true;
}

void publishLed(){
  publishNewLamp = true;
}

void setupPins(){
  pinMode(HALL_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void setupWifi(){

  WiFi.mode(WIFI_STA); //para que no inicie el SoftAP en el modo normal
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED and contconexion < 50) { //Cuenta hasta 50 si no se puede conectar lo cancela
    ++contconexion;
    delay(500);
//    Serial.print(".");
  }
  if (contconexion < 50) {
      //para usar con ip fija
       IPAddress Ip(XXX,XXX,XX,X);  //esp8266 ip
       IPAddress Gateway(XXX,XXX,XX,X);  //ipconfig gateway padrão
       IPAddress Subnet(XXX,XXX,XXX,X); //ipconfig sub-rede
       WiFi.config(Ip, Gateway, Subnet); 
      
      Serial.println("");
      Serial.println("WiFi conectado");
      Serial.println(WiFi.localIP());
  }
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
//  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
//    Serial.print(".");
    delay(500);
  }
//  Serial.println();
//  Serial.print("connected: ");
//  Serial.println(WiFi.localIP());
}

void setupFirebase(){
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.setBool("operation", false);
  Firebase.setBool("lamp", false);
  travelDistance = Firebase.getFloat("last_distance");
}

void setupVariables(){
  loopCount = 0;
  contaZero = 0;
  avgSpeed = 0.0;
  publishNewState = true;
  publishNewLamp = true;
  movimentoSkate = false;
  led_last_state = false;
  lastRefresh = millis();
}

void setupLed(){
  pixels.begin();
  pixels.show();
  pixels.setBrightness(25);
}

void setup() {

//  Serial.begin(9600);

  setupPins();
  setupWifi();    
  setupFirebase();
  setupVariables();
  setupLed();

  //Aciona o contador a cada pulso
  attachInterrupt(HALL_PIN, speedCalc, RISING);

  // Registra o ticker para publicar de tempos em tempos
  tickerPublish.attach_ms(PUBLISH_INTERVAL, publish);

  // Analisa o botão de tempos em tempos
  tickerLamp.attach_ms(LED_INTERVAL, publishLed);
}

void loop() {

  if(publishNewLamp){
    
    bool led_state = Firebase.getBool("lamp");
    
    if (led_state == true && led_last_state == false){
      for(int i=0; i<NUMPIXELS; i++) {
        pixels.setPixelColor(i, pixels.Color(255, 255, 255));
      }
      pixels.show();
      led_last_state = true;
    }

    if (led_state == false && led_last_state == true){
      pixels.clear();
      pixels.show();
      led_last_state = false;
    }

    publishNewLamp = false;
  }


  if (publishNewState)
  {
//    Serial.println("Publish new State");

    if (loopCount == 0 && contaZero < 3){
      avgSpeed = 0.0;

      if(movimentoSkate == true){
        Firebase.setBool("operation", false);
        movimentoSkate = false;
      }

      String avgSpeed_string = String(avgSpeed,1);
      String travelDistance_string = String(travelDistance,1);

      Firebase.pushString("speed", avgSpeed_string);
      Firebase.pushString("distance", travelDistance_string);

      contaZero++;
    }

    if(loopCount > 0){

      //Desabilita interrupcao durante o cálculo
      detachInterrupt(HALL_PIN);
 
      float deltaS = (wheelCirc*loopCount);
      unsigned long deltaT = (millis() - lastRefresh);
      avgSpeed = (3600*deltaS)/deltaT;

      travelDistance += deltaS;

      //Habilita interrupcao    
      attachInterrupt(HALL_PIN, speedCalc, RISING);

      if(movimentoSkate == false){
        Firebase.setBool("operation", true);
        movimentoSkate = true;
      }

      String avgSpeed_string = String(avgSpeed,1);
      String travelDistance_string = String(travelDistance,1);

      Firebase.pushString("speed", avgSpeed_string);
      Firebase.pushString("distance", travelDistance_string);
      Firebase.setFloat("last_distance", travelDistance);

      loopCount = 0;
      contaZero = 0;
    }

    lastRefresh = millis();
    publishNewState = false;
  }

}
