#include <ESP8266WiFi.h>
#include <Ticker.h>

#define HALL_PIN D2

// Publica a cada PUBLISH_INTERVAL milissegundos
#define PUBLISH_INTERVAL 1000*5

Ticker tickerPublish;
int contaZero;
float avgSpeed, travelDistance;
bool publishNewState;
float wheelCirc = 16;
unsigned long lastRefresh;
volatile unsigned long loopCount;

void speedCalc()
{
  loopCount++;
}

void publish(){
  publishNewState = true;
}

void setupPins(){
  pinMode(HALL_PIN, INPUT);
}

void setupVariables(){
  loopCount = 0;
  contaZero = 0;
  avgSpeed = 0.0;
  publishNewState = true;
  lastRefresh = millis();
}

void setup() {

  Serial.begin(9600);

  setupPins();
  setupVariables();

  //Aciona o contador a cada pulso
  attachInterrupt(HALL_PIN, speedCalc, RISING);

  //Define o ticker para publicar de tempos em tempos
  tickerPublish.attach_ms(PUBLISH_INTERVAL, publish);

}

void loop() {

  if (publishNewState)
  {

    if (loopCount == 0 && contaZero < 2){
      avgSpeed = 0.0;
      travelDistance = 0.0;

      contaZero++;
    }

    if(loopCount > 0){

      //Desabilita interrupcao durante o cálculo
      detachInterrupt(HALL_PIN);
 
      float deltaS = (wheelCirc*loopCount);
      unsigned long deltaT = (millis() - lastRefresh);
      avgSpeed = (1000*deltaS)/deltaT;
      travelDistance = deltaS;

      //Habilita interrupcao    
      attachInterrupt(HALL_PIN, speedCalc, RISING);

      Serial.print(loopCount);
      Serial.println(" voltas");

      String avgSpeed_string = String(avgSpeed, 2);
      String travelDistance_string = String(travelDistance, 1);

      Serial.print("speed [cm/s]: ");
      Serial.println(avgSpeed_string);
      Serial.print("distance [cm]: ");
      Serial.println(travelDistance_string);

      loopCount = 0;
      contaZero = 0;
    }

    lastRefresh = millis();
    publishNewState = false;
  }

}
