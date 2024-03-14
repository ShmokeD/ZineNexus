#define ENA 4
#define ENB 13
#define IN_1 0
#define IN_2 2
#define IN_3 15
#define IN_4 5
const int LED_PIN = 16;

#include <WiFiUdp.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>

const char* ssid = "Hot Signals In Your Area";
const char* password = "nonumbers1";
ESP8266WebServer server(80);
ESP8266HTTPUpdateServer httpUpdater;


int botSpeed = 105;
void setup() {
  WiFi.begin(ssid, password);
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT);
  pinMode(LED_PIN, OUTPUT);



  digitalWrite(IN_1, HIGH);
  digitalWrite(IN_2, LOW);  
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, HIGH);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);


  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Waiting to connect ...");
  }

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/blink", blink);
  server.on("/turn", turn);
  server.on("/stop", stop);
  server.on("/start", start);
  server.on("/basic", basicMovement);
  server.on("/rotate", rotate);

  httpUpdater.setup(&server);
  server.begin();

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void loop() {

  server.handleClient();
}


void start() {
  int runTime = 0;
  runTime = server.arg("time").toInt();
  if (server.hasArg("time")) {
    server.send(200, "text/plain", "Start Movement " + String(runTime));
    Serial.println("Started " + String(runTime));

  } else {
    server.send(200, "text/plain", "Start Movement No Args");
    Serial.println("started with no args");
  }

  if (server.hasArg("speed")) {
    botSpeed = server.arg("speed").toInt();
  }

  startMovement();

  if (runTime == 0) {
    return;
  }
  delay(runTime);
  stopMovement();
  Serial.println("STopped");
}

void stop() {
  server.send(200, "text/plain", "Stop Movement");
  stopMovement();
}

void stopMovement() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
void startMovement() {
  analogWrite(ENA, botSpeed);
  analogWrite(ENB, botSpeed);
}

void basicMovement() {
  server.send(200, "text/plain", "Running Basic Tests");
  for (int i = 1; i <= 5; i++) {

    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);


    startMovement();
    delay(1000);
    analogWrite(ENA, 76.5);
    delay(1000);
    startMovement();
    delay(100);
    analogWrite(ENB, 76.5);
    delay(1000);
    startMovement();
    delay(100);

    stopMovement();
    delay(500);
  }
}


void blink() {

  server.send(200, "text/plain", "Blinking");
  int blinks = 2;
  stopMovement();
  if (server.hasArg("plain")) {
    blinks = server.arg("plain").toInt();
    Serial.print("Received arguement blink ");
    Serial.println(blinks);
  }

  for (int i = 1; i <= blinks; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}


void turn() {
  if (server.hasArg("plain") == false) {
    Serial.println("No Arguement Turn");
    server.send(200, "text/plain", "Body not received");
    return;
  } else {
    server.send(200, "text/plain", "Turning");
  }
  int turnValue = server.arg("plain").toInt();


  Serial.print("Received TurnValue ");
  Serial.println(turnValue);



  if (turnValue > 65 && turnValue <= 255) {
    analogWrite(ENB, 0);
    analogWrite(ENA, turnValue);
  }

  else if (turnValue > 320) {
    analogWrite(ENA, 0);
    analogWrite(ENB, turnValue - 256);
  }

  else {
    startMovement();
  }
}
void rotate() {
  int timeToRotate = server.arg("time").toInt();
  int direction = server.arg("dirn").toInt();
  if (!server.hasArg("time") && !server.hasArg("dirn")) {
    Serial.println("No Arguement Turn");
    server.send(200, "text/plain", "Body not received");
    return;
  } else {
    server.send(200, "text/plain", "Rotating for " + String(timeToRotate) + " in " + String(direction));
  }
  stopMovement();


  if (direction == 1) {
    digitalWrite(IN_1, HIGH);
    digitalWrite(IN_2, LOW);
    digitalWrite(IN_3, HIGH);
    digitalWrite(IN_4, LOW);

  } else {
    digitalWrite(IN_1, LOW);
    digitalWrite(IN_2, HIGH);
    digitalWrite(IN_3, LOW);
    digitalWrite(IN_4, HIGH);
  }


  startMovement();
  delay(timeToRotate);
  stopMovement();

  digitalWrite(IN_1, HIGH);
  digitalWrite(IN_2, LOW);
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, HIGH);
}
