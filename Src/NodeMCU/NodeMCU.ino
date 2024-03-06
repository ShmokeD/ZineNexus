#define ENA 14
#define ENB 12
#define IN_1 15
#define IN_2 13
#define IN_3 2
#define IN_4 0
const int LED_PIN = 16;
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
const char* ssid = "Hot Signals In Your Area";
const char* password = "nonumbers1";
ESP8266WebServer server(80);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  WiFi.begin(ssid, password);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Waiting to connect ...");
  }

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/blink", blink);
  server.on("/turn", turn);
  server.on("/stop", stopMovement);
  server.on("/start", startMovement);
  server.on("/basic", basicMovement);
}

void loop() {
  server.handleClient();
}


void stopMovement() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
void startMovement() {
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
}

void basicMovement() {
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
  }

  int turnValue = server.arg("plain").toInt();
  Serial.print("Received TurnValue ");
  Serial.println(turnValue);

  if (turnValue == 0) {
    startMovement();
  }

  if (turnValue > 0) {
    analogWrite(ENA, turnValue);
  }

  else if (turnValue < 0) {
    analogWrite(ENB, abs(turnValue));
  }
}