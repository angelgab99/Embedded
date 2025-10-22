#include <WiFi.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include "lm75.h"

const char* ssid = "ESP32";
const char* password = "246810ES!";
lm75_t temp_sensor;
extern "C" {
  bool lm75_i2c_write_read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len);
}
//comentario para test
//comentario de Luis
#include <stdint.h>

#define LED 2
#define BTN 4
#define POT_PIN 36 //ADC0 pin36
#define RESOLUTION 8
#define SPEEDLMT 200

const char* ssid = "ESP32";
const char* password = "246810ES!";


unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // 50 ms

WebSocketsServer webSocket = WebSocketsServer(81);

bool ledState = false;

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

void speedReadADC_init(uint8_t resolution);
uint8_t speedReadADC_loop(int potentiometer, uint8_t speedLmt, int resolution);

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  pinMode(BTN, INPUT_PULLUP); // botón en GPIO4

  //Configurar Resolución ADC
  speedReadADC_init(RESOLUTION);
  

  // Configurar como Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  Serial.println();
  Serial.print("ESP32 en modo AP. IP: ");
  Serial.println(WiFi.softAPIP());  // → normalmente 192.168.4.1

  // Iniciar WebSocket
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  // Temperature sensor setup
  Wire.begin(21, 22); // SDA=21, SCL=22 
  lm75_init(&temp_sensor, LM75_SLAVE_ADDR, lm75_i2c_write_read);
}

void loop() {

  //Loop de velocidad
  uint8_t speed = speedReadADC_loop(POT_PIN, SPEEDLMT, RESOLUTION);

  // Mantener WebSocket activo
  //COMENTARIO TEST
  // webSocket.loop();

  // static bool lastButtonStable = HIGH;   // último estado estable del botón
  // static bool lastButtonReading = HIGH;  // última lectura cruda
  // bool currentReading = digitalRead(BTN);

  // // Si el valor cambió respecto a la última lectura
  // if (currentReading != lastButtonReading) {
  //   lastDebounceTime = millis(); // reiniciar el temporizador
  // }
  // lastButtonReading = currentReading;

  // // Solo cambiar estado si ya pasó el tiempo de debounce
  // if ((millis() - lastDebounceTime) > debounceDelay) {
  //   // Si hay un cambio respecto al último estado estable
  //   if (currentReading != lastButtonStable) {
  //     lastButtonStable = currentReading;

  //     // Detectar flanco de bajada (botón presionado)
  //     if (lastButtonStable == LOW) {
  //       // XOR para alternar
  //       ledState = ledState ^ 1;
  //       if (ledState) {
  //         digitalWrite(LED, HIGH);
  //         webSocket.broadcastTXT("1");
  //       } 
  //       else {
  //           digitalWrite(LED, LOW);
  //           webSocket.broadcastTXT("0");
  //       }
  //     }
  //   }
  // }

  // Tmperature sensor
  float temp;
  if (lm75_read_temp_c(&temp_sensor, &temp))
    Serial.printf("Temperature: %.3f °C\n", temp);
  else
    Serial.println("Error while reading LM75 sensor");
  delay(1000);
  webSocket.loop();

  static bool lastButtonStable = HIGH;   // último estado estable del botón
  static bool lastButtonReading = HIGH;  // última lectura cruda
  bool currentReading = digitalRead(BTN);

  // Si el valor cambió respecto a la última lectura
  if (currentReading != lastButtonReading) {
    lastDebounceTime = millis(); // reiniciar el temporizador
  }
  lastButtonReading = currentReading;

  // Solo cambiar estado si ya pasó el tiempo de debounce
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // Si hay un cambio respecto al último estado estable
    if (currentReading != lastButtonStable) {
      lastButtonStable = currentReading;

      // Detectar flanco de bajada (botón presionado)
      if (lastButtonStable == LOW) {
        // XOR para alternar
        ledState = ledState ^ 1;
        if (ledState) {
          digitalWrite(LED, HIGH);
          webSocket.broadcastTXT("1");
        } 
        else {
            digitalWrite(LED, LOW);
            webSocket.broadcastTXT("0");
        }
      }
    }
  }
}

void speedReadADC_init(uint8_t resolution){
  analogReadResolution(resolution);
  Serial.print("ADC read Init");
}

uint8_t speedReadADC_loop(int potentiometer, uint8_t speedLmt, int resolution){
  uint8_t speed = (analogRead(POT_PIN) * speedLmt)/(1 << resolution);
  Serial.print(speed);
  return speed;
}


void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED: {
      Serial.printf("Cliente [%u] conectado\n", num);
      // Al conectar, enviar el estado actual del LED
      if (ledState) webSocket.sendTXT(num, "1");
      else webSocket.sendTXT(num, "0");
      break;
    }

    case WStype_DISCONNECTED: {
      Serial.printf("Cliente [%u] desconectado\n", num);
      break;
    }

    case WStype_TEXT: {
      String msg = String((char*)payload).substring(0, length);
      Serial.printf("Mensaje recibido de [%u]: %s\n", num, msg.c_str());

      if (msg == "1") {
        digitalWrite(LED, HIGH);
        ledState = true;
        // Avisar a todos los clientes
        webSocket.broadcastTXT("1");
      } 
      else if (msg == "0") {
        digitalWrite(LED, LOW);
        ledState = false;
        webSocket.broadcastTXT("0");
      }
      break;
    }
  }
}

