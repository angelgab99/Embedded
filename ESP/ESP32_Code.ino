#include <WiFi.h>
#include <WebSocketsServer.h>

const char* ssid = "ESP32";
const char* password = "246810ES!";

#define LED 2
#define BTN 4

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // 50 ms

WebSocketsServer webSocket = WebSocketsServer(81);

bool ledState = false;

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  pinMode(BTN, INPUT_PULLUP); // botón en GPIO4

  // Configurar como Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  Serial.println();
  Serial.print("ESP32 en modo AP. IP: ");
  Serial.println(WiFi.softAPIP());  // → normalmente 192.168.4.1

  // Iniciar WebSocket
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
}

void loop() {
  // Mantener WebSocket activo
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

