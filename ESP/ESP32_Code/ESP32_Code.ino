#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include "lm75.h"
#include <mcp_can.h>
#include <SPI.h>

const char* ssid = "ESP32";
const char* password = "246810ES!";
lm75_t temp_sensor;
extern "C" {
  bool lm75_i2c_write_read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len);
}

#include <stdint.h>

#define LED 2
#define BTN 4
#define POT_PIN A0 // ADC único del ESP8266
#define SPEEDLMT 200

#define CAN0_INT 15      // Set INT to pin D2 related to GPIO15
#define CAN0_CS 5       // Set CS to pin D8 related to GPIO5 - ESP32
//#define CAN0_CS D8      // ESP8266
MCP_CAN CAN0(CAN0_CS);     

// Variables
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // 50 ms
unsigned long prevTX = 0;  //to sotre lasst exectution time
const unsigned int invlTX = 1000; // intervalo de envío CAN (ms)

uint32_t tempBits;
byte data[6];  
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];

 

// CAN VARIABLES
unsigned long prevTX = 0;                                        // Variable to store last execution time
//const unsigned int invlTX = 1000;                                // One second interval constant
const unsigned int invlTX = 10000;                                // Three seconds interval constant

WebSocketsServer webSocket = WebSocketsServer(81);


lm75_t temp_sensor;
extern "C" {
  bool lm75_i2c_write_read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len);
}

void speedReadADC_init(uint8_t resolution);
uint8_t speedReadADC_loop(int potentiometer, uint8_t speedLmt, int resolution);
void sendCANMessage(unsigned long id, byte *data, byte len);

void setup() {
  Serial.begin(115200);

  // Pines
  pinMode(LED, OUTPUT);
  pinMode(BTN, INPUT_PULLUP);

  // Configurar ADC (ESP8266 tiene 10 bits)
  speedReadADC_init();

  // Configurar ESP8266 como Access Point
  WiFi.softAP(ssid, password);
  Serial.println();
  Serial.print("ESP8266 en modo AP. IP: ");
  Serial.println(WiFi.softAPIP());

  // Iniciar WebSocket
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  // Sensor de temperatura LM75
  Wire.begin(D3, D1); // ESP8266: SDA=D2, SCL=D1
  lm75_init(&temp_sensor, LM75_SLAVE_ADDR, lm75_i2c_write_read);
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 inicializado correctamente.");
  else {
    Serial.println("Error al inicializar MCP2515.");
    while(1){}
  }

  // CAN0.setMode(MCP_NORMAL);
  // Serial.println("CAN listo en modo NORMAL.");
  if(CAN0.setMode(MCP_NORMAL) == CAN_OK)
    Serial.println("Modo normal activado");
  else
    Serial.println("Error al activar normal");
  delay(1000);

}

void loop() {

  //Loop de velocidad
  uint8_t speed = speedReadADC_loop(POT_PIN, SPEEDLMT, RESOLUTION);

  //Mantener WebSocket activo
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

  // delay(2000);
  // Mantener WebSocket activo
  webSocket.loop();

  // Si el valor cambió respecto a la última lectura
  if (currentReading != lastButtonReading) {
    lastDebounceTime = millis(); // reiniciar el temporizador
  }
  lastButtonReading = currentReading;

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (currentReading != lastButtonStable) {
      lastButtonStable = currentReading;
      if (lastButtonStable == LOW) {
        lockState = !lockState;
        digitalWrite(LED, lockState ? HIGH : LOW);
        webSocket.broadcastTXT(lockState ? "1" : "0");
        // Leer velocidad desde el potenciómetro
        speed = speedReadADC_loop(POT_PIN, SPEEDLMT);

        //leer temperatura
        if (lm75_read_temp_c(&temp_sensor, &temp))
        {
          tempBits = *((uint32_t*)&temp);
        }
        else{
          tempBits = 0x00;
        }
        data[0] = (0x01 & lockState);
        data[1] = (0xFF & speed);
        data[2] = (tempBits & 0xFF);
        data[3] = ((tempBits >> 8) & 0xFF);
        data[4] = ((tempBits >> 16) & 0xFF);
        data[5] = ((tempBits >> 24) & 0xFF);
        CAN0.sendMsgBuf(0x01, 0, 6, data);

      }
    }
  }
  //CODIGO CAN PARA ENVIAR CADA SEGUNDO 
  if (millis() - prevTX >= invlTX) {
    prevTX = millis();

    /*byte speedData[1];
    speedData[0] = speed;
    sendCANMessage(0x100, speedData, 1);*/

    int16_t tempInt = (int16_t)(temp * 100); 
    byte tempData[2];
    tempData[0] = tempInt >> 8;
    tempData[1] = tempInt & 0xFF;
    sendCANMessage(0x101, tempData, 2);

    /*byte btnData[1];
    btnData[0] = ledState ? 1 : 0;
    sendCANMessage(0x102, btnData, 1);*/

    Serial.printf("CAN → Vel:%d  Temp:%.2f°C  Btn:%d\n", speed, temp, ledState);
  }
}

// --- Funciones auxiliares ---
void speedReadADC_init() {
  // ESP8266 ADC tiene resolución fija de 10 bits, no se puede cambiar
  Serial.println("ADC inicializado (10 bits, ESP8266)");
}

uint8_t speedReadADC_loop(int potentiometer, uint8_t speedLmt) {
  uint8_t speed = (analogRead(potentiometer) * speedLmt) / 1023; // 10 bits
  return speed;
}

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("Cliente [%u] conectado\n", num);
      webSocket.sendTXT(num, lockState ? "1" : "0");
      break;
    case WStype_DISCONNECTED:
      Serial.printf("Cliente [%u] desconectado\n", num);
      break;
    case WStype_TEXT: {
      String msg = String((char*)payload).substring(0, length);
      Serial.printf("Mensaje de [%u]: %s\n", num, msg.c_str());
      if (msg == "1") { digitalWrite(LED, HIGH); lockState = true; webSocket.broadcastTXT("1"); }
      else if (msg == "0") { digitalWrite(LED, LOW); lockState = false; webSocket.broadcastTXT("0"); }
      break;
    }
  }
}

void sendCANMessage(unsigned long id, byte *data, byte len) {
  byte status = CAN0.sendMsgBuf(id, 0, len, data);
  if (status == CAN_OK)
    Serial.printf("Mensaje CAN ID 0x%03lX enviado OK\n", id);
  else
    Serial.printf("Error al enviar CAN ID 0x%03lX\n", id);
}

