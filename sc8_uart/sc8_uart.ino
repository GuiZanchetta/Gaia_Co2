#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>

// Defina os pinos para o sensor de CO2
#define RX_PIN 17  // GPIO17
#define TX_PIN 16  // GPIO16

// Configuração da rede Wi-Fi
const char* ssid = "zan";
const char* password = "12345678";

// Endereço IP e porta do destino OSC
const char* destIP = "192.168.72.136"; // IP do computador que receberá os dados
const int destPort = 8000;            // Porta de destino OSC

// Comunicação serial com o sensor (UART1)
HardwareSerial co2Serial(1);
WiFiUDP Udp;

void setup() {
  Serial.begin(115200);

  // Inicializa a serial do sensor
  co2Serial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  // Conexão Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Conectando ao WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi conectado.");
  Serial.print("Endereço IP: ");
  Serial.println(WiFi.localIP());

  // Inicializa UDP
  Udp.begin(destPort);
}

void loop() {
  if (co2Serial.available() >= 16) {
    uint8_t buffer[16];
    co2Serial.readBytes(buffer, 16);

    if (buffer[0] == 0x42 && buffer[1] == 0x4D) {
      uint16_t co2_concentration = (buffer[6] << 8) | buffer[7];
      uint8_t checksum = 0;

      for (int i = 0; i < 15; i++) {
        checksum += buffer[i];
      }

      if (checksum == buffer[15]) {
        Serial.print("CO2: ");
        Serial.println(co2_concentration);

        // Envia valor por OSC
        OSCMessage msg("/co2");
        msg.add((int32_t)co2_concentration);
        Udp.beginPacket(destIP, destPort);
        msg.send(Udp);
        Udp.endPacket();
        msg.empty();

      } else {
        Serial.println("Checksum error: Invalid data");
      }
    } else {
      Serial.println("Invalid header: Data not recognized");
    }
  }
}
