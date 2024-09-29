#include <WiFi.h>
#include <PubSubClient.h>
#include <Arduino.h>

// Configurações dos pinos
const int ppmPin = 5;         // Pino para o sinal PPM
const int zeroCrossPin = 34;  // Pino para o detector de cruzamento de zero
const int bypassPin = 19;     // Pino para o bypass
const int emergencyPin = 18;  // Pino para o relé de emergência
const int led = 21;
volatile bool zeroCrossDetected = false;
bool generatingPositivePulse = true;
bool generatingNegativePulse = false;

int estado = 1;
int pulseWidth = 600;          // Largura do pulso PPM em microssegundos (inicialmente 600us)
int posicao = 7800;            // Posição inicial do pulso (inicialmente 7800us)
int accelTime = 5;             // Tempo de aceleração em segundos
int decelTime = 5;             // Tempo de desaceleração em segundos
bool accelerating = true;      // Indicador de fase (aceleração ou desaceleração)
bool diminue = false;          // Indicador de desaceleração

unsigned long lastUpdate = 0;  // Tempo da última atualização da posição
const unsigned long updateInterval = 50; // Intervalo de atualização em milissegundos

// Configurações de Wi-Fi e MQTT
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";
const char* mqttServer = "YOUR_MQTT_SERVER";
const int mqttPort = 1883;
const char* mqttUser = "YOUR_MQTT_USER";    // Opcional, se o broker MQTT não usar autenticação, deixe vazio
const char* mqttPassword = "YOUR_MQTT_PASSWORD"; // Opcional

WiFiClient espClient;
PubSubClient client(espClient);

// Função para tratar mensagens MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  if (String(topic) == "control/accelTime") {
    accelTime = message.toInt();
    if (accelTime > 50) accelTime = 50;
    if (accelTime < 5) accelTime = 5;
  }
  
  if (String(topic) == "control/decelTime") {
    decelTime = message.toInt();
    if (decelTime > 50) decelTime = 50;
    if (decelTime < 5) decelTime = 5;
  }

  if (String(topic) == "control/start") {
    if (message == "start") {
      accelerating = true;
      posicao = 7000;
      diminue = false; 
    }
  }

  if (String(topic) == "control/stop") {
    if (message == "stop") {
      diminue = true;
      accelerating = false;
    }
  }

  Serial.print("Received message on topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  Serial.println(message);
}

// Conexão ao Wi-Fi
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.println("Endereço IP: ");
  Serial.println(WiFi.localIP());
}

// Conexão ao Broker MQTT
void reconnect() {
  while (!client.connected()) {
    Serial.print("Tentando conexão MQTT...");
    if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
      Serial.println("conectado");
      // Subscreve aos tópicos relevantes
      client.subscribe("control/accelTime");
      client.subscribe("control/decelTime");
      client.subscribe("control/start");
      client.subscribe("control/stop");
    } else {
      Serial.print("falhou, rc=");
      Serial.print(client.state());
      Serial.println(" tentando novamente em 5 segundos");
      delay(5000);
    }
  }
}

void IRAM_ATTR zeroCrossInterrupt() {
  zeroCrossDetected = true;
}

void setup() {
  // Inicializa a comunicação serial
  Serial.begin(115200);

  // Configura os pinos
  pinMode(ppmPin, OUTPUT);
  pinMode(zeroCrossPin, INPUT_PULLUP);
  pinMode(bypassPin, OUTPUT);
  pinMode(emergencyPin, OUTPUT);
  pinMode(led, OUTPUT);

  // Inicializa os pinos em nível baixo (desligado)
  digitalWrite(bypassPin, LOW);
  digitalWrite(emergencyPin, LOW);
  digitalWrite(led, LOW);

  // Configura o pino de interrupção para detectar cruzamento de zero
  attachInterrupt(digitalPinToInterrupt(zeroCrossPin), zeroCrossInterrupt, RISING);

  // Conecta ao Wi-Fi
  setup_wifi();
  
  // Configura MQTT
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Verifica se houve cruzamento de zero
  if (zeroCrossDetected) {
    zeroCrossDetected = false;
    unsigned long currentTime = millis();
    if (currentTime - lastUpdate >= updateInterval) {
      lastUpdate = currentTime;

      // Controle de aceleração e desaceleração
      if (accelerating) {
        posicao -= (7800 / (accelTime * 1000 / updateInterval)); // Decrementa a posição com base no tempo de aceleração
        if (posicao <= 210) { // Alcança o mínimo
          posicao = 210;
          digitalWrite(bypassPin, HIGH);
          accelerating = false; // Para de acelerar ao atingir o valor mínimo
        }
      } else if (diminue) {
        posicao += (7000 / (decelTime * 1000 / updateInterval)); // Incrementa a posição com base no tempo de desaceleração
        digitalWrite(bypassPin, LOW);
        if (posicao >= 7000) { // Limite máximo para a posição
          posicao = 7000;
          diminue = false; // Para de desacelerar ao atingir o valor máximo
        }
      }

      estado = 1;
    }

    if (estado == 1) {
      delayMicroseconds(posicao);
      
      digitalWrite(ppmPin, HIGH); // Pulso alto
      delayMicroseconds(pulseWidth); // Duração do pulso PPM
      digitalWrite(ppmPin, LOW); // Pulso baixo

      digitalWrite(led, LOW);
    } 
  } 
}
