#include <Arduino.h>
// Configurações dos pinos
const int ppmPin = 5;         // Pino para o sinal PPM
const int zeroCrossPin = 34;  // Pino para o detector de cruzamento de zero
const int relayPin = 18;      // Pino para controlar o relé
const int bypassPin = 16;     // Pino para o bypass
const int emergencyPin = 17;  // Pino para o relé de emergência

// Configurações do sinal PPM
const longint ppmPeriod = 20000;  // Período do sinal PPM em microssegundos (20 ms)
const long int minPulseWidth = 5000; // Largura do pulso mínima em microssegundos
const long int maxPulseWidth = 15000; // Largura do pulso máxima em microssegundos (ajustada para evitar problemas)
const long int stepSize = 1000;    // Tamanho do incremento/decremento do sinal PPM

// Variáveis
int pulseWidth = minPulseWidth; // Largura do pulso inicial
volatile bool zeroCrossDetected = false;

void IRAM_ATTR zeroCrossInterrupt() {
  zeroCrossDetected = true;
}

void setup() {
  // Inicializa a comunicação serial
  Serial.begin(115200);

  // Configura os pinos
  pinMode(ppmPin, OUTPUT);
  pinMode(zeroCrossPin, INPUT_PULLUP);
  pinMode(relayPin, OUTPUT);
  pinMode(bypassPin, OUTPUT);
  pinMode(emergencyPin, OUTPUT);
  
  // Inicializa os pinos em nível baixo (desligado)
  digitalWrite(bypassPin, LOW);
  digitalWrite(emergencyPin, LOW);

  // Configura o pino de interrupção para detectar cruzamento de zero
  attachInterrupt(digitalPinToInterrupt(zeroCrossPin), zeroCrossInterrupt, FALLING);
}

void loop() {
  // Gera o sinal PPM
  static unsigned long lastPPMTime = 0;
  unsigned long currentTime = micros();

  if (currentTime - lastPPMTime >= ppmPeriod) {
    lastPPMTime = currentTime;

    // Verifica se pulseWidth está dentro dos limites válidos
    pulseWidth = constrain(pulseWidth, minPulseWidth, maxPulseWidth);

    // Gera o sinal PPM
    digitalWrite(ppmPin, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(ppmPin, LOW);
    delayMicroseconds(ppmPeriod - pulseWidth);
  }

  // Verifica se houve cruzamento de zero
  if (zeroCrossDetected) {
    zeroCrossDetected = false;
    Serial.println("Cruzamento de zero detectado!");

    // Controle do relé
    digitalWrite(relayPin, HIGH);   // Ativa o relé
    delay(1000);                    // Mantém o relé ativado por 1 segundo
    digitalWrite(relayPin, LOW);    // Desativa o relé
  }

  // Verifica se há dados disponíveis no monitor serial
  if (Serial.available() > 0) {
    char command = Serial.read(); // Lê o comando enviado pelo monitor serial

    // Ajusta a largura do pulso com base no comando recebido
    if (command == 'd') { // Diminuir o sinal PPM
      pulseWidth = max(minPulseWidth, pulseWidth - stepSize);
      Serial.print("Largura do pulso diminuída: ");
      Serial.println(pulseWidth);
    }
    else if (command == 'D') { // Aumentar o sinal PPM
      pulseWidth = min(maxPulseWidth, pulseWidth + stepSize);
      Serial.print("Largura do pulso aumentada: ");
      Serial.println(pulseWidth);
    }
    else if (command == 'b') { // Ativar o pino de bypass
      digitalWrite(bypassPin, HIGH);
      Serial.println("Pino de bypass acionado.");
    }
    else if (command == 'B') { // Desativar o pino de bypass
      digitalWrite(bypassPin, LOW);
      Serial.println("Pino de bypass desativado.");
    }
    else if (command == 'p') { // Ativar o pino de emergência
      digitalWrite(emergencyPin, HIGH);
      Serial.println("Pino de emergência acionado.");
    }
    else if (command == 'P') { // Desativar o pino de emergência
      digitalWrite(emergencyPin, LOW);
      Serial.println("Pino de emergência desativado.");
    }
    else {
      Serial.println("Comando desconhecido.");
    }
  }
}
