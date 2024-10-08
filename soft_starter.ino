#include <Arduino.h>

// Configurações dos pinos
const int ppmPin = 5;         // Pino para o sinal PPM
const int zeroCrossPin = 34;  // Pino para o detector de cruzamento de zero
const int bypassPin = 19;     // Pino para o bypass
const int emergencyPin = 18;  // Pino para o relé de emergência
const int led = 21;
const int adcPin = 32;        // Pino ADC para medição
volatile bool zeroCrossDetected = false;

int estado = 1;
int pulseWidth = 600;          // Largura do pulso PPM em microssegundos (inicialmente 600us)
int posicao = 7800;            // Posição inicial do pulso (inicialmente 7800us)
int accelTime = 5;             // Tempo de aceleração em segundos
int decelTime = 5;             // Tempo de desaceleração em segundos
bool accelerating = true;      // Indicador de fase (aceleração ou desaceleração)
bool diminue = false;          // Indicador de desaceleração
int adcValue = 0;  // Leitura do valor ADC no pino 32
float voltage ;  // Converte para tensão (assumindo 3.3V como referência)
bool ppmHabilitado = true;  // Controle se o sinal PPM está habilitado

unsigned long lastUpdate = 0;  // Tempo da última atualização da posição
const unsigned long updateInterval = 50; // Intervalo de atualização em milissegundos

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
  pinMode(adcPin, INPUT); // Configuração do pino ADC para leitura analógica
  pinMode(led, OUTPUT); // Configura o LED como saída
  
  // Inicializa os pinos em nível baixo (desligado)
  digitalWrite(bypassPin, LOW);
  digitalWrite(emergencyPin, LOW);

  // Configura o pino de interrupção para detectar cruzamento de zero
  attachInterrupt(digitalPinToInterrupt(zeroCrossPin), zeroCrossInterrupt, RISING);
}

void loop() 
{
  // Verifica se há dados disponíveis no console serial
  if (Serial.available() > 0) {
    // Lê o caractere do console serial
    char input = Serial.read();

    // Ajusta o tempo de aceleração/desaceleração
    if (input == 'a') {
      accelTime += 1;
      if (accelTime > 50) accelTime = 50;
    } else if (input == 'A') {
      accelTime -= 1;
      if (accelTime < 5) accelTime = 5;
    } else if (input == 'd') {
      decelTime += 1;
      if (decelTime > 50) decelTime = 50;
    } else if (input == 'D') {
      decelTime -= 1;
      if (decelTime < 5) decelTime = 5;
    } else if (input == 's') { // Acelerar
      accelerating = true;
      posicao = 7000;
      diminue = false; // Interrompe a desaceleração quando 's' é pressionado
    } else if (input == 'r') { // Desacelerar
      diminue = true;
      accelerating = false; // Interrompe a aceleração quando 'r' é pressionado
    } else if (input == 'b') { // Ativar o pino de bypass
      digitalWrite(bypassPin, HIGH);
      Serial.println("Pino de bypass acionado.");
    } else if (input == 'B') { // Desativar o pino de bypass
      digitalWrite(bypassPin, LOW);
      Serial.println("Pino de bypass desativado.");
    } else if (input == 'p') { // Ativar o pino de emergência
      digitalWrite(emergencyPin, HIGH);
      Serial.println("Pino de emergência acionado.");
    } else if (input == 'P') { // Desativar o pino de emergência
      digitalWrite(emergencyPin, LOW);
      Serial.println("Pino de emergência desativado.");
    } else {
      Serial.println("Comando desconhecido.");
    }

    // Exibe os novos valores no console serial
    Serial.print("Tempo de Aceleração: ");
    Serial.print(accelTime);
    Serial.print(" s, Tempo de Desaceleração: ");
    Serial.println(decelTime);
  }

  // Verifica se houve cruzamento de zero
  if (zeroCrossDetected) 
  {
    zeroCrossDetected = false;
    unsigned long currentTime = millis();
    if (currentTime - lastUpdate >= updateInterval) {
      lastUpdate = currentTime;
      
      // Controle de aceleração e desaceleração
      if (accelerating) {
        digitalWrite(emergencyPin, LOW);
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
          digitalWrite(emergencyPin, HIGH);
          diminue = false; // Para de desacelerar ao atingir o valor máximo
        }
      }

      // Verificar se a corrente está em 150% e desativar o sinal PPM temporariamente
      if (voltage >= 1.44 && voltage < 1.67) // 150% da corrente nominal
      {
        Serial.println("Corrente a 150% - Desativando PPM");
        ppmHabilitado = false;  // Desabilita o sinal PPM
      } 
      else if (voltage >= 1.67) // 200% da corrente nominal
      {
        Serial.println("Corrente a 200% - Desligando motor imediatamente");
        digitalWrite(emergencyPin, HIGH);  // Desliga o motor
      } 
      else if (voltage < 1.44) // Corrente voltou abaixo de 150%, reabilitar PPM
      {
        Serial.println("Corrente normalizada - Reativando PPM");
        ppmHabilitado = true;  // Reabilita o sinal PPM
      }
    }

    // Sinal PPM controlado pela variável ppmHabilitado
    if (ppmHabilitado && estado == 1) {
      delayMicroseconds(posicao);
      
      digitalWrite(ppmPin, HIGH); // Pulso alto
      delayMicroseconds(pulseWidth); // Duração do pulso PPM
      digitalWrite(ppmPin, LOW); // Pulso baixo

      digitalWrite(led, LOW);
    } 
  } // Fechamento correto da verificação do cruzamento de zero

  // Leitura do valor ADC no pino 32
  adcValue = analogRead(adcPin);
  voltage = (adcValue / 4095.0) * 3.3;  // Converte o valor ADC para tensão



} // Fechamento correto da função loop
