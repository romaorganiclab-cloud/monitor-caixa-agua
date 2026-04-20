//#include <RunningMedian.h>
#include <NewPing.h>
#include "thingProperties.h"

// --- Definição dos Pinos ---
#define PIN_TRIG D7
#define PIN_ECHO D6
#define PIN_LED_G D1
#define PIN_LED_Y D2
#define PIN_LED_R D3
#define PIN_RELE D5

// --- Constantes de Calibração ---
const float ALTURA_SENSOR_BASE = 105.5;    // Distância do sensor até o fundo
const float ALTURA_UTIL_AGUA = 80.5;       // Altura da coluna de água (100%)
const float DISTANCIA_CHEIO = 26.5;        // Sensor lê isso quando está cheio (105.5 - 80.5)

// --- Filtro de Média Móvel ---
//const int NUM_LEITURAS = 10;
//float leituras[NUM_LEITURAS];
//int indice_leitura = 0;

unsigned long ultimo_tempo_medicao = 0;
const long INTERVALO_MEDICAO = 2000;
bool auto_aberto_por_nivel = false;

NewPing sonar(PIN_TRIG, PIN_ECHO, 120);

void setup() {
  Serial.begin(9600);
  delay(1500);
  
  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_Y, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);
  
  // Inicializa o Relé DESLIGADO usando Alta Impedância
  pinMode(PIN_RELE, INPUT);
}

void loop() {
  ArduinoCloud.update();
  
  if (millis() - ultimo_tempo_medicao > INTERVALO_MEDICAO) {
    ultimo_tempo_medicao = millis();
    medirNivelComFiltro();
    verificarAlarmes();
  }
}

void medirNivelComFiltro() {
  //digitalWrite(PIN_TRIG, LOW);
  //delayMicroseconds(2);
  //digitalWrite(PIN_TRIG, HIGH);
  //delayMicroseconds(20);
  //digitalWrite(PIN_TRIG, LOW);

  //long duracao = pulseIn(PIN_ECHO, HIGH, 30000);
  //distancia_instantanea = duracao * 0.0343 / 2;
  unsigned int uS = sonar.ping_median(7);

  // calculamos la distancia en cm
  float distancia_cm = sonar.convert_cm(uS);
  

  // Validação: Ignora leituras fora do range físico da caixa
  /*if (distancia_instantanea >= DISTANCIA_CHEIO && distancia_instantanea <= 20) {
    leituras[indice_leitura] = distancia_instantanea;
    indice_leitura = (indice_leitura + 1) % NUM_LEITURAS;
  }

  float soma = 0;
  for (int i = 0; i < NUM_LEITURAS; i++) soma += leituras[i];
  float distancia_media = soma / NUM_LEITURAS;
  */

  // --- Cálculo da Porcentagem ---
  // Se a distância lida for 105.5, o nível é 0%. Se for 25, o nível é 100%.
  float nivel = ((ALTURA_SENSOR_BASE - distancia_cm) / ALTURA_UTIL_AGUA) * 100.0;
  nivel_percentual = constrain(nivel, 0, 100);
  
  // calculamos la distancia de la altura de agua en cm
  distancia_instantanea = ALTURA_SENSOR_BASE - distancia_cm;

  Serial.print("Distância inst: ");
  Serial.print(distancia_instantanea);
  //Serial.print("cm | Media: ");
  //Serial.print(distancia_media);
  Serial.print("cm | Nível: "); 
  Serial.print(nivel_percentual);
  Serial.println("%");
}

void verificarAlarmes() { 

  bool condicaoMedia = (nivel_percentual < 70);
  bool condicaoCritica = (nivel_percentual < 30);

  // Lógica dos LEDs (Visual)
  digitalWrite(PIN_LED_G, (nivel_percentual >= 70));
  digitalWrite(PIN_LED_Y, (nivel_percentual < 70 && nivel_percentual >= 30));
  digitalWrite(PIN_LED_R, (nivel_percentual < 30));

  // Lógica do Buzzer (Alarme)
  if (condicaoCritica) {
    alerta_ativo = true;
  } else {
    alerta_ativo = false;
  }

  // --- LÓGICA DE FECHAMENTO (DOIS NÍVEIS) ---
  if (abrir_valvula) {
    if (auto_aberto_por_nivel && nivel_percentual >= 50) {
      // Se foi a automação que abriu, ela para em 50%
      abrir_valvula = false;
      auto_aberto_por_nivel = false;
      onAbrirValvulaChange();
      Serial.println("AUTOMAÇÃO: Atingiu 50%. Válvula fechada.");
    } 
    else if (nivel_percentual >= 98) {
      // Se foi manual (auto_aberto_por_nivel é false), ou por segurança, para em 98%
      abrir_valvula = false;
      auto_aberto_por_nivel = false;
      onAbrirValvulaChange();
      Serial.println("MANUAL/SEGURANÇA: Caixa cheia (98%). Válvula fechada.");
    }
  }

  // --- LÓGICA DE ABERTURA AUTOMÁTICA (IGUAL) ---
  else if (condicaoCritica && !abrir_valvula && !auto_aberto_por_nivel) {
    abrir_valvula = true;
    auto_aberto_por_nivel = true; 
    onAbrirValvulaChange();
    Serial.println("AUTOMAÇÃO: Nível crítico detectado. Abrindo válvula.");
  }

  // Reseta a trava para permitir nova automação no futuro
  if (nivel_percentual > 35 && !abrir_valvula) {
    auto_aberto_por_nivel = false;
  }
}


/*
  Since AbrirValvula is READ_WRITE variable, onAbrirValvulaChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onAbrirValvulaChange()  {
  if (abrir_valvula) {
    // LIGAR (Ativar Relé - Active Low)
    pinMode(PIN_RELE, OUTPUT);
    digitalWrite(PIN_RELE, LOW);
    Serial.println("Válvula ABERTA manualmente.");
  } else {
    // DESLIGAR (Alta Impedância / Flutuante)
    pinMode(PIN_RELE, INPUT);
    auto_aberto_por_nivel = false;
    Serial.println("Válvula FECHADA manualmente.");
  }
}