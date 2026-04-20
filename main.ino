//#include <RunningMedian.h>
#include <NewPing.h>
#include <math.h>
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
unsigned long ultima_mudanca_valvula_ms = 0;

NewPing sonar(PIN_TRIG, PIN_ECHO, 120);

// --- Parametros de estabilizacao ---
const float NIVEL_ABERTURA_AUTO = 28.0;
const float NIVEL_FECHAMENTO_AUTO = 52.0;
const float NIVEL_FECHAMENTO_SEGURANCA = 98.0;
const float NIVEL_REARME_AUTO = 38.0;

const float DISTANCIA_MIN_VALIDA_CM = 20.0;
const float DISTANCIA_MAX_VALIDA_CM = ALTURA_SENSOR_BASE + 1.0;
const float MAX_SALTO_NIVEL_VALIDO = 8.0;       // por ciclo de 2s
const int MAX_OUTLIERS_CONSECUTIVOS = 3;        // aceita salto se persistir
const float EMA_ALPHA = 0.20;
const float MAX_PASSO_FILTRO_POR_CICLO = 1.5;   // limita variacao no dashboard

const unsigned long TEMPO_MIN_LIGADA_MS = 120000;
const unsigned long TEMPO_MIN_DESLIGADA_MS = 60000;
const int LEITURAS_CONSECUTIVAS_ABRIR = 3;
const int LEITURAS_CONSECUTIVAS_FECHAR_AUTO = 3;
const int LEITURAS_CONSECUTIVAS_FECHAR_SEGURANCA = 2;

bool filtro_inicializado = false;
float ultimo_nivel_bruto_aceito = 0.0;
float nivel_filtrado = 0.0;
int outliers_consecutivos = 0;
int rejeicoes_leitura = 0;
int leituras_aceitas = 0;
int contagem_confirmacao_abertura = 0;
int contagem_confirmacao_fechamento_auto = 0;
int contagem_confirmacao_fechamento_seg = 0;

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
  ultima_mudanca_valvula_ms = millis();
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
  bool leitura_valida = true;
  const char* motivo_rejeicao = "";

  if (uS == 0) {
    leitura_valida = false;
    motivo_rejeicao = "echo_timeout";
  } else if (distancia_cm < DISTANCIA_MIN_VALIDA_CM || distancia_cm > DISTANCIA_MAX_VALIDA_CM) {
    leitura_valida = false;
    motivo_rejeicao = "fora_faixa_fisica";
  }

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
  float nivel_bruto = ((ALTURA_SENSOR_BASE - distancia_cm) / ALTURA_UTIL_AGUA) * 100.0;
  nivel_bruto = constrain(nivel_bruto, 0, 100);

  if (leitura_valida && filtro_inicializado) {
    float salto = fabs(nivel_bruto - ultimo_nivel_bruto_aceito);
    if (salto > MAX_SALTO_NIVEL_VALIDO) {
      outliers_consecutivos++;
      if (outliers_consecutivos < MAX_OUTLIERS_CONSECUTIVOS) {
        leitura_valida = false;
        motivo_rejeicao = "salto_brusco";
      } else {
        // Se o salto se repete por varios ciclos, pode ser mudanca real.
        outliers_consecutivos = 0;
      }
    } else {
      outliers_consecutivos = 0;
    }
  }

  if (leitura_valida) {
    if (!filtro_inicializado) {
      filtro_inicializado = true;
      nivel_filtrado = nivel_bruto;
      ultimo_nivel_bruto_aceito = nivel_bruto;
    } else {
      ultimo_nivel_bruto_aceito = nivel_bruto;
      float alvo_ema = (EMA_ALPHA * nivel_bruto) + ((1.0 - EMA_ALPHA) * nivel_filtrado);
      float delta = alvo_ema - nivel_filtrado;
      if (delta > MAX_PASSO_FILTRO_POR_CICLO) delta = MAX_PASSO_FILTRO_POR_CICLO;
      if (delta < -MAX_PASSO_FILTRO_POR_CICLO) delta = -MAX_PASSO_FILTRO_POR_CICLO;
      nivel_filtrado += delta;
    }

    nivel_percentual = constrain(nivel_filtrado, 0, 100);
    distancia_instantanea = (nivel_percentual / 100.0) * ALTURA_UTIL_AGUA;
    leituras_aceitas++;
  } else {
    rejeicoes_leitura++;
    // Mantem ultima leitura estavel para evitar serrilhado e chattering.
    nivel_percentual = constrain(nivel_filtrado, 0, 100);
    distancia_instantanea = (nivel_percentual / 100.0) * ALTURA_UTIL_AGUA;
  }

  Serial.print("Distância inst: ");
  Serial.print(distancia_instantanea);
  //Serial.print("cm | Media: ");
  //Serial.print(distancia_media);
  Serial.print("cm | Nível: "); 
  Serial.print(nivel_percentual);
  Serial.print("% | Bruto: ");
  Serial.print(nivel_bruto);
  Serial.print("% | uS: ");
  Serial.print(uS);
  Serial.print(" | Aceitas/Rej: ");
  Serial.print(leituras_aceitas);
  Serial.print("/");
  Serial.print(rejeicoes_leitura);
  if (!leitura_valida) {
    Serial.print(" | rejeitada=");
    Serial.print(motivo_rejeicao);
  }
  Serial.println();
}

void verificarAlarmes() { 

  bool condicaoCritica = (nivel_percentual <= NIVEL_ABERTURA_AUTO);

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
  unsigned long agora = millis();
  bool respeitou_tempo_min_ligada = (agora - ultima_mudanca_valvula_ms) >= TEMPO_MIN_LIGADA_MS;
  bool respeitou_tempo_min_desligada = (agora - ultima_mudanca_valvula_ms) >= TEMPO_MIN_DESLIGADA_MS;

  if (abrir_valvula) {
    if (nivel_percentual >= NIVEL_FECHAMENTO_SEGURANCA) {
      contagem_confirmacao_fechamento_seg++;
    } else {
      contagem_confirmacao_fechamento_seg = 0;
    }

    if (auto_aberto_por_nivel && nivel_percentual >= NIVEL_FECHAMENTO_AUTO) {
      contagem_confirmacao_fechamento_auto++;
    } else {
      contagem_confirmacao_fechamento_auto = 0;
    }

    if (contagem_confirmacao_fechamento_seg >= LEITURAS_CONSECUTIVAS_FECHAR_SEGURANCA) {
      abrir_valvula = false;
      auto_aberto_por_nivel = false;
      onAbrirValvulaChange();
      Serial.println("MANUAL/SEGURANÇA: Caixa cheia (98%). Válvula fechada.");
    }
    else if (auto_aberto_por_nivel &&
             contagem_confirmacao_fechamento_auto >= LEITURAS_CONSECUTIVAS_FECHAR_AUTO &&
             respeitou_tempo_min_ligada) {
      abrir_valvula = false;
      auto_aberto_por_nivel = false;
      onAbrirValvulaChange();
      Serial.println("AUTOMAÇÃO: Fechamento confirmado em >=52%.");
    }
  }

  // --- LÓGICA DE ABERTURA AUTOMÁTICA (IGUAL) ---
  else if (condicaoCritica && !abrir_valvula && !auto_aberto_por_nivel) {
    contagem_confirmacao_abertura++;
    if (contagem_confirmacao_abertura >= LEITURAS_CONSECUTIVAS_ABRIR && respeitou_tempo_min_desligada) {
      abrir_valvula = true;
      auto_aberto_por_nivel = true; 
      onAbrirValvulaChange();
      Serial.println("AUTOMAÇÃO: Nível crítico confirmado. Abrindo válvula.");
      contagem_confirmacao_abertura = 0;
    }
  } else {
    contagem_confirmacao_abertura = 0;
  }

  // Reseta a trava para permitir nova automação no futuro
  if (nivel_percentual > NIVEL_REARME_AUTO && !abrir_valvula) {
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
    ultima_mudanca_valvula_ms = millis();
    contagem_confirmacao_fechamento_auto = 0;
    contagem_confirmacao_fechamento_seg = 0;
    Serial.println("Válvula ABERTA manualmente.");
  } else {
    // DESLIGAR (Alta Impedância / Flutuante)
    pinMode(PIN_RELE, INPUT);
    ultima_mudanca_valvula_ms = millis();
    contagem_confirmacao_abertura = 0;
    auto_aberto_por_nivel = false;
    Serial.println("Válvula FECHADA manualmente.");
  }
}