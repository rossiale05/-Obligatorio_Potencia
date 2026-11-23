#include <Arduino.h>
#include <driver/gpio.h>
#include <esp_arduino_version.h>

// ESP32 clasica. Referencia: un flanco FALLING por ciclo de red.
constexpr uint8_t PIN_SYNC = 32;
constexpr uint8_t PIN_POT = 34; // Extremos del potenciómetro a 3V3 y GND
const uint8_t GATE[6] = {25, 26, 27, 18, 19, 23}; // DT1..DT6

// MODO DE CONTROL:
// true  = el ángulo lo controla el potenciómetro en GPIO34
// false = se utiliza el ángulo fijo definido en ANGULO_FIJO

constexpr bool USAR_POTENCIOMETRO = true;
//constexpr bool USAR_POTENCIOMETRO = false;
//constexpr uint16_t ANGULO_FIJO = 90; // Usado cuando USAR_POTENCIOMETRO = false
//constexpr uint16_t ANGULO_FIJO = 45;
//constexpr uint16_t ANGULO_FIJO = 135;

const uint8_t ORDEN[6] = {0, 5, 1, 3, 2, 4};      // DT1, DT6, DT2, DT4, DT3, DT5

constexpr uint32_t TICK_US = 50;
constexpr uint32_t PULSO_US = 200;
constexpr uint32_t FILTRO_US = 1000;
constexpr uint32_t T_MIN_US = 14285; // 70 Hz
constexpr uint32_t T_MAX_US = 25000; // 40 Hz

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t *timerDisparo = nullptr;

volatile uint32_t ultimoFlanco = 0;
volatile uint32_t referenciaPendiente = 0;
volatile uint32_t periodoPendiente = 0;
volatile bool primerFlanco = false;
volatile bool nuevaReferencia = false;
volatile bool sincronizado = false;
volatile uint16_t alfaSolicitado = 90; // Ángulo actualizado desde loop()
constexpr uint8_t MUESTRAS_POT = 32;
constexpr uint8_t CAMBIO_MIN_GRADOS = 2;

uint32_t referencia = 0, periodo = 20000;
uint32_t retardo[6], inicioPulso[6];
bool yaDisparo[6] = {}, salidaAlta[6] = {};

void IRAM_ATTR flancoDescendente() {
  uint32_t ahora = micros();
  portENTER_CRITICAL_ISR(&mux);

  if (!primerFlanco) {
    ultimoFlanco = ahora;
    primerFlanco = true;
  } else {
    uint32_t dt = ahora - ultimoFlanco;

    if (dt >= T_MIN_US && dt <= T_MAX_US) {
      ultimoFlanco = ahora;
      referenciaPendiente = ahora;
      periodoPendiente = dt;
      nuevaReferencia = true;
      sincronizado = true;
    } else if (dt > T_MAX_US) {
      ultimoFlanco = ahora;       // Recupera la referencia tras perder ciclos
      sincronizado = false;
      nuevaReferencia = false;
    }
    // Flancos dentro del filtro o períodos demasiado cortos se ignoran.
  }

  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR actualizarDisparos() {
  uint32_t ahora = micros();
  portENTER_CRITICAL_ISR(&mux);

  if (nuevaReferencia) {
    referencia = referenciaPendiente;
    periodo = periodoPendiente;
    nuevaReferencia = false;

    uint16_t alfa = alfaSolicitado;
    for (uint8_t p = 0; p < 6; p++) {
      uint8_t canal = ORDEN[p];
      uint16_t fase = (alfa + 60U * p) % 360U;
      retardo[canal] = (periodo * fase + 180U) / 360U;
      yaDisparo[canal] = false;
    }
  }

  if (sincronizado && uint32_t(ahora - ultimoFlanco) > periodo + periodo / 2) {
    sincronizado = false;
  }

  for (uint8_t i = 0; i < 6; i++) {
    if (salidaAlta[i] && uint32_t(ahora - inicioPulso[i]) >= PULSO_US) {
      gpio_set_level((gpio_num_t)GATE[i], 0);
      salidaAlta[i] = false;
    }
  }

  if (sincronizado) {
    uint32_t transcurrido = ahora - referencia;
    for (uint8_t i = 0; i < 6; i++) {
      if (!yaDisparo[i] && transcurrido >= retardo[i]) {
        gpio_set_level((gpio_num_t)GATE[i], 1);
        inicioPulso[i] = ahora;
        salidaAlta[i] = true;
        yaDisparo[i] = true;
      }
    }
  } else {
    for (uint8_t i = 0; i < 6; i++) {
      if (salidaAlta[i]) gpio_set_level((gpio_num_t)GATE[i], 0);
      salidaAlta[i] = false;
    }
  }

  portEXIT_CRITICAL_ISR(&mux);
}

void setup() {
  for (uint8_t i = 0; i < 6; i++) {
    pinMode(GATE[i], OUTPUT);
    digitalWrite(GATE[i], LOW);
  }

  pinMode(PIN_SYNC, INPUT); // La entrada debe tener niveles compatibles 0..3,3 V

  pinMode(PIN_POT, INPUT);
  analogReadResolution(12);                 // ADC: 0..4095
  analogSetPinAttenuation(PIN_POT, ADC_11db);

  attachInterrupt(digitalPinToInterrupt(PIN_SYNC), flancoDescendente, FALLING);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  timerDisparo = timerBegin(1000000); // 1 MHz
  timerAttachInterrupt(timerDisparo, &actualizarDisparos);
  timerAlarm(timerDisparo, TICK_US, true, 0);
#else
  timerDisparo = timerBegin(0, 80, true); // 1 MHz en ESP32 clasica
  timerAttachInterrupt(timerDisparo, &actualizarDisparos, true);
  timerAlarmWrite(timerDisparo, TICK_US, true);
  timerAlarmEnable(timerDisparo);
#endif
}

void loop() {
  if (USAR_POTENCIOMETRO) {
  // Promedia 32 lecturas y solo actualiza si el cambio supera 2 grados.
  static uint32_t ultimaLecturaMs = 0;
  static uint32_t suma = 0;
  static uint8_t muestras = 0;
  static uint16_t anguloFiltrado = 90;

  if (millis() - ultimaLecturaMs >= 5) {
    ultimaLecturaMs = millis();
    suma += analogRead(PIN_POT);

    if (++muestras >= MUESTRAS_POT) {
      uint16_t promedio = (suma + MUESTRAS_POT / 2) / MUESTRAS_POT;
      uint16_t lectura = (uint32_t(promedio) * 180U + 2047U) / 4095U;

      // Filtro pasa-bajos: amortigua pequeñas fluctuaciones del ADC.
      anguloFiltrado = (3U * anguloFiltrado + lectura + 2U) / 4U;

      uint16_t diferencia = (anguloFiltrado > alfaSolicitado)
                              ? anguloFiltrado - alfaSolicitado
                              : alfaSolicitado - anguloFiltrado;

      if (diferencia >= CAMBIO_MIN_GRADOS) {
        portENTER_CRITICAL(&mux);
        alfaSolicitado = anguloFiltrado;
        portEXIT_CRITICAL(&mux);
      }

      suma = 0;
      muestras = 0;
    }
  }
  }
}
