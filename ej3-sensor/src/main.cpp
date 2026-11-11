/*
 * Ejercicio 3 - Sensor analógico con umbral e histéresis (ej3-sensor)
 * Lee un potenciómetro cada 100 ms con millis(), calcula un promedio móvil
 * de las últimas 8 muestras, escala el valor a 0.0 - 100.0 % y controla
 * una alarma con histéresis (activa > 70 %, desactiva < 60 %), imprimiendo
 * por Serial únicamente cuando cambia el estado de la alarma.
 */

#include <Arduino.h>

// Pines y comunicación serie
constexpr uint8_t PIN_SENSOR = 34;      // ADC1 en ESP32 (0 a 4095)
constexpr uint8_t PIN_LED_ALARMA = 2;
constexpr unsigned long SERIAL_BAUD = 115200;

// Muestreo y filtrado (promedio móvil)
constexpr unsigned long INTERVALO_MUESTREO_MS = 100;
constexpr size_t CANTIDAD_MUESTRAS = 8;

// Escalado del ADC a porcentaje (en Nano 10 bits sería 1023; en ESP32 12 bits es 4095)
constexpr int ADC_MAX = 4095;
constexpr float ESCALA_MAX_PCT = 100.0f;

// Umbrales de alarma con histéresis (%)
constexpr float UMBRAL_ALARMA_ON_PCT = 70.0f;
constexpr float UMBRAL_ALARMA_OFF_PCT = 60.0f;
constexpr uint8_t DECIMALES_IMPRESION = 1;

// Estado del filtro de promedio móvil
int muestras[CANTIDAD_MUESTRAS] = {0};
size_t indiceMuestra = 0;
size_t muestrasCargadas = 0;
long sumaMuestras = 0;

// Estado de la alarma y temporización
bool alarmaActiva = false;
unsigned long ultimoMuestreoMs = 0;

float actualizarPromedioMovil(int nuevaMuestra) {
  if (muestrasCargadas < CANTIDAD_MUESTRAS) {
    muestrasCargadas++;
  } else {
    // Resta la muestra más vieja antes de sobrescribirla
    sumaMuestras -= muestras[indiceMuestra];
  }

  muestras[indiceMuestra] = nuevaMuestra;
  sumaMuestras += nuevaMuestra;
  indiceMuestra = (indiceMuestra + 1) % CANTIDAD_MUESTRAS;

  return static_cast<float>(sumaMuestras) / static_cast<float>(muestrasCargadas);
}

float convertirAPorcentaje(float valorAdc) {
  return (valorAdc * ESCALA_MAX_PCT) / static_cast<float>(ADC_MAX);
}

void actualizarAlarma(float porcentaje) {
  const bool estadoAnterior = alarmaActiva;

  if (!alarmaActiva && (porcentaje > UMBRAL_ALARMA_ON_PCT)) {
    alarmaActiva = true;
  } else if (alarmaActiva && (porcentaje < UMBRAL_ALARMA_OFF_PCT)) {
    alarmaActiva = false;
  }

  if (alarmaActiva != estadoAnterior) {
    digitalWrite(PIN_LED_ALARMA, alarmaActiva ? HIGH : LOW);
    Serial.print("Alarma: ");
    Serial.print(alarmaActiva ? "ACTIVA" : "INACTIVA");
    Serial.print(" (");
    Serial.print(porcentaje, DECIMALES_IMPRESION);
    Serial.println(" %)");
  }
}

void setup() {
  pinMode(PIN_LED_ALARMA, OUTPUT);
  digitalWrite(PIN_LED_ALARMA, LOW);
  pinMode(PIN_SENSOR, INPUT);

  Serial.begin(SERIAL_BAUD);
  Serial.println();
  Serial.println("Inicio - Ejercicio 3: Sensor analogico con umbral e histeresis (ej3-sensor)");

  ultimoMuestreoMs = millis();
}

void loop() {
  const unsigned long ahoraMs = millis();

  if (ahoraMs - ultimoMuestreoMs >= INTERVALO_MUESTREO_MS) {
    ultimoMuestreoMs += INTERVALO_MUESTREO_MS;

    const int lecturaCruda = analogRead(PIN_SENSOR);
    const float promedioAdc = actualizarPromedioMovil(lecturaCruda);
    const float porcentaje = convertirAPorcentaje(promedioAdc);

    actualizarAlarma(porcentaje);
  }
}
