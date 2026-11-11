/*
 * Ejercicio 2 - Botón y máquina de estados (ej2-boton)
 * Máquina de 3 modos (APAGADO, FIJO, PARPADEO) controlada con un pulsador
 * con antirrebote por software (40 ms) y detección de flanco descendente.
 * Toda la temporización (antirrebote, parpadeo y beep de confirmación) usa millis().
 */

#include <Arduino.h>

// Pines y comunicación serie
constexpr uint8_t PIN_LED = 2;
constexpr uint8_t PIN_BOTON = 4;
constexpr uint8_t PIN_BUZZER = 18;
constexpr unsigned long SERIAL_BAUD = 115200;

// Umbrales y tiempos (en milisegundos)
constexpr unsigned long DEBOUNCE_MS = 40;
constexpr unsigned long INTERVALO_PARPADEO_MS = 500;
constexpr unsigned long DURACION_BEEP_MS = 80;

// Frecuencias del buzzer por modo (Hz)
constexpr unsigned int FREQ_BEEP_APAGADO_HZ = 600;
constexpr unsigned int FREQ_BEEP_FIJO_HZ = 1000;
constexpr unsigned int FREQ_BEEP_PARPADEO_HZ = 1400;

// Modos de la máquina de estados
enum class Modo {
  APAGADO,
  FIJO,
  PARPADEO
};

// Estado actual del sistema (el modo vive en una sola variable)
Modo modoActual = Modo::APAGADO;

// Estado del antirrebote por software
int lecturaAnteriorBoton = HIGH;
int estadoEstableBoton = HIGH;
unsigned long ultimoCambioBotonMs = 0;

// Estado del parpadeo (reutilizado del ejercicio 1) y del buzzer
bool ledEncendido = false;
unsigned long ultimoParpadeoMs = 0;
bool buzzerActivo = false;
unsigned long inicioBeepMs = 0;

const char* obtenerNombreModo(Modo modo) {
  switch (modo) {
    case Modo::APAGADO:
      return "APAGADO";
    case Modo::FIJO:
      return "FIJO";
    case Modo::PARPADEO:
      return "PARPADEO";
  }
  return "DESCONOCIDO";
}

void iniciarBeep(unsigned int frecuenciaHz, unsigned long ahoraMs) {
  tone(PIN_BUZZER, frecuenciaHz);
  inicioBeepMs = ahoraMs;
  buzzerActivo = true;
}

void actualizarBuzzer(unsigned long ahoraMs) {
  if (buzzerActivo && (ahoraMs - inicioBeepMs >= DURACION_BEEP_MS)) {
    noTone(PIN_BUZZER);
    buzzerActivo = false;
  }
}

void actualizarParpadeo(unsigned long ahoraMs) {
  if (ahoraMs - ultimoParpadeoMs >= INTERVALO_PARPADEO_MS) {
    ultimoParpadeoMs += INTERVALO_PARPADEO_MS;
    ledEncendido = !ledEncendido;
    digitalWrite(PIN_LED, ledEncendido ? HIGH : LOW);
  }
}

/**
 * Devuelve true una sola vez por cada pulsación válida (flanco HIGH -> LOW),
 * filtrando rebotes durante DEBOUNCE_MS.
 */
bool detectarPulsacion(unsigned long ahoraMs) {
  const int lecturaActual = digitalRead(PIN_BOTON);

  if (lecturaActual != lecturaAnteriorBoton) {
    ultimoCambioBotonMs = ahoraMs;
    lecturaAnteriorBoton = lecturaActual;
  }

  if (ahoraMs - ultimoCambioBotonMs >= DEBOUNCE_MS) {
    if (lecturaActual != estadoEstableBoton) {
      estadoEstableBoton = lecturaActual;
      // Con INPUT_PULLUP, el botón presionado lleva el pin a LOW
      if (estadoEstableBoton == LOW) {
        return true;
      }
    }
  }

  return false;
}

void cambiarSiguienteModo(unsigned long ahoraMs) {
  unsigned int frecuenciaBeep = FREQ_BEEP_APAGADO_HZ;

  switch (modoActual) {
    case Modo::APAGADO:
      modoActual = Modo::FIJO;
      frecuenciaBeep = FREQ_BEEP_FIJO_HZ;
      break;
    case Modo::FIJO:
      modoActual = Modo::PARPADEO;
      frecuenciaBeep = FREQ_BEEP_PARPADEO_HZ;
      // Arranca el parpadeo con el LED encendido un ciclo completo
      ultimoParpadeoMs = ahoraMs;
      ledEncendido = true;
      digitalWrite(PIN_LED, HIGH);
      break;
    case Modo::PARPADEO:
      modoActual = Modo::APAGADO;
      frecuenciaBeep = FREQ_BEEP_APAGADO_HZ;
      break;
  }

  Serial.print("modo: ");
  Serial.println(obtenerNombreModo(modoActual));
  iniciarBeep(frecuenciaBeep, ahoraMs);
}

void actualizarSalidaModo(unsigned long ahoraMs) {
  switch (modoActual) {
    case Modo::APAGADO:
      ledEncendido = false;
      digitalWrite(PIN_LED, LOW);
      break;
    case Modo::FIJO:
      ledEncendido = true;
      digitalWrite(PIN_LED, HIGH);
      break;
    case Modo::PARPADEO:
      actualizarParpadeo(ahoraMs);
      break;
  }
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  pinMode(PIN_BOTON, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);

  lecturaAnteriorBoton = digitalRead(PIN_BOTON);
  estadoEstableBoton = lecturaAnteriorBoton;

  Serial.begin(SERIAL_BAUD);
  Serial.println();
  Serial.println("Inicio - Ejercicio 2: Boton y maquina de estados (ej2-boton)");
  Serial.print("modo: ");
  Serial.println(obtenerNombreModo(modoActual));

  const unsigned long inicioMs = millis();
  ultimoCambioBotonMs = inicioMs;
  ultimoParpadeoMs = inicioMs;
}

void loop() {
  const unsigned long ahoraMs = millis();

  if (detectarPulsacion(ahoraMs)) {
    cambiarSiguienteModo(ahoraMs);
  }

  actualizarSalidaModo(ahoraMs);
  actualizarBuzzer(ahoraMs);
}
