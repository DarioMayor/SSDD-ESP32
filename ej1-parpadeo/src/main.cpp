/*
 * Lógica no bloqueante con millis():
 * En cada vuelta de loop() se consulta el reloj actual (ahoraMs) sin usar delay().
 * Cada tarea guarda su última marca de tiempo y evalúa (ahoraMs - ultimaMarca >= intervalo)
 * para ejecutarse solo cuando vence su plazo. Así, el parpadeo del LED (500 ms) y
 * el uptime por Serial (2000 ms) corren en paralelo sin frenarse entre sí, y la resta
 * entre enteros sin signo evita fallos ante el desbordamiento de millis().
 */

#include <Arduino.h>

// Pines y comunicación serie
constexpr uint8_t PIN_LED = 2;
constexpr unsigned long SERIAL_BAUD = 115200;

// Intervalos y umbrales de tiempo (en milisegundos)
constexpr unsigned long INTERVALO_PARPADEO_MS = 500;
constexpr unsigned long INTERVALO_UPTIME_MS = 2000;
constexpr unsigned long MS_POR_SEGUNDO = 1000;

// Estado de las tareas no bloqueantes
bool ledEncendido = true;
unsigned long ultimoParpadeoMs = 0;
unsigned long ultimoUptimeMs = 0;

/**
 * Alterna el estado del LED cada INTERVALO_PARPADEO_MS sin bloquear el loop.
 */
void actualizarParpadeo(unsigned long ahoraMs) {
  if (ahoraMs - ultimoParpadeoMs >= INTERVALO_PARPADEO_MS) {
    // Sumar el intervalo fijo evita deriva acumulada entre ciclos
    ultimoParpadeoMs += INTERVALO_PARPADEO_MS;
    ledEncendido = !ledEncendido;
    digitalWrite(PIN_LED, ledEncendido ? HIGH : LOW);
  }
}

/**
 * Imprime por Serial el tiempo transcurrido desde el arranque cada INTERVALO_UPTIME_MS.
 */
void actualizarUptime(unsigned long ahoraMs) {
  if (ahoraMs - ultimoUptimeMs >= INTERVALO_UPTIME_MS) {
    ultimoUptimeMs += INTERVALO_UPTIME_MS;
    const unsigned long uptimeSegundos = ahoraMs / MS_POR_SEGUNDO;
    Serial.print("uptime: ");
    Serial.print(uptimeSegundos);
    Serial.println(" s");
  }
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, ledEncendido ? HIGH : LOW);

  Serial.begin(SERIAL_BAUD);
  Serial.println();
  Serial.println("Inicio - Ejercicio 1: Latido no bloqueante (ej1-parpadeo)");

  const unsigned long inicioMs = millis();
  ultimoParpadeoMs = inicioMs;
  ultimoUptimeMs = inicioMs;
}

void loop() {
  const unsigned long ahoraMs = millis();
  actualizarParpadeo(ahoraMs);
  actualizarUptime(ahoraMs);
}
