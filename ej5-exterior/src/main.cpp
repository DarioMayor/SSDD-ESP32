/*
 * Ejercicio 5 (Camino A) - ESP32 con MQTT (ej5-exterior)
 * Conecta a Wi-Fi (Wokwi-GUEST) y a un broker MQTT público sin bloquear loop(),
 * publica cada 5 s un JSON {"seq", "ts", "valor"} en unmdp/sd2026/<apellido>/medicion,
 * recibe comandos on/off en .../cmd para controlar un LED, usa Last Will ("offline")
 * y "online" retenido en .../estado, y reconecta con backoff exponencial (1 s a 30 s).
 */

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ctype.h>
#include <esp_system.h>
#include <string.h>

// Identificador del alumno: reemplazar por tu apellido o legajo (sin espacios)
constexpr const char* ALUMNO_ID = "MAYOR";

// Credenciales Wi-Fi del simulador Wokwi
constexpr const char* WIFI_SSID = "Wokwi-GUEST";
constexpr const char* WIFI_PASSWORD = "";

// Broker MQTT y tópicos
constexpr const char* MQTT_BROKER = "broker.hivemq.com";
constexpr uint16_t MQTT_PORT = 1883;
constexpr const char* MQTT_TOPIC_PREFIJO = "unmdp/sd2026";
constexpr const char* MQTT_MSG_ONLINE = "online";
constexpr const char* MQTT_MSG_OFFLINE = "offline";
constexpr uint8_t MQTT_WILL_QOS = 0;
constexpr bool MQTT_WILL_RETAIN = true;
constexpr uint16_t MQTT_SOCKET_TIMEOUT_S = 5;

// Pines y comunicación serie
constexpr uint8_t PIN_SENSOR = 34;      // ADC1 en ESP32 (compatible con Wi-Fi)
constexpr uint8_t PIN_LED = 2;
constexpr unsigned long SERIAL_BAUD = 115200;

// Muestreo, filtrado y escalado del sensor
constexpr unsigned long INTERVALO_MUESTREO_MS = 100;
constexpr size_t CANTIDAD_MUESTRAS = 8;
constexpr int ADC_MAX = 4095;
constexpr float ESCALA_MAX_PCT = 100.0f;

// Intervalos de publicación y reconexión no bloqueante
constexpr unsigned long INTERVALO_PUBLICACION_MS = 5000;
constexpr unsigned long INTERVALO_REINTENTO_WIFI_MS = 5000;
constexpr unsigned long BACKOFF_INICIAL_MS = 1000;
constexpr unsigned long BACKOFF_MAX_MS = 30000;
constexpr unsigned long BACKOFF_FACTOR = 2;
constexpr unsigned long MS_POR_SEGUNDO = 1000;

// Tamaños de búfer fijos (sin usar String en el flujo principal)
constexpr size_t TAM_CLIENT_ID = 64;
constexpr size_t TAM_TOPICO = 96;
constexpr size_t TAM_PAYLOAD_JSON = 128;
constexpr size_t TAM_COMANDO = 32;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

char mqttClientId[TAM_CLIENT_ID];
char topicMedicion[TAM_TOPICO];
char topicCmd[TAM_TOPICO];
char topicEstado[TAM_TOPICO];

// Estado del filtro de promedio móvil
int muestras[CANTIDAD_MUESTRAS] = {0};
size_t indiceMuestra = 0;
size_t muestrasCargadas = 0;
long sumaMuestras = 0;
float ultimoValorPct = 0.0f;

// Estado de temporización y conexiones
unsigned long ultimoMuestreoMs = 0;
unsigned long ultimaPublicacionMs = 0;
unsigned long ultimoReintentoWifiMs = 0;
unsigned long ultimoIntentoMqttMs = 0;
unsigned long esperaActualMqttMs = 0;
unsigned long backoffActualMs = BACKOFF_INICIAL_MS;
unsigned long seqMedicion = 0;

bool wifiEstabaConectado = false;
bool mqttEstabaConectado = false;
bool primeraPublicacion = true;

float actualizarPromedioMovil(int nuevaMuestra) {
  if (muestrasCargadas < CANTIDAD_MUESTRAS) {
    muestrasCargadas++;
  } else {
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

void construirIdentidadYTopicos() {
  const uint16_t sufijoAleatorio = static_cast<uint16_t>(esp_random() & 0xFFFF);
  snprintf(mqttClientId, sizeof(mqttClientId), "esp32-sd2026-%s-%04X", ALUMNO_ID, sufijoAleatorio);
  snprintf(topicMedicion, sizeof(topicMedicion), "%s/%s/medicion", MQTT_TOPIC_PREFIJO, ALUMNO_ID);
  snprintf(topicCmd, sizeof(topicCmd), "%s/%s/cmd", MQTT_TOPIC_PREFIJO, ALUMNO_ID);
  snprintf(topicEstado, sizeof(topicEstado), "%s/%s/estado", MQTT_TOPIC_PREFIJO, ALUMNO_ID);
}

void procesarComandoLed(const char* comando) {
  if (strcasecmp(comando, "on") == 0) {
    digitalWrite(PIN_LED, HIGH);
    Serial.println("CMD -> LED encendido");
  } else if (strcasecmp(comando, "off") == 0) {
    digitalWrite(PIN_LED, LOW);
    Serial.println("CMD -> LED apagado");
  } else {
    Serial.print("CMD no reconocido: ");
    Serial.println(comando);
  }
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, topicCmd) != 0) {
    return;
  }

  if (length >= TAM_COMANDO) {
    Serial.println("CMD ignorado: mensaje demasiado largo");
    return;
  }

  char buffer[TAM_COMANDO];
  memcpy(buffer, payload, length);
  buffer[length] = '\0';

  // Recortar espacios y saltos de línea al principio y al final
  char* inicio = buffer;
  while (*inicio != '\0' && isspace(static_cast<unsigned char>(*inicio))) {
    inicio++;
  }
  size_t len = strlen(inicio);
  while (len > 0 && isspace(static_cast<unsigned char>(inicio[len - 1]))) {
    inicio[len - 1] = '\0';
    len--;
  }

  procesarComandoLed(inicio);
}

void iniciarConexionWifi(unsigned long ahoraMs) {
  if (strlen(WIFI_PASSWORD) == 0) {
    WiFi.begin(WIFI_SSID);
  } else {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
  ultimoReintentoWifiMs = ahoraMs;
}

bool gestionarWifi(unsigned long ahoraMs) {
  const bool conectado = (WiFi.status() == WL_CONNECTED);

  if (conectado && !wifiEstabaConectado) {
    wifiEstabaConectado = true;
    Serial.print("Wi-Fi conectado. IP: ");
    Serial.println(WiFi.localIP());
    // Permite intentar la primera conexión MQTT de inmediato
    esperaActualMqttMs = 0;
    backoffActualMs = BACKOFF_INICIAL_MS;
  } else if (!conectado) {
    if (wifiEstabaConectado) {
      wifiEstabaConectado = false;
      Serial.println("Wi-Fi desconectado. Reintentando...");
    }
    if (ahoraMs - ultimoReintentoWifiMs >= INTERVALO_REINTENTO_WIFI_MS) {
      WiFi.disconnect();
      iniciarConexionWifi(ahoraMs);
    }
  }

  return conectado;
}

void gestionarMqtt(unsigned long ahoraMs, bool wifiConectado) {
  if (!wifiConectado) {
    mqttEstabaConectado = false;
    return;
  }

  if (mqttClient.connected()) {
    mqttEstabaConectado = true;
    mqttClient.loop();
    return;
  }

  if (mqttEstabaConectado) {
    mqttEstabaConectado = false;
    Serial.println("Conexion MQTT perdida. Iniciando reconexion con backoff...");
    ultimoIntentoMqttMs = ahoraMs;
    esperaActualMqttMs = BACKOFF_INICIAL_MS;
    backoffActualMs = BACKOFF_INICIAL_MS * BACKOFF_FACTOR;
    return;
  }

  if (ahoraMs - ultimoIntentoMqttMs < esperaActualMqttMs) {
    return;
  }

  ultimoIntentoMqttMs = ahoraMs;
  Serial.print("Conectando a MQTT (");
  Serial.print(mqttClientId);
  Serial.println(")...");

  const bool conectado = mqttClient.connect(
      mqttClientId,
      topicEstado,
      MQTT_WILL_QOS,
      MQTT_WILL_RETAIN,
      MQTT_MSG_OFFLINE);

  if (conectado) {
    mqttEstabaConectado = true;
    backoffActualMs = BACKOFF_INICIAL_MS;
    esperaActualMqttMs = 0;
    primeraPublicacion = true;

    mqttClient.publish(topicEstado, MQTT_MSG_ONLINE, true);
    mqttClient.subscribe(topicCmd);

    Serial.print("MQTT conectado. Estado retenido -> ");
    Serial.println(topicEstado);
    Serial.print("Suscripto a -> ");
    Serial.println(topicCmd);
  } else {
    esperaActualMqttMs = backoffActualMs;
    const unsigned long siguiente = backoffActualMs * BACKOFF_FACTOR;
    backoffActualMs = (siguiente > BACKOFF_MAX_MS) ? BACKOFF_MAX_MS : siguiente;

    Serial.print("Fallo MQTT (rc=");
    Serial.print(mqttClient.state());
    Serial.print("). Reintento en ");
    Serial.print(esperaActualMqttMs / MS_POR_SEGUNDO);
    Serial.println(" s");
  }
}

void publicarMedicion(unsigned long ahoraMs) {
  if (!mqttClient.connected()) {
    return;
  }

  if (!primeraPublicacion && (ahoraMs - ultimaPublicacionMs < INTERVALO_PUBLICACION_MS)) {
    return;
  }
  primeraPublicacion = false;
  ultimaPublicacionMs = ahoraMs;

  seqMedicion++;
  const unsigned long tsSegundos = ahoraMs / MS_POR_SEGUNDO;

  char payload[TAM_PAYLOAD_JSON];
  snprintf(
      payload,
      sizeof(payload),
      "{\"seq\":%lu,\"ts\":%lu,\"valor\":%.1f}",
      seqMedicion,
      tsSegundos,
      ultimoValorPct);

  if (mqttClient.publish(topicMedicion, payload)) {
    Serial.print("PUB ");
    Serial.print(topicMedicion);
    Serial.print(" -> ");
    Serial.println(payload);
  } else {
    Serial.println("Error al publicar medicion MQTT");
  }
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  pinMode(PIN_SENSOR, INPUT);

  Serial.begin(SERIAL_BAUD);
  Serial.println();
  Serial.println("Inicio - Ejercicio 5 (Camino A): ESP32 con MQTT (ej5-exterior)");

  construirIdentidadYTopicos();
  Serial.print("Topico medicion: ");
  Serial.println(topicMedicion);
  Serial.print("Topico cmd:      ");
  Serial.println(topicCmd);
  Serial.print("Topico estado:   ");
  Serial.println(topicEstado);

  WiFi.mode(WIFI_STA);
  iniciarConexionWifi(millis());

  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(onMqttMessage);
  mqttClient.setSocketTimeout(MQTT_SOCKET_TIMEOUT_S);

  const unsigned long inicioMs = millis();
  ultimoMuestreoMs = inicioMs;
  ultimaPublicacionMs = inicioMs;
  ultimoValorPct = convertirAPorcentaje(actualizarPromedioMovil(analogRead(PIN_SENSOR)));
}

void loop() {
  const unsigned long ahoraMs = millis();

  if (ahoraMs - ultimoMuestreoMs >= INTERVALO_MUESTREO_MS) {
    ultimoMuestreoMs = ahoraMs;
    const int lecturaCruda = analogRead(PIN_SENSOR);
    const float promedioAdc = actualizarPromedioMovil(lecturaCruda);
    ultimoValorPct = convertirAPorcentaje(promedioAdc);
  }

  const bool wifiConectado = gestionarWifi(ahoraMs);
  gestionarMqtt(ahoraMs, wifiConectado);
  publicarMedicion(ahoraMs);
}
