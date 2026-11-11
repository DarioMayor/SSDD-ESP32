/*
 * Ejercicio integrador - Mini estación de monitoreo (ej-integrador)
 * Combina:
 * - Lectura de sensor cada 100 ms con promedio móvil (8 muestras) y alarma con histéresis (Ej. 3)
 * - Pulsador con antirrebote (40 ms) y flanco para alternar intervalo: 5 s, 15 s, 60 s (Ej. 2)
 * - Pantalla OLED I2C SSD1306 con refresco parcial sin borrar todo en cada ciclo (Ej. 4)
 * - Telemetría JSON y comandos validados por MQTT sobre Wi-Fi con Last Will y backoff (Ej. 5)
 * - Watchdog hardware (esp_task_wdt, 8 s) alimentado desde loop()
 * - Estado seguro: > 30 s sin lectura válida del sensor -> alarma activa y pantalla "SENSOR"
 */

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ctype.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// Identificador del alumno y red Wi-Fi / MQTT
constexpr const char* ALUMNO_ID = "MAYOR";
constexpr const char* WIFI_SSID = "Wokwi-GUEST";
constexpr const char* WIFI_PASSWORD = "";

constexpr const char* MQTT_BROKER = "broker.hivemq.com";
constexpr uint16_t MQTT_PORT = 1883;
constexpr const char* MQTT_TOPIC_PREFIJO = "unmdp/sd2026";
constexpr const char* MQTT_MSG_ONLINE = "online";
constexpr const char* MQTT_MSG_OFFLINE = "offline";
constexpr uint8_t MQTT_WILL_QOS = 0;
constexpr bool MQTT_WILL_RETAIN = true;
constexpr uint16_t MQTT_SOCKET_TIMEOUT_S = 5;

// Pines (ESP32 DevKit)
constexpr uint8_t PIN_SENSOR = 34;       // ADC1 compatible con Wi-Fi
constexpr uint8_t PIN_LED_ALARMA = 2;
constexpr uint8_t PIN_BOTON = 4;
constexpr uint8_t PIN_BUZZER = 18;
constexpr uint8_t PIN_I2C_SDA = 21;
constexpr uint8_t PIN_I2C_SCL = 22;
constexpr unsigned long SERIAL_BAUD = 115200;

// Muestreo, promedio móvil y escalado
constexpr unsigned long INTERVALO_MUESTREO_MS = 100;
constexpr size_t CANTIDAD_MUESTRAS = 8;
constexpr int ADC_MAX = 4095;
constexpr float ESCALA_MAX_PCT = 100.0f;

// Umbrales de alarma (histéresis) y refresco de pantalla
constexpr float UMBRAL_ALARMA_ON_PCT = 70.0f;
constexpr float UMBRAL_ALARMA_OFF_PCT = 60.0f;
constexpr float UMBRAL_REFRESCO_PCT = 0.5f;
constexpr uint8_t DECIMALES_IMPRESION = 1;

// Pulsador y buzzer
constexpr unsigned long DEBOUNCE_MS = 40;
constexpr unsigned long DURACION_BEEP_BOTON_MS = 60;
constexpr unsigned long DURACION_BEEP_ALARMA_MS = 150;
constexpr unsigned long PERIODO_BEEP_ALARMA_MS = 2000;
constexpr unsigned int FREQ_BEEP_BOTON_HZ = 1100;
constexpr unsigned int FREQ_BEEP_ALARMA_HZ = 1800;

// Intervalos de reporte (botón y rango válido por comando)
constexpr unsigned long INTERVALOS_BOTON_S[] = {5, 15, 60};
constexpr size_t CANTIDAD_INTERVALOS_BOTON = sizeof(INTERVALOS_BOTON_S) / sizeof(INTERVALOS_BOTON_S[0]);
constexpr unsigned long INTERVALO_MIN_S = 1;
constexpr unsigned long INTERVALO_MAX_S = 60;
constexpr unsigned long MS_POR_SEGUNDO = 1000;

// Reconexión no bloqueante (Wi-Fi y MQTT con backoff)
constexpr unsigned long INTERVALO_REINTENTO_WIFI_MS = 5000;
constexpr unsigned long BACKOFF_INICIAL_MS = 1000;
constexpr unsigned long BACKOFF_MAX_MS = 30000;
constexpr unsigned long BACKOFF_FACTOR = 2;

// Watchdog (8 s) y estado seguro del sensor (30 s)
constexpr uint32_t WDT_TIMEOUT_MS = 8000;
constexpr unsigned long TIMEOUT_SENSOR_VALIDO_MS = 30000;

// Pantalla OLED SSD1306
constexpr uint8_t OLED_ANCHO = 128;
constexpr uint8_t OLED_ALTO = 64;
constexpr int8_t OLED_PIN_RESET = -1;
constexpr uint8_t OLED_I2C_ADDR_DEFECTO = 0x3C;
constexpr uint8_t I2C_DIR_MIN = 0x01;
constexpr uint8_t I2C_DIR_MAX = 0x7E;
constexpr uint8_t I2C_UMBRAL_DOS_DIGITOS_HEX = 0x10;

// Coordenadas del layout OLED
constexpr uint8_t TEXTO_TAM_CHICO = 1;
constexpr uint8_t TEXTO_TAM_GRANDE = 2;
constexpr int16_t POS_Y_TITULO = 0;
constexpr int16_t POS_Y_LINEA = 10;
constexpr int16_t POS_Y_VALOR = 15;
constexpr int16_t ALTO_VALOR = 16;
constexpr int16_t POS_Y_ETIQUETA_EST = 38;
constexpr int16_t POS_X_ESTADO = 30;
constexpr int16_t POS_Y_ESTADO = 34;
constexpr int16_t ALTO_ESTADO = 16;
constexpr int16_t POS_Y_PIE = 54;
constexpr int16_t POS_X_INT_VAL = 26;
constexpr int16_t ANCHO_INT_VAL = 36;
constexpr int16_t POS_X_MQTT = 68;
constexpr int16_t ANCHO_MQTT = 60;
constexpr int16_t ALTO_PIE = 8;

// Búferes fijos de texto (sin String)
constexpr size_t TAM_CLIENT_ID = 64;
constexpr size_t TAM_TOPICO = 96;
constexpr size_t TAM_PAYLOAD_JSON = 192;
constexpr size_t TAM_COMANDO = 48;

enum class EstadoSistema {
  OK,
  ALERTA,
  SENSOR_FALLA
};

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
Adafruit_SSD1306 display(OLED_ANCHO, OLED_ALTO, &Wire, OLED_PIN_RESET);

char mqttClientId[TAM_CLIENT_ID];
char topicMedicion[TAM_TOPICO];
char topicCmd[TAM_TOPICO];
char topicEstado[TAM_TOPICO];
char serialCmdBuf[TAM_COMANDO];
size_t serialCmdLen = 0;

// Filtro de promedio móvil
int muestras[CANTIDAD_MUESTRAS] = {0};
size_t indiceMuestra = 0;
size_t muestrasCargadas = 0;
long sumaMuestras = 0;
float valorActualPct = 0.0f;

// Estado general de la estación
EstadoSistema estadoActual = EstadoSistema::OK;
unsigned long intervaloReporteS = INTERVALOS_BOTON_S[0];
unsigned long seqReporte = 0;
bool fallaSensorSimulada = false;
unsigned long ultimaLecturaValidaMs = 0;

// Antirrebote del botón
int lecturaAnteriorBoton = HIGH;
int estadoEstableBoton = HIGH;
unsigned long ultimoCambioBotonMs = 0;

// Temporizadores no bloqueantes
unsigned long ultimoMuestreoMs = 0;
unsigned long ultimoReporteMs = 0;
unsigned long ultimoReintentoWifiMs = 0;
unsigned long ultimoIntentoMqttMs = 0;
unsigned long esperaActualMqttMs = 0;
unsigned long backoffActualMs = BACKOFF_INICIAL_MS;

// Buzzer no bloqueante
bool buzzerSonando = false;
unsigned long inicioBuzzerMs = 0;
unsigned long duracionBuzzerMs = 0;
unsigned long ultimoBeepAlarmaMs = 0;

// Estado de red y pantalla
bool wifiEstabaConectado = false;
bool mqttEstabaConectado = false;
bool primeraPublicacion = true;
bool pantallaInicializada = false;
bool pantallaConDatos = false;
float ultimoValorPantallaPct = 0.0f;
EstadoSistema ultimoEstadoPantalla = EstadoSistema::OK;
unsigned long ultimoIntervaloPantallaS = 0;
bool ultimoMqttPantalla = false;

const char* textoEstado(EstadoSistema estado) {
  switch (estado) {
    case EstadoSistema::OK:
      return "OK";
    case EstadoSistema::ALERTA:
      return "ALERTA";
    case EstadoSistema::SENSOR_FALLA:
      return "SENSOR";
  }
  return "OK";
}

bool esAlarmaActiva(EstadoSistema estado) {
  return estado != EstadoSistema::OK;
}

void construirIdentidadYTopicos() {
  const uint16_t sufijo = static_cast<uint16_t>(esp_random() & 0xFFFF);
  snprintf(mqttClientId, sizeof(mqttClientId), "esp32-sd2026-%s-%04X", ALUMNO_ID, sufijo);
  snprintf(topicMedicion, sizeof(topicMedicion), "%s/%s/medicion", MQTT_TOPIC_PREFIJO, ALUMNO_ID);
  snprintf(topicCmd, sizeof(topicCmd), "%s/%s/cmd", MQTT_TOPIC_PREFIJO, ALUMNO_ID);
  snprintf(topicEstado, sizeof(topicEstado), "%s/%s/estado", MQTT_TOPIC_PREFIJO, ALUMNO_ID);
}

void inicializarWatchdog() {
  const esp_task_wdt_config_t wdtConfig = {
      .timeout_ms = WDT_TIMEOUT_MS,
      .idle_core_mask = 0,
      .trigger_panic = true};

  // En ESP-IDF v5 / Arduino 3.x el TWDT puede venir ya iniciado antes de setup()
  if (esp_task_wdt_init(&wdtConfig) == ESP_ERR_INVALID_STATE) {
    esp_task_wdt_reconfigure(&wdtConfig);
  }
  if (esp_task_wdt_status(NULL) != ESP_OK) {
    esp_task_wdt_add(NULL);
  }
  Serial.print("Watchdog activo (plazo ");
  Serial.print(WDT_TIMEOUT_MS / MS_POR_SEGUNDO);
  Serial.println(" s)");
}

uint8_t detectarOledI2C() {
  uint8_t direccion = 0;
  for (uint8_t dir = I2C_DIR_MIN; dir <= I2C_DIR_MAX; dir++) {
    Wire.beginTransmission(dir);
    if (Wire.endTransmission() == 0) {
      if (direccion == 0) {
        direccion = dir;
      }
      Serial.print("I2C detectado: 0x");
      if (dir < I2C_UMBRAL_DOS_DIGITOS_HEX) {
        Serial.print('0');
      }
      Serial.println(dir, HEX);
    }
  }
  return (direccion != 0) ? direccion : OLED_I2C_ADDR_DEFECTO;
}

bool inicializarPantalla(uint8_t direccionI2C) {
  if (!display.begin(SSD1306_SWITCHCAPVCC, direccionI2C, true, false)) {
    Serial.println("Error al inicializar OLED SSD1306");
    return false;
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(TEXTO_TAM_CHICO);

  display.setCursor(0, POS_Y_TITULO);
  display.print("ESTACION MONITOREO");
  display.drawFastHLine(0, POS_Y_LINEA, OLED_ANCHO, SSD1306_WHITE);

  display.setCursor(0, POS_Y_ETIQUETA_EST);
  display.print("Est:");

  display.setCursor(0, POS_Y_PIE);
  display.print("Int:");

  display.display();
  return true;
}

void refrescarPantalla(bool forzar) {
  if (!pantallaInicializada) {
    return;
  }

  const bool mqttConectado = mqttClient.connected();
  const bool cambioEstado = !pantallaConDatos || (estadoActual != ultimoEstadoPantalla);
  const bool cambioValor = !pantallaConDatos ||
                           (estadoActual != EstadoSistema::SENSOR_FALLA &&
                            fabsf(valorActualPct - ultimoValorPantallaPct) >= UMBRAL_REFRESCO_PCT);
  const bool cambioIntervalo = !pantallaConDatos || (intervaloReporteS != ultimoIntervaloPantallaS);
  const bool cambioMqtt = !pantallaConDatos || (mqttConectado != ultimoMqttPantalla);

  if (!forzar && !cambioEstado && !cambioValor && !cambioIntervalo && !cambioMqtt) {
    return;
  }

  if (forzar || cambioValor || cambioEstado) {
    display.fillRect(0, POS_Y_VALOR, OLED_ANCHO, ALTO_VALOR, SSD1306_BLACK);
    display.setTextSize(TEXTO_TAM_GRANDE);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, POS_Y_VALOR);
    if (estadoActual == EstadoSistema::SENSOR_FALLA) {
      display.print("---.- %");
    } else {
      display.print(valorActualPct, DECIMALES_IMPRESION);
      display.print(" %");
    }
    ultimoValorPantallaPct = valorActualPct;
  }

  if (forzar || cambioEstado) {
    display.fillRect(POS_X_ESTADO, POS_Y_ESTADO, OLED_ANCHO - POS_X_ESTADO, ALTO_ESTADO, SSD1306_BLACK);
    display.setTextSize(TEXTO_TAM_GRANDE);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(POS_X_ESTADO, POS_Y_ESTADO);
    display.print(textoEstado(estadoActual));
    ultimoEstadoPantalla = estadoActual;
  }

  if (forzar || cambioIntervalo) {
    display.fillRect(POS_X_INT_VAL, POS_Y_PIE, ANCHO_INT_VAL, ALTO_PIE, SSD1306_BLACK);
    display.setTextSize(TEXTO_TAM_CHICO);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(POS_X_INT_VAL, POS_Y_PIE);
    display.print(intervaloReporteS);
    display.print("s");
    ultimoIntervaloPantallaS = intervaloReporteS;
  }

  if (forzar || cambioMqtt) {
    display.fillRect(POS_X_MQTT, POS_Y_PIE, ANCHO_MQTT, ALTO_PIE, SSD1306_BLACK);
    display.setTextSize(TEXTO_TAM_CHICO);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(POS_X_MQTT, POS_Y_PIE);
    display.print(mqttConectado ? "MQTT:ON" : "MQTT:--");
    ultimoMqttPantalla = mqttConectado;
  }

  pantallaConDatos = true;
  display.display();
}

void iniciarBuzzer(unsigned int freqHz, unsigned long duracionMs, unsigned long ahoraMs) {
  tone(PIN_BUZZER, freqHz);
  inicioBuzzerMs = ahoraMs;
  duracionBuzzerMs = duracionMs;
  buzzerSonando = true;
}

void actualizarSalidasAlarma(unsigned long ahoraMs) {
  const bool alarma = esAlarmaActiva(estadoActual);
  digitalWrite(PIN_LED_ALARMA, alarma ? HIGH : LOW);

  if (buzzerSonando && (ahoraMs - inicioBuzzerMs >= duracionBuzzerMs)) {
    noTone(PIN_BUZZER);
    buzzerSonando = false;
  }

  if (alarma && !buzzerSonando && (ahoraMs - ultimoBeepAlarmaMs >= PERIODO_BEEP_ALARMA_MS)) {
    ultimoBeepAlarmaMs = ahoraMs;
    iniciarBuzzer(FREQ_BEEP_ALARMA_HZ, DURACION_BEEP_ALARMA_MS, ahoraMs);
  }
}

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

bool esLecturaValida(int lecturaCruda) {
  return !fallaSensorSimulada && (lecturaCruda >= 0) && (lecturaCruda <= ADC_MAX);
}

void actualizarSensorYEstado(unsigned long ahoraMs) {
  if (ahoraMs - ultimoMuestreoMs >= INTERVALO_MUESTREO_MS) {
    ultimoMuestreoMs = ahoraMs;

    const int lecturaCruda = analogRead(PIN_SENSOR);
    if (esLecturaValida(lecturaCruda)) {
      ultimaLecturaValidaMs = ahoraMs;
      valorActualPct = convertirAPorcentaje(actualizarPromedioMovil(lecturaCruda));

      if (estadoActual == EstadoSistema::SENSOR_FALLA) {
        Serial.println("Sensor recuperado: saliendo de estado seguro");
        estadoActual = (valorActualPct > UMBRAL_ALARMA_ON_PCT) ? EstadoSistema::ALERTA : EstadoSistema::OK;
      } else if (estadoActual == EstadoSistema::OK && (valorActualPct > UMBRAL_ALARMA_ON_PCT)) {
        estadoActual = EstadoSistema::ALERTA;
        Serial.print("Alarma ACTIVADA (");
        Serial.print(valorActualPct, DECIMALES_IMPRESION);
        Serial.println(" %)");
      } else if (estadoActual == EstadoSistema::ALERTA && (valorActualPct < UMBRAL_ALARMA_OFF_PCT)) {
        estadoActual = EstadoSistema::OK;
        Serial.print("Alarma DESACTIVADA (");
        Serial.print(valorActualPct, DECIMALES_IMPRESION);
        Serial.println(" %)");
      }
    }
  }

  // Estado seguro: > 30 s sin lectura válida mantiene alarma encendida y pantalla en SENSOR
  if ((estadoActual != EstadoSistema::SENSOR_FALLA) &&
      (ahoraMs - ultimaLecturaValidaMs > TIMEOUT_SENSOR_VALIDO_MS)) {
    estadoActual = EstadoSistema::SENSOR_FALLA;
    Serial.println("ESTADO SEGURO: >30 s sin lectura valida -> alarma activa, pantalla SENSOR");
  }
}

bool detectarPulsacion(unsigned long ahoraMs) {
  const int lecturaActual = digitalRead(PIN_BOTON);

  if (lecturaActual != lecturaAnteriorBoton) {
    ultimoCambioBotonMs = ahoraMs;
    lecturaAnteriorBoton = lecturaActual;
  }

  if (ahoraMs - ultimoCambioBotonMs >= DEBOUNCE_MS) {
    if (lecturaActual != estadoEstableBoton) {
      estadoEstableBoton = lecturaActual;
      if (estadoEstableBoton == LOW) {
        return true;
      }
    }
  }
  return false;
}

void alternarIntervaloPorBoton(unsigned long ahoraMs) {
  size_t siguienteIdx = 0;
  for (size_t i = 0; i < CANTIDAD_INTERVALOS_BOTON; i++) {
    if (intervaloReporteS == INTERVALOS_BOTON_S[i]) {
      siguienteIdx = (i + 1) % CANTIDAD_INTERVALOS_BOTON;
      break;
    }
  }

  intervaloReporteS = INTERVALOS_BOTON_S[siguienteIdx];
  Serial.print("Boton -> intervalo vigente: ");
  Serial.print(intervaloReporteS);
  Serial.println(" s");

  if (!esAlarmaActiva(estadoActual)) {
    iniciarBuzzer(FREQ_BEEP_BOTON_HZ, DURACION_BEEP_BOTON_MS, ahoraMs);
  }
  refrescarPantalla(true);
}

void recortarEspacios(char* texto) {
  char* inicio = texto;
  while (*inicio != '\0' && isspace(static_cast<unsigned char>(*inicio))) {
    inicio++;
  }
  if (inicio != texto) {
    memmove(texto, inicio, strlen(inicio) + 1);
  }
  size_t len = strlen(texto);
  while (len > 0 && isspace(static_cast<unsigned char>(texto[len - 1]))) {
    texto[len - 1] = '\0';
    len--;
  }
}

void procesarComandoTexto(char* comando) {
  recortarEspacios(comando);
  if (comando[0] == '\0') {
    return;
  }

  if (strncasecmp(comando, "INTERVALO ", 10) == 0) {
    const char* arg = comando + 10;
    while (*arg != '\0' && isspace(static_cast<unsigned char>(*arg))) {
      arg++;
    }
    if (*arg == '\0') {
      Serial.println("CMD ERROR: falta valor de intervalo (1 a 60)");
      return;
    }

    char* finNumero = nullptr;
    const long valor = strtol(arg, &finNumero, 10);
    if (finNumero == arg || *finNumero != '\0' ||
        valor < static_cast<long>(INTERVALO_MIN_S) ||
        valor > static_cast<long>(INTERVALO_MAX_S)) {
      Serial.println("CMD ERROR: intervalo invalido (debe ser entero de 1 a 60 s)");
      return;
    }

    intervaloReporteS = static_cast<unsigned long>(valor);
    Serial.print("CMD OK -> intervalo actualizado a ");
    Serial.print(intervaloReporteS);
    Serial.println(" s");
    refrescarPantalla(true);
    return;
  }

  if (strcasecmp(comando, "SENSOR OFF") == 0) {
    fallaSensorSimulada = true;
    Serial.println("CMD OK -> simulando falla de sensor (esperar 30 s para estado seguro)");
    return;
  }

  if (strcasecmp(comando, "SENSOR ON") == 0) {
    fallaSensorSimulada = false;
    Serial.println("CMD OK -> sensor restaurado");
    return;
  }

  if (strcasecmp(comando, "TEST WDT") == 0) {
    Serial.println("CMD OK -> entrando en bucle infinito para probar reinicio por Watchdog (8 s)...");
    Serial.flush();
    while (true) {
      // Bucle infinito intencional para validar el watchdog de 8 s
    }
  }

  Serial.print("CMD ERROR: comando no reconocido -> ");
  Serial.println(comando);
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, topicCmd) != 0) {
    return;
  }
  if (length >= TAM_COMANDO) {
    Serial.println("CMD ERROR: mensaje demasiado largo");
    return;
  }

  char buffer[TAM_COMANDO];
  memcpy(buffer, payload, length);
  buffer[length] = '\0';
  procesarComandoTexto(buffer);
}

void leerComandosSerial() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      serialCmdBuf[serialCmdLen] = '\0';
      procesarComandoTexto(serialCmdBuf);
      serialCmdLen = 0;
    } else if (serialCmdLen < TAM_COMANDO - 1) {
      serialCmdBuf[serialCmdLen++] = c;
    }
  }
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
    Serial.println("Conexion MQTT perdida. Reconectando con backoff...");
    ultimoIntentoMqttMs = ahoraMs;
    esperaActualMqttMs = BACKOFF_INICIAL_MS;
    backoffActualMs = BACKOFF_INICIAL_MS * BACKOFF_FACTOR;
    refrescarPantalla(false);
    return;
  }

  if (ahoraMs - ultimoIntentoMqttMs < esperaActualMqttMs) {
    return;
  }

  ultimoIntentoMqttMs = ahoraMs;
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
    Serial.print("MQTT conectado. Suscripto a: ");
    Serial.println(topicCmd);
    refrescarPantalla(false);
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

void publicarReporte(unsigned long ahoraMs) {
  if (!mqttClient.connected()) {
    return;
  }

  const unsigned long intervaloMs = intervaloReporteS * MS_POR_SEGUNDO;
  if (!primeraPublicacion && (ahoraMs - ultimoReporteMs < intervaloMs)) {
    return;
  }
  primeraPublicacion = false;
  ultimoReporteMs = ahoraMs;

  seqReporte++;
  const unsigned long tsSegundos = ahoraMs / MS_POR_SEGUNDO;

  char payload[TAM_PAYLOAD_JSON];
  snprintf(
      payload,
      sizeof(payload),
      "{\"seq\":%lu,\"ts\":%lu,\"valor\":%.1f,\"alarma\":\"%s\",\"intervalo\":%lu}",
      seqReporte,
      tsSegundos,
      valorActualPct,
      textoEstado(estadoActual),
      intervaloReporteS);

  if (mqttClient.publish(topicMedicion, payload)) {
    Serial.print("PUB -> ");
    Serial.println(payload);
  }
}

void setup() {
  pinMode(PIN_LED_ALARMA, OUTPUT);
  digitalWrite(PIN_LED_ALARMA, LOW);
  pinMode(PIN_BOTON, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_SENSOR, INPUT);

  lecturaAnteriorBoton = digitalRead(PIN_BOTON);
  estadoEstableBoton = lecturaAnteriorBoton;

  Serial.begin(SERIAL_BAUD);
  Serial.println();
  Serial.println("Inicio - Ejercicio integrador: Mini estacion de monitoreo (ej-integrador)");

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  const uint8_t direccionOled = detectarOledI2C();
  pantallaInicializada = inicializarPantalla(direccionOled);

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
  ultimoReporteMs = inicioMs;
  ultimoCambioBotonMs = inicioMs;
  ultimaLecturaValidaMs = inicioMs;

  const int lecturaInicial = analogRead(PIN_SENSOR);
  if (esLecturaValida(lecturaInicial)) {
    valorActualPct = convertirAPorcentaje(actualizarPromedioMovil(lecturaInicial));
    estadoActual = (valorActualPct > UMBRAL_ALARMA_ON_PCT) ? EstadoSistema::ALERTA : EstadoSistema::OK;
  }

  refrescarPantalla(true);
  inicializarWatchdog();
}

void loop() {
  // Alimentar el watchdog en cada vuelta del bucle principal
  esp_task_wdt_reset();

  // Para probar el watchdog por código, descomentá: while (true) {}
  // (o enviá "TEST WDT" por Serial o por MQTT en .../cmd)

  const unsigned long ahoraMs = millis();

  leerComandosSerial();
  actualizarSensorYEstado(ahoraMs);

  if (detectarPulsacion(ahoraMs)) {
    alternarIntervaloPorBoton(ahoraMs);
  }

  actualizarSalidasAlarma(ahoraMs);
  refrescarPantalla(false);

  const bool wifiConectado = gestionarWifi(ahoraMs);
  gestionarMqtt(ahoraMs, wifiConectado);
  publicarReporte(ahoraMs);
}
