/*
 * Ejercicio 4 - Pantalla I2C (ej4-pantalla)
 * Escanea el bus I2C al inicio para detectar la dirección de la pantalla (0x3C),
 * lee el potenciómetro cada 100 ms con promedio móvil de 8 muestras (ej. 3),
 * evalúa alarma con histéresis (> 70 % ALERTA, < 60 % OK) y actualiza solo los
 * campos dinámicos del OLED SSD1306 cuando cambia más de un umbral mínimo.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// Pines y comunicación serie
constexpr uint8_t PIN_SENSOR = 34;       // ADC1 en ESP32 (0 a 4095)
constexpr uint8_t PIN_I2C_SDA = 21;      // Bus I2C en ESP32: SDA = GPIO 21
constexpr uint8_t PIN_I2C_SCL = 22;      // Bus I2C en ESP32: SCL = GPIO 22
constexpr unsigned long SERIAL_BAUD = 115200;

// Parámetros del escáner I2C y pantalla OLED SSD1306
constexpr uint8_t I2C_DIR_MIN = 0x01;
constexpr uint8_t I2C_DIR_MAX = 0x7E;
constexpr uint8_t I2C_UMBRAL_DOS_DIGITOS_HEX = 0x10;
constexpr uint8_t OLED_ANCHO = 128;
constexpr uint8_t OLED_ALTO = 64;
constexpr int8_t OLED_PIN_RESET = -1;
constexpr uint8_t OLED_I2C_ADDR_DEFECTO = 0x3C;

// Coordenadas y tamaños de texto en el OLED (para refresco parcial)
constexpr uint8_t TEXTO_TAM_TITULO = 1;
constexpr uint8_t TEXTO_TAM_VALOR = 2;
constexpr uint8_t TEXTO_TAM_ESTADO = 2;
constexpr int16_t POS_Y_TITULO = 0;
constexpr int16_t POS_Y_LINEA = 11;
constexpr int16_t POS_Y_ETIQUETA_VALOR = 16;
constexpr int16_t POS_Y_VALOR = 26;
constexpr int16_t ALTO_CAMPO_VALOR = 16;
constexpr int16_t POS_Y_ETIQUETA_ESTADO = 52;
constexpr int16_t POS_X_ESTADO = 48;
constexpr int16_t POS_Y_ESTADO = 48;
constexpr int16_t ALTO_CAMPO_ESTADO = 16;

// Muestreo, filtrado y escalado del sensor
constexpr unsigned long INTERVALO_MUESTREO_MS = 100;
constexpr size_t CANTIDAD_MUESTRAS = 8;
constexpr int ADC_MAX = 4095;
constexpr float ESCALA_MAX_PCT = 100.0f;

// Umbrales de alarma (histéresis) y refresco de pantalla
constexpr float UMBRAL_ALARMA_ON_PCT = 70.0f;
constexpr float UMBRAL_ALARMA_OFF_PCT = 60.0f;
constexpr float UMBRAL_REFRESCO_PCT = 0.5f;
constexpr uint8_t DECIMALES_IMPRESION = 1;

Adafruit_SSD1306 display(OLED_ANCHO, OLED_ALTO, &Wire, OLED_PIN_RESET);

// Estado del filtro de promedio móvil
int muestras[CANTIDAD_MUESTRAS] = {0};
size_t indiceMuestra = 0;
size_t muestrasCargadas = 0;
long sumaMuestras = 0;

// Estado de muestreo, alarma y pantalla
unsigned long ultimoMuestreoMs = 0;
bool alarmaActiva = false;
bool pantallaInicializada = false;
bool pantallaConDatos = false;
float ultimoMostradoPct = 0.0f;
bool ultimaAlarmaMostrada = false;

uint8_t escanearBusI2C() {
  uint8_t direccionDetectada = 0;
  uint8_t dispositivos = 0;

  Serial.println("Escaneando bus I2C...");
  for (uint8_t dir = I2C_DIR_MIN; dir <= I2C_DIR_MAX; dir++) {
    Wire.beginTransmission(dir);
    const uint8_t error = Wire.endTransmission();

    if (error == 0) {
      dispositivos++;
      if (direccionDetectada == 0) {
        direccionDetectada = dir;
      }
      Serial.print("Dispositivo I2C encontrado en: 0x");
      if (dir < I2C_UMBRAL_DOS_DIGITOS_HEX) {
        Serial.print('0');
      }
      Serial.println(dir, HEX);
    }
  }

  if (dispositivos == 0) {
    Serial.println("No se encontraron dispositivos en el bus I2C.");
  }

  return direccionDetectada;
}

bool inicializarPantalla(uint8_t direccionI2C) {
  if (!display.begin(SSD1306_SWITCHCAPVCC, direccionI2C, true, false)) {
    Serial.println("Error al inicializar la pantalla OLED SSD1306.");
    return false;
  }

  // Se limpia la pantalla completa una sola vez para dibujar el marco fijo
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(TEXTO_TAM_TITULO);

  display.setCursor(0, POS_Y_TITULO);
  display.print("EJ4 - PANTALLA I2C");
  display.drawFastHLine(0, POS_Y_LINEA, OLED_ANCHO, SSD1306_WHITE);

  display.setCursor(0, POS_Y_ETIQUETA_VALOR);
  display.print("Valor:");

  display.setCursor(0, POS_Y_ETIQUETA_ESTADO);
  display.print("Estado:");

  display.display();
  return true;
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

void actualizarAlarma(float porcentaje) {
  const bool estadoAnterior = alarmaActiva;

  if (!alarmaActiva && (porcentaje > UMBRAL_ALARMA_ON_PCT)) {
    alarmaActiva = true;
  } else if (alarmaActiva && (porcentaje < UMBRAL_ALARMA_OFF_PCT)) {
    alarmaActiva = false;
  }

  if (alarmaActiva != estadoAnterior) {
    Serial.print("Estado: ");
    Serial.print(alarmaActiva ? "ALERTA" : "OK");
    Serial.print(" (");
    Serial.print(porcentaje, DECIMALES_IMPRESION);
    Serial.println(" %)");
  }
}

void refrescarPantalla(float porcentaje, bool alarma) {
  if (!pantallaInicializada) {
    return;
  }

  const bool cambioValor = !pantallaConDatos || (fabsf(porcentaje - ultimoMostradoPct) >= UMBRAL_REFRESCO_PCT);
  const bool cambioAlarma = !pantallaConDatos || (alarma != ultimaAlarmaMostrada);

  if (!cambioValor && !cambioAlarma) {
    return;
  }

  // Solo borra y redibuja los campos dinámicos, preservando títulos y etiquetas
  display.fillRect(0, POS_Y_VALOR, OLED_ANCHO, ALTO_CAMPO_VALOR, SSD1306_BLACK);
  display.setTextSize(TEXTO_TAM_VALOR);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, POS_Y_VALOR);
  display.print(porcentaje, DECIMALES_IMPRESION);
  display.print(" %");
  ultimoMostradoPct = porcentaje;

  if (cambioAlarma) {
    display.fillRect(POS_X_ESTADO, POS_Y_ESTADO, OLED_ANCHO - POS_X_ESTADO, ALTO_CAMPO_ESTADO, SSD1306_BLACK);
    display.setTextSize(TEXTO_TAM_ESTADO);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(POS_X_ESTADO, POS_Y_ESTADO);
    display.print(alarma ? "ALERTA" : "OK");
    ultimaAlarmaMostrada = alarma;
  }

  pantallaConDatos = true;
  display.display();
}

void setup() {
  pinMode(PIN_SENSOR, INPUT);

  Serial.begin(SERIAL_BAUD);
  Serial.println();
  Serial.println("Inicio - Ejercicio 4: Pantalla I2C (ej4-pantalla)");

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  const uint8_t direccionDetectada = escanearBusI2C();
  const uint8_t direccionOled = (direccionDetectada != 0) ? direccionDetectada : OLED_I2C_ADDR_DEFECTO;

  pantallaInicializada = inicializarPantalla(direccionOled);
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
    refrescarPantalla(porcentaje, alarmaActiva);
  }
}
