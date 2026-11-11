# SSDD-ESP32 — Práctica con Wokwi y PlatformIO

Colección de ejercicios progresivos sobre **ESP32 (`esp32dev`)** usando **PlatformIO (framework Arduino)** y simulación en **Wokwi**, desarrollados para la práctica de Sistemas Digitales (`unmdp/sd2026`).

---

## Estructura del repositorio

| Carpeta | Ejercicio | Descripción |
| :--- | :--- | :--- |
| `ej1-parpadeo/` | **1. Latido no bloqueante** | Parpadeo de LED (`500 ms` ON / `500 ms` OFF) y reporte de `uptime` por Serial cada `2 s` en paralelo usando `millis()`. |
| `ej2-boton/` | **2. Botón y máquina de estados** | Pulsador con `INPUT_PULLUP`, antirrebote por software (`40 ms`) y detección de flanco para rotar entre `APAGADO`, `FIJO` y `PARPADEO` (`enum class` + `switch`) con confirmación en buzzer. |
| `ej3-sensor/` | **3. Sensor analógico con umbral e histéresis** | Lectura de potenciómetro en ADC1 cada `100 ms`, promedio móvil de `8` muestras, escalado a `0.0 - 100.0 %` y alarma con histéresis (`> 70 %` activa, `< 60 %` desactiva), imprimiendo solo ante cambios de estado. |
| `ej4-pantalla/` | **4. Pantalla I2C** | Escáner I2C al arranque (`0x3C`) y visualización en OLED `SSD1306` de valor y estado (`OK` / `ALERTA`) con refresco parcial (`>= 0.5 %` o cambio de alarma), sin borrar la pantalla completa en cada ciclo. |
| `ej5-exterior/` | **5. Comunicación con el exterior (Camino A)** | Conexión no bloqueante a `Wokwi-GUEST` y MQTT (`broker.hivemq.com:1883`), publicación JSON cada `5 s`, suscripción a comandos `on`/`off`, Last Will `offline` + `online` retenido y reconexión con backoff exponencial (`1 s` a `30 s`). |
| `ej-integrador/` | **Integrador. Mini estación de monitoreo** | Combina medición cada `100 ms`, promedio móvil e histéresis, pulsador (`5 s` / `15 s` / `60 s`), OLED parcial, MQTT JSON, validación de comandos `INTERVALO <1..60>`, watchdog hardware (`esp_task_wdt`, `8 s`) y estado seguro (`> 30 s` sin lectura válida → alarma activa + `SENSOR`). |

---

## Wokwi ya viene configurado

Cada carpeta de ejercicio es un proyecto PlatformIO independiente y **ya incluye toda la configuración necesaria para simularlo en Wokwi sin cablear ni configurar desde cero**:

- **`diagram.json`**: define la placa `board-esp32-devkit-c-v4`, los componentes virtuales (LEDs, resistencias de `220 Ω`, pulsador, buzzer, potenciómetro, pantalla OLED `SSD1306`), todas las conexiones eléctricas y los pines `TX`/`RX` hacia `$serialMonitor`.
- **`wokwi.toml`**: vincula la simulación con los artefactos compilados por PlatformIO (`.pio/build/esp32dev/firmware.bin` y `.pio/build/esp32dev/firmware.elf`).
- **`platformio.ini`**: define el entorno `[env:esp32dev]`, velocidad del monitor serie (`monitor_speed = 115200`) y dependencias automáticas (`lib_deps`).
- **`libraries.txt`** *(en `ej4-pantalla`, `ej5-exterior` y `ej-integrador`)*: lista las bibliotecas externas requeridas si copiás el proyecto a **Wokwi Web**.

---

## Cómo usarlo

### Opción A: En el editor (PlatformIO + extensión Wokwi Simulator)

1. Cloná el repositorio:
   ```bash
   git clone https://github.com/DarioMayor/SSDD-ESP32.git
   ```
2. Abrí la carpeta del ejercicio que quieras probar (por ejemplo `ej-integrador/`).
3. Compilá el firmware con PlatformIO (`pio run` o el botón **Build** ✔ en la barra inferior) para generar `.pio/build/esp32dev/firmware.bin` y `firmware.elf`.
4. Con la extensión **Wokwi Simulator** (`wokwi.wokwi-vscode`) instalada, abrí `diagram.json` y presioná **Play** (*Start Simulation*).
   - Si es la primera vez con la extensión, activá la licencia gratuita desde la paleta de comandos (`F1` → `Wokwi: Request a New License`).

### Opción B: En el navegador (`wokwi.com`, sin instalar nada)

1. Creá un proyecto nuevo de **ESP32** en [Wokwi](https://wokwi.com/projects/new/esp32).
2. Pegá el contenido de `src/main.cpp` del ejercicio en la pestaña `sketch.ino`.
3. Pegá el contenido de `diagram.json` del ejercicio en la pestaña `diagram.json`.
4. Si el ejercicio usa bibliotecas externas (`ej4-pantalla`, `ej5-exterior` o `ej-integrador`), agregá en la pestaña **Library Manager** las indicadas en su `libraries.txt`:
   - `ej4-pantalla`: `Adafruit SSD1306`, `Adafruit GFX Library`
   - `ej5-exterior`: `PubSubClient`
   - `ej-integrador`: `Adafruit SSD1306`, `Adafruit GFX Library`, `PubSubClient`
5. Presioná **Start the simulation** (`▶`).

---

## Cómo probar MQTT (`ej5-exterior` y `ej-integrador`)

Ambos proyectos están configurados con `ALUMNO_ID = "MAYOR"` (modificable en una sola constante al inicio de `src/main.cpp`), red Wi-Fi del simulador `Wokwi-GUEST` (sin contraseña) y broker público `broker.hivemq.com` (puerto `1883` en ESP32).

- **Tópico de telemetría:** `unmdp/sd2026/MAYOR/medicion`
- **Tópico de comandos:** `unmdp/sd2026/MAYOR/cmd`
- **Tópico de presencia (LWT):** `unmdp/sd2026/MAYOR/estado` (`online` retenido al conectar; `offline` por Last Will si cae la conexión)

Para observar los mensajes y enviar comandos desde un cliente externo:
1. Abrí [MQTTX Web](https://mqttx.app/web-client/index.html) (o MQTTX Desktop).
2. Conectate a `wss://broker.hivemq.com:8884/mqtt` (o `mqtt://broker.hivemq.com:1883` en escritorio).
3. Suscribite a `unmdp/sd2026/MAYOR/#`.
4. Enviá comandos en `unmdp/sd2026/MAYOR/cmd`:
   - En `ej5-exterior`: `on` / `off` para encender o apagar el LED.
   - En `ej-integrador`:
     - `INTERVALO <segundos>` (entero válido entre `1` y `60`; valores fuera de rango o malformados se rechazan).
     - `SENSOR OFF` / `SENSOR ON` para simular falla del sensor y comprobar el estado seguro a los `30 s`.
     - `TEST WDT` (también por Serial Monitor) para forzar un bucle infinito y comprobar el reinicio por Watchdog a los `8 s`.

---

## Cómo está hecho (diseño y hardware)

### Criterios de diseño aplicados en toda la práctica
- **Ejecución 100 % no bloqueante:** toda la temporización (parpadeo, antirrebote, buzzer, muestreo ADC, reconexión Wi-Fi/MQTT y publicación) se resuelve con `millis()` mediante resta entre enteros sin signo (`ahoraMs - ultimaMarca >= intervalo`), sin usar `delay()`.
- **Constantes con nombre:** pines, umbrales, tiempos, direcciones I2C, coordenadas de pantalla y parámetros de red se definen al inicio de cada `src/main.cpp`.
- **Cuidado de memoria y estabilidad:** en MQTT y lectura de comandos se utilizan búferes `char[]` de tamaño fijo y `snprintf` en lugar de `String`.
- **Protección en el integrador:** utiliza `esp_task_wdt` (`timeout_ms = 8000`, compatible con ESP-IDF v5 / Arduino ESP32 3.x vía `esp_task_wdt_init` / `esp_task_wdt_reconfigure`) alimentado desde `loop()`, junto con monitoreo de frescura del sensor (`30 s`).

### Mapa de pines en `board-esp32-devkit-c-v4`

Los pines se eligieron de forma compatible en todos los ejercicios para evitar conflictos entre ADC, I2C, Wi-Fi y pines de boot:

| Componente | Pin ESP32 | Notas | Usado en |
| :--- | :--- | :--- | :--- |
| LED (+ resistencia `220 Ω` a `GND`) | `GPIO 2` | Salida digital / alarma | `ej1`, `ej2`, `ej3`, `ej5`, `ej-integrador` |
| Pulsador (a `GND`) | `GPIO 4` | `INPUT_PULLUP`, flanco `HIGH -> LOW` | `ej2`, `ej-integrador` |
| Buzzer piezoeléctrico (a `GND`) | `GPIO 18` | `tone()` / `noTone()` no bloqueante | `ej2`, `ej-integrador` |
| Bus I2C `SDA` (OLED `SSD1306`) | `GPIO 21` | Dirección `0x3C` | `ej4`, `ej-integrador` |
| Bus I2C `SCL` (OLED `SSD1306`) | `GPIO 22` | Reloj I2C hardware | `ej4`, `ej-integrador` |
| Potenciómetro (`3V3` y `GND`) | `GPIO 34` | `ADC1` (12 bits `0..4095`, compatible con Wi-Fi) | `ej3`, `ej4`, `ej5`, `ej-integrador` |
