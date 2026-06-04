/**
 * @file SistemaSeguridad.ino
 * @brief Sistema de confort térmico y control de acceso basado en FSM.
 *
 * @details
 * Sistema embebido sobre Arduino Mega 2560 que combina control de acceso
 * con RFID y gestión de confort térmico mediante PMV (Predicted Mean Vote).
 *
 * Arquitectura principal:
 *  - Máquina de estados finita (FSM) con 8 estados.
 *  - Toda la temporización usa millis() — sin delay() bloqueantes en loop().
 *  - Interrupciones reales: botón físico (INT3/pin20) y sensor fuego digital (INT4/pin19).
 *  - EEPROM persiste usuarios, roles, horarios y umbrales entre reinicios.
 *  - Reloj interno manual (sin RTC físico): configurable desde GESTIÓN, tecla 5.
 *  - PMV calculado con modelo de Fanger, ajustado por rol y número de personas.
 *
 * Estados FSM:
 *  - INICIO:            Autenticación RFID + clave. 3 intentos fallidos → BLOQUEO.
 *  - CONFIG:            Registro de usuarios (solo Gerente autenticado).
 *  - MONITOR_AMBIENTAL: Lectura DHT11, LDR, fuego. Control relay por PMV.
 *  - MONITOR_PUERTAS:   Detección Hall + Micrófono. 3 eventos → GESTION.
 *  - ALARMA:            Buzzer + LED rojo. Countdown. 3 alarmas → BLOQUEO.
 *  - BLOQUEO:           Sistema bloqueado. Desbloqueo por Seguridad/Gerente.
 *  - GESTION:           Ajuste de umbrales y consulta de usuarios (Coordinador+).
 *  - CAMBIO_CLAVE:      Cambio obligatorio de clave cada 4 usos.
 *
 * Tabla de pines:
 *  | Pin | Componente             | Modo        |
 *  |-----|------------------------|-------------|
 *  |   2 | LCD D7                 | OUTPUT      |
 *  |   3 | LCD D6                 | OUTPUT      |
 *  |   4 | LCD D5                 | OUTPUT      |
 *  |   5 | LCD D4                 | OUTPUT      |
 *  |   7 | DHT11 datos            | INPUT       |
 *  |  10 | Buzzer                 | OUTPUT      |
 *  |  11 | LCD Enable             | OUTPUT      |
 *  |  12 | LCD RS                 | OUTPUT      |
 *  |  13 | Servo (cerradura)      | OUTPUT      |
 *  |  19 | Sensor fuego digital   | INPUT_PU    |
 *  |  20 | Botón físico           | INPUT_PU    |
 *  |  22 | Relé ventilador        | OUTPUT      |
 *  |  24 | LED RGB - Rojo         | OUTPUT      |
 *  |  26 | LED RGB - Verde        | OUTPUT      |
 *  |  28 | LED RGB - Azul         | OUTPUT      |
 *  |  30 | Teclado fila 0         | OUTPUT      |
 *  |  32 | Teclado fila 1         | OUTPUT      |
 *  |  34 | Teclado fila 2         | OUTPUT      |
 *  |  36 | Teclado fila 3         | OUTPUT      |
 *  |  38 | Teclado col 0          | INPUT_PU    |
 *  |  40 | Teclado col 1          | INPUT_PU    |
 *  |  42 | Teclado col 2          | INPUT_PU    |
 *  |  44 | Teclado col 3          | INPUT_PU    |
 *  |  49 | RFID RST               | OUTPUT      |
 *  |  53 | RFID SS/SDA            | OUTPUT      |
 *  |  A0 | LDR                    | INPUT       |
 *  |  A2 | Sensor Hall            | INPUT       |
 *  |  A3 | Micrófono analógico    | INPUT       |
 *  |  A4 | Sensor fuego analógico | INPUT       |
 *
 * Layout EEPROM:
 *  | Addr    | Contenido                           |
 *  |---------|-------------------------------------|
 *  | 0       | MAGIC byte (0xA8)                   |
 *  | 1       | Total de usuarios                   |
 *  | 2..331  | Registros de usuario (6 × 55 bytes) |
 *  | 332     | UMBRAL_TEMP float (4 bytes)          |
 *  | 336     | UMBRAL_LUZ int (2 bytes)             |
 *
 * @author Sebastián Ruiz / Miguel Mera
 * @date   2026
 * @version 1.0
 */

#include <LiquidCrystal.h>
#include <Keypad.h>
#include <SPI.h>
#include <MFRC522.h>
#include <DHT.h>
#include <Servo.h>
#include <EEPROM.h>

// ── MODO DIAGNÓSTICO ──
// Cambiar a 1 SOLO durante pruebas en hardware para ver telemetría por Serial Monitor.
// Con 0, todas las funciones debug...() se compilan como inline vacías sin overhead.
#define DEBUG_SERIAL 1

// ============================================================================
// PINES
// ============================================================================
#define DHTPIN          7
#define DHTTYPE         DHT11
#define LDR_PIN         A0
#define HALL_PIN        A2
#define MIC_PIN         A3
#define FUEGO_ANA_PIN   A4
#define BOTON_PIN       20    ///< INT3 en Mega
#define FUEGO_INT_PIN   19    ///< INT4 en Mega
#define RELAY_PIN       22
#define SERVO_PIN       13
#define RED_PIN         24
#define GREEN_PIN       26
#define BLUE_PIN        28
#define BUZZER_PIN      10
#define SS_PIN          53
#define RST_PIN         49

// ============================================================================
// EEPROM — LAYOUT (sin colisión)
// ============================================================================
// [0]       MAGIC byte
// [1]       N_USR byte
// [2..331]  Registros de usuario (6 x 55 bytes = 330 bytes)  addr 2..331
// [332]     UMBRAL_TEMP float (4 bytes)
// [336]     UMBRAL_LUZ  int   (2 bytes)

#define EEPROM_MAGIC_ADDR   0
#define EEPROM_MAGIC_VAL    0xA8
#define EEPROM_N_USR_ADDR   1
#define EEPROM_USR_BASE     2
#define EEPROM_UMBRAL_TEMP  332
#define EEPROM_UMBRAL_LUZ   336
#define MAX_USUARIOS        6

#define USR_NOMBRE_LEN  12
#define USR_TAG_LEN     21   ///< UID RFID hasta 7 bytes = 14 hex chars + null
#define USR_CLAVE_LEN    9
#define USR_RECORD_SIZE (USR_NOMBRE_LEN + USR_TAG_LEN + USR_CLAVE_LEN + \
                         1 + 1 + 1 + 1 + USR_CLAVE_LEN)  ///< = 55 bytes

// ============================================================================
// ROLES
// ============================================================================
enum Rol : uint8_t {
  ROL_OPERARIO    = 0,
  ROL_SEGURIDAD   = 1,
  ROL_COORDINADOR = 2,
  ROL_GERENTE     = 3
};

const float TEMP_CONFORT[] = {24.0, 23.5, 23.0, 22.0};
const char* NOMBRE_ROL[]   = {"Operario", "Seguridad", "Coordinad", "Gerente"};

// ============================================================================
// ESTRUCTURA USUARIO
// ============================================================================
struct Usuario {
  char    nombre[USR_NOMBRE_LEN];
  char    tag[USR_TAG_LEN];
  char    clave[USR_CLAVE_LEN];
  char    claveAnt[USR_CLAVE_LEN];
  uint8_t horaIni;
  uint8_t horaFin;
  Rol     rol;
  uint8_t usosClave;
};

Usuario  usuarios[MAX_USUARIOS];
uint8_t  totalUsuarios = 0;
Usuario* usuarioActivo = nullptr;

// ============================================================================
// PERIFÉRICOS
// ============================================================================
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

const byte ROWS = 4, COLS = 4;
char keyMap[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {30, 32, 34, 36};
byte colPins[COLS] = {38, 40, 42, 44};
Keypad keypad = Keypad(makeKeymap(keyMap), rowPins, colPins, ROWS, COLS);

MFRC522    mfrc522(SS_PIN, RST_PIN);
DHT        dht(DHTPIN, DHTTYPE);
Servo      servo;

// ============================================================================
// FSM — ESTADOS
// ============================================================================
enum Estado {
  INICIO,
  CONFIG,
  MONITOR_AMBIENTAL,
  MONITOR_PUERTAS,
  ALARMA,
  BLOQUEO,
  GESTION,
  CAMBIO_CLAVE      ///< Sub-estado no-bloqueante para forzarCambioClave
};

Estado estadoActual   = INICIO;
Estado estadoAnterior = INICIO;

// ============================================================================
// FLAGS ISR — VOLATILE
// ============================================================================
volatile bool flagBoton = false;
volatile bool flagFuego = false;

void ISR_Boton() { flagBoton = true; }
void ISR_Fuego() { flagFuego = true; }

// ============================================================================
// TEMPORIZACIÓN
// ============================================================================
const unsigned long T_ACCESO_OK          = 5000UL;
const unsigned long T_CONFIG             = 7000UL;
const unsigned long T_MONITOR_AMB        = 5000UL;
const unsigned long T_MONITOR_PUERTAS    = 2000UL;
const unsigned long T_EVENTO_ALARMA      = 4000UL;   ///< Tiempo de evento sostenido para activar alarma de puerta (4 s)
const unsigned long T_ALARMA_DUR         = 3000UL;
const unsigned long T_SERVO_ABIERTO      = 5000UL;
const unsigned long T_MSG_ERROR          = 1500UL;
const unsigned long T_BLOQUEO_AUTO       = 7000UL;
const unsigned long T_DET_RESET          = 30000UL;
const unsigned long T_CONFIG_AUTH        = 12000UL; ///< timeout para autenticación de CONFIG
const unsigned long T_CAMBIO_CLAVE       = 30000UL; ///< timeout máximo para completar cambio de clave
const unsigned long T_SENSOR_GLOBAL      = 500UL;   ///< intervalo de actualización de la capa de sensores
const unsigned long T_PMV_STALE          = 10000UL; ///< tiempo máximo antes de considerar el PMV desactualizado
const unsigned long T_FUEGO_FILTRO_MS    = 100UL;   ///< tiempo mínimo entre muestras del filtro de fuego
const unsigned long T_HALL_FILTRO_MS     = 80UL;    ///< Tiempo de debounce del sensor Hall
const unsigned long T_MIC_FILTRO_MS      = 150UL;   ///< tiempo mínimo entre muestras válidas del micrófono
const unsigned long T_REG_PASO          = 20000UL;  ///< tiempo máximo por paso en el registro de usuario

// ── Umbrales de detección de sensores ──
const int FUEGO_UMBRAL = 250;   ///< Ajustado a 250 para el KY-013 (equivale a ~53°C, en reposo lee ~512 a 25°C)
const int HALL_UMBRAL  = 512;   ///< valor ADC por encima del cual se considera apertura de puerta
const int MIC_UMBRAL   = 600;   ///< valor ADC por encima del cual se considera sonido relevante
const int LDR_UMBRAL_MAX = 900; ///< valor ADC máximo razonable de LDR (filtro de lectura errada)

const unsigned long LED_ALARMA_ON   = 100UL;
const unsigned long LED_ALARMA_OFF  = 200UL;
const unsigned long LED_BLOQUEO_ON  = 300UL;
const unsigned long LED_BLOQUEO_OFF = 700UL;

unsigned long tEstado = 0;
unsigned long tLed    = 0;
bool          ledOn   = false;

// ── INICIO ──
unsigned long tAccesoOK    = 0;
unsigned long tMsgError    = 0;
bool          accesoOK     = false;
bool          inicioMostro = false;
bool          servoAbierto = false;
unsigned long tServoCierre = 0;
bool          mostrandoError = false;
bool          esperandoOK    = false;
char          tagLeido[USR_TAG_LEN]   = "";
char          tagBloqueo[USR_TAG_LEN] = "";

// ── CONFIG ──
unsigned long tConfig      = 0;
bool          configMostro = false;

// ── GESTIÓN input numérico no-bloqueante ──
enum GestInputState { GEST_IDLE, GEST_TEMP, GEST_LUZ, GEST_HORA, GEST_MINUTO };
GestInputState gestInputState = GEST_IDLE;

// ── MONITOR_AMBIENTAL ──
unsigned long tAmbiental = 0;
bool          ambMostro  = false;
unsigned long lastDHTRead     = 0;
const unsigned long T_DHT_INTERVALO = 2000UL;
bool          dhtValido       = false; ///< true tras primera lectura válida del DHT11
bool          humedadValida   = false; ///< true tras primera lectura válida de humedad

// ── MONITOR_PUERTAS ──
unsigned long tPuertas          = 0;
unsigned long tEvento           = 0;
bool          enEvento          = false;
bool          puertasMostro     = false;
unsigned long tDeteccionesReset = 0;
bool          hallAnterior      = false;
unsigned long tDebounceHall     = 0;
// T_MIC_FILTRO_MS (150ms) = gap mínimo entre muestras de mic que contribuyen al evento.
// La ventana de evento sostenido sigue siendo T_EVENTO_ALARMA (1800ms) sin cambios.
unsigned long tMicUltimaLectura = 0;  ///< timestamp de la última muestra mic válida
bool          micConfirmado     = false; ///< true si el mic superó umbral en esta ventana

// ── ALARMA ──
unsigned long tAlarma     = 0;
bool          alarmaMostro = false;
char          causaAlarma[12] = "";

// ── BLOQUEO ──
bool bloqueoMostro = false;

bool bloqueoAuthPendiente = false;   ///< true mientras se espera auth de Gerente desde BLOQUEO
unsigned long tBloqueoAuth = 0;      ///< timer de timeout para auth desde BLOQUEO

// ── GESTIÓN ──
bool          gestionMostro  = false;
int           gestionPagUsr  = 0;
unsigned long tGestionScroll = 0;
bool          gestionScrollActivo = false;

// ── CAMBIO_CLAVE (sub-FSM no-bloqueante) ──
int           cambioClaveIdx      = -1;
char          nuevaClave[9]       = "";
uint8_t       nuevaClaveLen       = 0;
bool          cambioClaveOK       = false;
bool          cambioClaveMostro   = false;
unsigned long tCambioClave        = 0; ///< timer de seguridad para el estado CAMBIO_CLAVE
bool          esperandoTimeout    = false; ///< true mientras se muestra mensaje de timeout
unsigned long tMsgTimeoutClave    = 0;     ///< timestamp para mensaje de timeout no-bloqueante

// ── CONFIG auth ──
bool     configAuthPendiente = false; ///< true mientras se espera auth para CONFIG
unsigned long tConfigAuth    = 0;     ///< timer de timeout para auth de CONFIG
char     configAuthTag[USR_TAG_LEN] = ""; ///< tag RFID para auth
bool     configAuthTagOK     = false;
bool     configAuthError     = false; ///< flag de error de autenticación CONFIG
unsigned long tConfigAuthError = 0;   ///< timer para mostrar mensaje de error CONFIG
enum RegState { REG_IDLE, REG_RFID, REG_NOMBRE, REG_CLAVE, REG_ROL, REG_HORA_INI, REG_HORA_FIN, REG_DONE };
RegState      regState    = REG_IDLE;
unsigned long tRegRFID    = 0;
unsigned long tRegPaso    = 0;     ///< timestamp de inicio de cada paso del registro
char          regTag[USR_TAG_LEN] = "";
char          regNombre[USR_NOMBRE_LEN] = "";
char          regClave[USR_CLAVE_LEN]   = "";
uint8_t       regClaveLen = 0;
uint8_t       regRol      = 0;
uint8_t       regHoraIni  = 0;
uint8_t       regHoraFin  = 23;
char          regBuf[17]  = "";
uint8_t       regBufLen   = 0;

// ============================================================================
// CONTADORES
// ============================================================================
int  intentosFallidos  = 0;
int  alarmasConsec     = 0;
int  deteccionesPuerta = 0;
bool cicloAmbSinAlarma = false;
uint8_t fuegoLecturas  = 0; ///< contador antirruido del sensor de fuego (requiere 3 lecturas)

int personasPresentes = 0;

// ============================================================================
// SENSORES / UMBRALES
// ============================================================================
float tempDHT    = 0.0;   ///< temperatura leída del DHT11
float humedad    = 0.0;
int   luzLDR     = 0;
int   valorFuego = 1023;

float UMBRAL_TEMP = 49.0;
int   UMBRAL_LUZ  = 999;

// ── PMV ──
float pmvActual = 0.0;
bool          pmvValido              = false; ///< true tras primera actualización válida de sensores
unsigned long tUltimaLecturaAmbiental = 0;    ///< timestamp de la última lectura ambiental
unsigned long tSensorGlobal           = 0;    ///< timer para la capa unificada de sensores

// ── RELOJ INTERNO MANUAL — sin RTC ──
uint8_t horaActual = 8;       ///< Hora lógica actual configurada manualmente (0..23)
uint8_t minutoActual = 0;     ///< Minuto lógico actual configurado manualmente (0..59)
uint8_t horaTempConfig = 0;   ///< Auxiliar para configuración HH:MM desde GESTIÓN
unsigned long tRelojInterno = 0;
bool relojConfigurado = false; ///< true cuando el usuario configura hora desde GESTIÓN

// ── Filtro temporal sensor de fuego ──
unsigned long tFuegoUltimaLectura = 0;  ///< timestamp de la última lectura que bajó el umbral

// ── FILTRO RUIDO BUZZER y SERVO ──
unsigned long tBuzzerSilencioHasta = 0;
unsigned long tServoSilencioHasta  = 0;
void myTone(uint8_t pin, unsigned int frequency) {
  ::tone(pin, frequency);
}
void myTone(uint8_t pin, unsigned int frequency, unsigned long duration) {
  ::tone(pin, frequency, duration);
  if (pin == BUZZER_PIN) {
    tBuzzerSilencioHasta = millis() + duration + 150;
  }
}
#define tone(...) myTone(__VA_ARGS__)

// ============================================================================
// ENTRADA DE CLAVE — char[] no-bloqueante
// ============================================================================
char    claveIngresada[9] = "";
uint8_t claveLen          = 0;
bool    claveConfirmada   = false;
bool    claveCancelada    = false;

/**
 * @brief Procesa una tecla del teclado matricial para entrada de clave no-bloqueante.
 * @details '#' confirma la clave si tiene >= 4 dígitos. '*' borra el buffer.
 *          Dígitos 0-9 se acumulan hasta máximo 8.
 * @param mostrarEnLCD Si es true, actualiza el LCD con asteriscos '*' en línea 1.
 */
void procesarTeclaClave(bool mostrarEnLCD = true) {
  char key = keypad.getKey();
  if (!key) return;

  if (key == '#') {
    if (claveLen >= 4) claveConfirmada = true;
    return;
  }
  if (key == '*') {
    claveLen = 0;
    claveIngresada[0] = '\0';
    claveCancelada  = false;
    claveConfirmada = false;
    if (mostrarEnLCD) {
      lcd.setCursor(0, 1);
      lcd.print(F("                "));
      lcd.setCursor(0, 1);
    }
    return;
  }
  if (key >= '0' && key <= '9' && claveLen < 8) {
    claveIngresada[claveLen++] = key;
    claveIngresada[claveLen]   = '\0';
    if (mostrarEnLCD) {
      lcd.setCursor(0, 1);
      for (uint8_t i = 0; i < claveLen; i++)   lcd.print('*');
      for (uint8_t i = claveLen; i < 16; i++)  lcd.print(' ');
    }
  }
}

/**
 * @brief Procesa una tecla ya leída para entrada de clave no-bloqueante.
 * @details Versión que recibe la tecla como parámetro para evitar doble
 *          llamada a keypad.getKey() en CONFIG_AUTH y BLOQUEO_AUTH. '#' confirma la
 *          clave si tiene >= 4 dígitos. '*' borra el buffer. Dígitos 0-9 se acumulan.
 * @param key         Tecla ya leída por el caller.
 * @param mostrarEnLCD Si es true, actualiza el LCD con asteriscos '*' en línea 1.
 */
void procesarTeclaClaveConKey(char key, bool mostrarEnLCD = true) {
  if (!key) return;

  if (key == '#') {
    if (claveLen >= 4) claveConfirmada = true;
    return;
  }
  if (key == '*') {
    claveLen = 0;
    claveIngresada[0] = '\0';
    claveCancelada  = false;
    claveConfirmada = false;
    if (mostrarEnLCD) {
      lcd.setCursor(0, 1);
      lcd.print(F("                "));
      lcd.setCursor(0, 1);
    }
    return;
  }
  if (key >= '0' && key <= '9' && claveLen < 8) {
    claveIngresada[claveLen++] = key;
    claveIngresada[claveLen]   = '\0';
    if (mostrarEnLCD) {
      lcd.setCursor(0, 1);
      for (uint8_t i = 0; i < claveLen; i++)   lcd.print('*');
      for (uint8_t i = claveLen; i < 16; i++)  lcd.print(' ');
    }
  }
}


void resetClave() {
  claveLen = 0;
  claveIngresada[0] = '\0';
  claveConfirmada = false;
  claveCancelada  = false;
}

// ============================================================================
// EEPROM
// ============================================================================
/**
 * @brief Calcula la dirección EEPROM del registro del usuario i.
 * @param i Índice del usuario (0..MAX_USUARIOS-1).
 * @return Dirección base en EEPROM.
 */
int eepromDirUsuario(int i) {
  return EEPROM_USR_BASE + i * USR_RECORD_SIZE;
}

/**
 * @brief Guarda todos los usuarios y umbrales en EEPROM.
 */
void eepromGuardarTodo() {
  EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VAL);
  EEPROM.write(EEPROM_N_USR_ADDR, totalUsuarios);
  for (int i = 0; i < totalUsuarios; i++) {
    EEPROM.put(eepromDirUsuario(i), usuarios[i]);
  }
  EEPROM.put(EEPROM_UMBRAL_TEMP, UMBRAL_TEMP);
  EEPROM.put(EEPROM_UMBRAL_LUZ,  UMBRAL_LUZ);
}

/**
 * @brief Carga usuarios y umbrales desde EEPROM. Si MAGIC no coincide, inicializa defaults.
 */
void eepromCargar() {
  if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC_VAL) {
    totalUsuarios = 2;

    strncpy(usuarios[0].nombre,   "Admin",    USR_NOMBRE_LEN - 1);
    strncpy(usuarios[0].tag,      "F23BE52E", USR_TAG_LEN - 1);
    strncpy(usuarios[0].clave,    "1234",     USR_CLAVE_LEN - 1);
    strncpy(usuarios[0].claveAnt, "",         USR_CLAVE_LEN - 1);
    usuarios[0].nombre[USR_NOMBRE_LEN-1] = '\0';
    usuarios[0].horaIni  = 0;
    usuarios[0].horaFin  = 23;
    usuarios[0].rol      = ROL_GERENTE;
    usuarios[0].usosClave = 0;

    strncpy(usuarios[1].nombre,   "Andrea",   USR_NOMBRE_LEN - 1);
    strncpy(usuarios[1].tag,      "59D8C019", USR_TAG_LEN - 1);
    strncpy(usuarios[1].clave,    "5678",     USR_CLAVE_LEN - 1);
    strncpy(usuarios[1].claveAnt, "",         USR_CLAVE_LEN - 1);
    usuarios[1].nombre[USR_NOMBRE_LEN-1] = '\0';
    usuarios[1].horaIni  = 6;
    usuarios[1].horaFin  = 18;
    usuarios[1].rol      = ROL_OPERARIO;
    usuarios[1].usosClave = 0;

    eepromGuardarTodo();
    return;
  }

  totalUsuarios = EEPROM.read(EEPROM_N_USR_ADDR);
  if (totalUsuarios > MAX_USUARIOS) totalUsuarios = 0;

  for (int i = 0; i < totalUsuarios; i++) {
    EEPROM.get(eepromDirUsuario(i), usuarios[i]);
    usuarios[i].nombre[USR_NOMBRE_LEN-1] = '\0';
    usuarios[i].tag[USR_TAG_LEN-1]       = '\0';
    usuarios[i].clave[USR_CLAVE_LEN-1]   = '\0';
    usuarios[i].claveAnt[USR_CLAVE_LEN-1]= '\0';
  }

  EEPROM.get(EEPROM_UMBRAL_TEMP, UMBRAL_TEMP);
  EEPROM.get(EEPROM_UMBRAL_LUZ,  UMBRAL_LUZ);

 if (UMBRAL_TEMP < 10.0 || UMBRAL_TEMP > 80.0) UMBRAL_TEMP = 49.0;
 if (UMBRAL_LUZ  < 50   || UMBRAL_LUZ  > 1023) UMBRAL_LUZ  = 999;
}

/**
 * @brief Guarda un único usuario en EEPROM y actualiza contadores.
 * @param i Índice del usuario a guardar.
 */
void eepromGuardarUsuario(int i) {
  EEPROM.put(eepromDirUsuario(i), usuarios[i]);
  EEPROM.write(EEPROM_N_USR_ADDR, totalUsuarios);
  EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VAL);
}

// ============================================================================
// HORARIO — reloj interno manual sin RTC
// ============================================================================
/**
 * @brief Avanza un reloj lógico interno usando millis().
 * @details No es hora real autónoma como un RTC; la hora se configura manualmente
 *          desde GESTIÓN con la tecla 5. Si Arduino se reinicia, vuelve a 08:00.
 */
void actualizarRelojInterno() {
  if (millis() - tRelojInterno >= 60000UL) {
    tRelojInterno += 60000UL;
    minutoActual++;
    if (minutoActual >= 60) {
      minutoActual = 0;
      horaActual++;
      if (horaActual >= 24) horaActual = 0;
    }
  }
}

/**
 * @brief Imprime la hora lógica HH:MM en el LCD.
 */
void imprimirHoraInternaLCD() {
  if (horaActual < 10) lcd.print(F("0"));
  lcd.print(horaActual);
  lcd.print(F(":"));
  if (minutoActual < 10) lcd.print(F("0"));
  lcd.print(minutoActual);
}

/**
 * @brief Valida si el usuario puede acceder según su ventana horaria.
 * @details Sin RTC físico, compara contra horaActual/minutoActual del reloj interno.
 *          Soporta rangos normales y rangos que cruzan medianoche. Gerente siempre accede.
 * @param u Usuario a validar.
 * @return true si no se ha configurado la hora, o si la hora lógica está dentro de la ventana permitida.
 */
bool horarioPermitido(const Usuario& u) {
  if (u.rol == ROL_GERENTE) return true;

  // Para no romper el flujo anterior, si nadie configuró la hora manual,
  // el sistema permite acceso. La validación horaria se activa después de GESTIÓN → tecla 5.
  if (!relojConfigurado) return true;

  uint8_t hora = horaActual;
  if (u.horaIni <= u.horaFin) {
    return (hora >= u.horaIni && hora < u.horaFin);
  } else {
    return (hora >= u.horaIni || hora < u.horaFin);
  }
}

// ============================================================================
// LED RGB
// ============================================================================
/**
 * @brief Establece el color del LED RGB.
 * @param r Componente rojo.
 * @param g Componente verde.
 * @param b Componente azul.
 */
void setColor(bool r, bool g, bool b) {
  digitalWrite(RED_PIN,   r ? HIGH : LOW);
  digitalWrite(GREEN_PIN, g ? HIGH : LOW);
  digitalWrite(BLUE_PIN,  b ? HIGH : LOW);
}

/** @brief Apaga el LED RGB. */
void setColorOff() { setColor(false, false, false); }

/**
 * @brief Parpadea el LED RGB de forma no-bloqueante.
 * @param r,g,b Color de parpadeo.
 * @param onMs  Duración del encendido en ms.
 * @param offMs Duración del apagado en ms.
 */
void parpadeaLED(bool r, bool g, bool b, unsigned long onMs, unsigned long offMs) {
  unsigned long ahora = millis();
  if (ledOn  && (ahora - tLed >= onMs))  { ledOn = false; tLed = ahora; setColorOff(); }
  else if (!ledOn && (ahora - tLed >= offMs)) { ledOn = true;  tLed = ahora; setColor(r,g,b); }
}

// ============================================================================
// SERVO — CERRADURA
// ============================================================================
/**
 * @brief Abre la cerradura (servo a 0°) e inicia el temporizador de cierre automático.
 */
void abrirCerradura() {
  servo.write(0);
  tServoSilencioHasta = millis() + 1000; // Silenciar sensor de fuego por 1 segundo por el consumo del servo
  servoAbierto = true;
  tServoCierre = millis();
  lcd.setCursor(0, 1);
  lcd.print(F("OPEN            "));
}

/**
 * @brief Gestiona el cierre automático del servo. Debe llamarse cada ciclo del loop().
 */
void gestionarServoCierre() {
  if (!servoAbierto) return;
  unsigned long transcurrido = millis() - tServoCierre;
  if (transcurrido < T_SERVO_ABIERTO) {
    unsigned long restante = (T_SERVO_ABIERTO - transcurrido) / 1000UL;
    if (estadoActual == INICIO && accesoOK) {
      lcd.setCursor(0, 0);
      lcd.print(F("OPEN "));
      if (restante < 10) lcd.print(F("0"));
      lcd.print(restante);
      lcd.print(F("s           "));
    }
  } else {
    servo.write(90);
    tServoSilencioHasta = millis() + 1000; // Silenciar sensor de fuego por 1 segundo al cerrar el servo
    servoAbierto = false;
  }
}

// ============================================================================
// SENSORES
// ============================================================================
/**
 * @brief Lee los sensores analógicos rápidos (LDR y detector de fuego analógico).
 * void leerSensores() {
  luzLDR     = analogRead(LDR_PIN);
  valorFuego = analogRead(FUEGO_ANA_PIN);
}

// ── leerLM35() y fusionarTemperatura() eliminadas en v19 (sensor LM35 no disponible) ──
// La temperatura se obtiene únicamente del DHT11. tempDHT es la única fuente térmica.

// ============================================================================
// CAPA UNIFICADA DE SENSORES
// ============================================================================
/**
 * @brief Actualiza todos los sensores y recalcula PMV de forma centralizada.
 * @details Llamada desde loop() cada T_SENSOR_GLOBAL (500 ms).
 *          Evita lecturas dispersas en múltiples estados y garantiza que pmvActual
 *          y tempDHT sean siempre frescos. Si los datos superan T_PMV_STALE (10s),
 *          pmvValido=false y verificarPMVAlarma() no disparará con datos viejos.
 */
void actualizarSensoresGlobal() {
  luzLDR     = constrain(analogRead(LDR_PIN), 0, LDR_UMBRAL_MAX);
  valorFuego = analogRead(FUEGO_ANA_PIN);

  // DHT con intervalo propio
  if (millis() - lastDHTRead >= T_DHT_INTERVALO) {
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t)) { tempDHT = t; dhtValido = true; }
    if (!isnan(h)) { humedad = h; humedadValida = true; }
    lastDHTRead = millis();
  }

  // PMV solo si DHT tiene lectura válida
  if (dhtValido) {
    float humUsar = humedadValida ? humedad : 50.0f;
    const float MET_ROL[] = {120.0, 100.0, 80.0, 70.0};
    float met = (usuarioActivo != nullptr) ? MET_ROL[usuarioActivo->rol] : 80.0f;
    met += (personasPresentes > 1) ? (personasPresentes - 1) * 2.0f : 0.0f;
    pmvActual = calcularPMV(tempDHT, humUsar, met);
    pmvValido = true;
    tUltimaLecturaAmbiental = millis();
  }
}

/**
 * @brief Indica si los datos del PMV son recientes (< T_PMV_STALE ms).
 * @return true si pmvValido y la lectura tiene menos de T_PMV_STALE ms.
 */
bool sensoresValidos() {
  if (!pmvValido) return false;
  return (millis() - tUltimaLecturaAmbiental) < T_PMV_STALE;
}

// ============================================================================
// DIAGNÓSTICO SERIAL
// ============================================================================
#if DEBUG_SERIAL

/**
 * @brief Nombres de los estados FSM para diagnóstico.
 */
static const char* NOMBRE_ESTADO[] = {
  "INICIO", "CONFIG", "MON_AMB", "MON_PUERTAS",
  "ALARMA", "BLOQUEO", "GESTION", "CAMBIO_CLAVE"
};

/**
 * @brief Imprime telemetría completa de sensores por Serial.
 * @details Imprime por Serial: tempDHT, humedad, luzLDR, valorFuego,
 *          pmvActual, pmvValido, personasPresentes, alarmasConsec, detecciones,
 *          edad de la última lectura ambiental en ms.
 */
void debugSensores() {
  Serial.print(F("[SENS] T_DHT="));    Serial.print(tempDHT, 1);
  Serial.print(F(" H="));              Serial.print(humedad, 0);
  Serial.print(F("% LDR="));           Serial.print(luzLDR);
  Serial.print(F(" Fuego="));          Serial.print(valorFuego);
  Serial.print(F(" PMV="));            Serial.print(pmvActual, 2);
  Serial.print(F(" pmvOK="));          Serial.print(pmvValido ? "S" : "N");
  Serial.print(F(" edadSens="));
  Serial.print(pmvValido ? (millis() - tUltimaLecturaAmbiental) : 99999UL);
  Serial.print(F("ms personas="));     Serial.print(personasPresentes);
  Serial.print(F(" alarmasC="));       Serial.print(alarmasConsec);
  Serial.print(F(" detPuerta="));      Serial.println(deteccionesPuerta);
}

/**
 * @brief Imprime la transición de estado FSM por Serial.
 * @param de  Estado origen.
 * @param a   Estado destino.
 */
void debugTransicion(Estado de, Estado a) {
  Serial.print(F("[FSM] "));
  Serial.print(NOMBRE_ESTADO[de]);
  Serial.print(F(" -> "));
  Serial.println(NOMBRE_ESTADO[a]);
}

/**
 * @brief Imprime estado FSM actual y usuario activo por Serial.
 */
void debugEstadoActual() {
  Serial.print(F("[LOOP] Estado="));
  Serial.print(NOMBRE_ESTADO[estadoActual]);
  if (usuarioActivo) {
    Serial.print(F(" User="));  Serial.print(usuarioActivo->nombre);
    Serial.print(F(" Rol="));   Serial.print(usuarioActivo->rol);
  } else {
    Serial.print(F(" User=NONE"));
  }
  Serial.println();
}

#else
// Versión vacía para producción — el compilador las elimina sin overhead
inline void debugSensores()                     {}
inline void debugTransicion(Estado, Estado)     {}
inline void debugEstadoActual()                 {}
#endif  // DEBUG_SERIAL

/**
 * @brief Determina si hay fuego detectado por lectura analógica.
 * @return true si el valor analógico del sensor de fuego está por debajo del umbral.
 */
bool fuegoDetectado() { return valorFuego < FUEGO_UMBRAL; }

/**
 * @brief Verifica emergencia de fuego y transiciona a ALARMA si corresponde.
 * @details Lee analogRead(FUEGO_ANA_PIN) antes de evaluar el umbral.
 *          T_FUEGO_FILTRO_MS (100ms). Evita que 3 lecturas ocurran en microsegundos
 *          (loop muy rápido). La ISR sigue disparando inmediatamente sin filtro.
 * @return true si se detectó fuego confirmado y se cambió a ALARMA.
 */
bool verificarEmergenciaFuego() {
  if (estadoActual == ALARMA) return false;

  // Si el buzzer o el servo se activaron recientemente, silenciamos temporalmente el sensor de fuego para evitar falsos positivos por caída de tensión
  if ((tBuzzerSilencioHasta > 0 && millis() < tBuzzerSilencioHasta) || 
      (tServoSilencioHasta > 0 && millis() < tServoSilencioHasta)) {
    noInterrupts(); flagFuego = false; interrupts();
    fuegoLecturas = 0;
    tFuegoUltimaLectura = 0;
    return false;
  }

  // El KY-013 es puramente analógico (no tiene salida digital).
  // Ignoramos y vaciamos el flag de la interrupción del Pin 19 para evitar ruido por pin flotante.
  noInterrupts(); flagFuego = false; interrupts();

  valorFuego = analogRead(FUEGO_ANA_PIN);
  unsigned long ahora = millis();
  if (valorFuego < FUEGO_UMBRAL) {
    if (ahora - tFuegoUltimaLectura >= T_FUEGO_FILTRO_MS) {
      fuegoLecturas++;
      tFuegoUltimaLectura = ahora;
    }
  } else {
    fuegoLecturas = 0;
    tFuegoUltimaLectura = 0;
  }

  if (fuegoLecturas >= 3) {
    fuegoLecturas = 0;
    tFuegoUltimaLectura = 0;
    strncpy(causaAlarma, "FUEGO", sizeof(causaAlarma) - 1);
    causaAlarma[sizeof(causaAlarma)-1] = '\0';
    cambiarEstado(ALARMA);
    return true;
  }
  return false;
}

/**
 * @brief Verifica globalmente si el PMV supera el umbral de alarma.
 * @details Llamada en loop() junto a verificarEmergenciaFuego().
 *          Evita que un PMV calculado hace >10s dispare alarma desde CONFIG o GESTION.
 * @return true si se detectó PMV alto confirmado y se cambió a ALARMA.
 */
bool verificarPMVAlarma() {
  if (estadoActual == ALARMA || estadoActual == BLOQUEO) return false;
  if (!sensoresValidos()) return false;
  if (pmvActual > 0.7f) {
    strncpy(causaAlarma, "PMV ALTO", sizeof(causaAlarma) - 1);
    causaAlarma[sizeof(causaAlarma)-1] = '\0';
    cambiarEstado(ALARMA);
    return true;
  }
  return false;
}


/**
 * @brief Calcula el Predicted Mean Vote (PMV) simplificado según ASHRAE 55 / ISO 7730.
 *
 * @details Aproximación polinomial de Fanger para uso embebido, calibrada para el
 *          rango 18–32 °C y 30–80 % HR. La temperatura neutral depende de la actividad
 *          metabólica del rol del usuario.
 *          Ajuste por ocupación: por cada 5 personas adicionales se reduce la temperatura
 *          neutral en 0.3 °C, simulando el calor adicional generado por el grupo.
 *
 * @param temp        Temperatura fusionada del aire (°C).
 * @param humedad     Humedad relativa (%).
 * @param metActivity Actividad metabólica (W/m²):
 *                    Operario=120, Seguridad=100, Coordinador=80, Gerente=70.
 * @return PMV en [-3, +3]. < -0.7 = muy frío; [-0.7, +0.5] = confort; > +0.5 = caliente.
 */
float calcularPMV(float temp, float hum, float metActivity) {
  float to = temp;
  float tw = to - ((100.0 - hum) / 5.0);
  float T_neutral = 33.5 - (metActivity - 58.0) * 0.08;

  int gruposPersonas = min(personasPresentes / 5, 4);  // máximo 4 grupos (20 personas)
  T_neutral -= gruposPersonas * 0.3;

  float pmv = 0.22 * (to - T_neutral) + 0.006 * (hum - 50.0);
  pmv += 0.05 * (tw - 18.0) * 0.01;

  if (pmv >  3.0) pmv =  3.0;
  if (pmv < -3.0) pmv = -3.0;
  return pmv;
}

// ============================================================================
// RFID
// ============================================================================
/**
 * @brief Lee un tag RFID y lo convierte a cadena hexadecimal en mayúsculas.
 * @param out    Buffer de salida (se escribe la UID como hex + null).
 * @param maxLen Tamaño del buffer de salida (debe ser >= USR_TAG_LEN).
 * @return true si se leyó un tag correctamente.
 */
bool leerTag(char* out, uint8_t maxLen) {
  if (!mfrc522.PICC_IsNewCardPresent()) return false;
  if (!mfrc522.PICC_ReadCardSerial())   return false;
  uint8_t j = 0;
  for (byte i = 0; i < mfrc522.uid.size && j < maxLen - 1; i++) {
    uint8_t b = mfrc522.uid.uidByte[i];
    char hi = (b >> 4);
    char lo = (b & 0x0F);
    out[j++] = hi < 10 ? '0' + hi : 'A' + hi - 10;
    out[j++] = lo < 10 ? '0' + lo : 'A' + lo - 10;
  }
  out[j] = '\0';
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
  return true;
}

// ============================================================================
// CAMBIO DE CLAVE — SUB-FSM NO-BLOQUEANTE
// ============================================================================
/**
 * @brief Inicia el sub-estado de cambio de clave obligatorio para un usuario.
 * @param idx Índice del usuario en el arreglo usuarios[].
 */
void iniciarCambioClave(int idx) {
  cambioClaveIdx    = idx;
  nuevaClaveLen     = 0;
  nuevaClave[0]     = '\0';
  cambioClaveOK     = false;
  cambioClaveMostro = false;
  tCambioClave      = millis();
  cambiarEstado(CAMBIO_CLAVE);
}

/**
 * @brief Ejecuta el estado CAMBIO_CLAVE: solicita nueva clave no-bloqueante,
 *        verifica no reutilización y guarda en EEPROM al confirmar.
 * @details Timeout de T_CAMBIO_CLAVE (30 s). Si vence sin completar
 *          el cambio, vuelve a INICIO sin abrir la puerta (es cambio obligatorio).
 */
void estadoCambioClave() {
  // esperandoTimeout reemplaza el delay al expirar el timeout de 30 s.
  // Patrón idéntico al esperandoOK ya existente: muestra mensaje en LCD,
  // arma tMsgTimeoutClave, y en el siguiente ciclo espera T_MSG_ERROR ms
  // antes de transicionar → CERO bloqueo de loop().
  if (esperandoTimeout) {
    if (millis() - tMsgTimeoutClave >= T_MSG_ERROR) {
      esperandoTimeout = false;
      cambiarEstado(INICIO);
    }
    return;
  }

  if (!cambioClaveOK && (millis() - tCambioClave >= T_CAMBIO_CLAVE)) {
    lcd.clear();
    lcd.print(F("Tiempo agotado  "));
    lcd.setCursor(0, 1);
    lcd.print(F("Vuelve a INICIO "));
    esperandoTimeout   = true;
    tMsgTimeoutClave   = millis();
    return;
  }

  if (!cambioClaveMostro) {
    lcd.clear();
    lcd.print(F("CKEY ! Obligator"));
    lcd.setCursor(0, 1);
    lcd.print(F("Nueva(4-8): "));
    cambioClaveMostro = true;
    nuevaClaveLen = 0;
    nuevaClave[0] = '\0';
  }

  if (cambioClaveOK && esperandoOK && (millis() - tMsgError >= 1500UL)) {
    esperandoOK = false;
    accesoOK  = false;
    cambiarEstado(MONITOR_AMBIENTAL);
    return;
  }

  char key = keypad.getKey();
  if (!key) return;

  if (key == '#') {
    if (nuevaClaveLen < 4) return;
    bool mismaActual   = (strncmp(nuevaClave, usuarios[cambioClaveIdx].clave,    USR_CLAVE_LEN) == 0);
    bool mismaAnterior = (strncmp(nuevaClave, usuarios[cambioClaveIdx].claveAnt, USR_CLAVE_LEN) == 0);
    if (mismaActual || mismaAnterior) {
      lcd.clear();
      lcd.print(F("Clave repetida!"));
      lcd.setCursor(0,1);
      lcd.print(F("Elige otra.     "));
      nuevaClaveLen     = 0;
      nuevaClave[0]     = '\0';
      cambioClaveMostro = false;
      return;
    }
    strncpy(usuarios[cambioClaveIdx].claveAnt, usuarios[cambioClaveIdx].clave, USR_CLAVE_LEN);
    strncpy(usuarios[cambioClaveIdx].clave, nuevaClave, USR_CLAVE_LEN);
    usuarios[cambioClaveIdx].usosClave = 0;
    eepromGuardarUsuario(cambioClaveIdx);
    lcd.clear();
    lcd.print(F("Clave cambiada! "));
    lcd.setCursor(0, 1);
    lcd.print(F("Bienvenido/a    "));
    abrirCerradura();
    setColor(false, true, false);
    tone(BUZZER_PIN, 1200, 80);
    cambioClaveOK = true;
    tMsgError     = millis();
    esperandoOK   = true;
    return;
  }

  if (key == '*') {
    nuevaClaveLen = 0;
    nuevaClave[0] = '\0';
    lcd.setCursor(0, 1);
    lcd.print(F("                "));
    lcd.setCursor(0, 1);
    return;
  }

  if (key >= '0' && key <= '9' && nuevaClaveLen < 8) {
    nuevaClave[nuevaClaveLen++] = key;
    nuevaClave[nuevaClaveLen]   = '\0';
    lcd.setCursor(12, 1);
    for (uint8_t i = 0; i < nuevaClaveLen; i++) lcd.print('*');
  }
}

// ============================================================================
// CAMBIO DE ESTADO
// ============================================================================
/**
 * @brief Cambia el estado actual de la FSM, limpia variables de transición.
 * @details Al ir a INICIO: cierra sesión (usuarioActivo = nullptr), limpia tags,
 *          restablece banderas de display y reinicia timers de LED.
 * @param nuevo El nuevo estado al que se transiciona.
 */
void cambiarEstado(Estado nuevo) {
  debugTransicion(estadoActual, nuevo);
  estadoAnterior = estadoActual;
  estadoActual   = nuevo;
  tEstado        = millis();

  if (nuevo == INICIO) {
    usuarioActivo = nullptr;
  }

  if (nuevo != MONITOR_PUERTAS) {
    deteccionesPuerta = 0;
    tDeteccionesReset = 0;
  }

  tagLeido[0]        = '\0';
  tagBloqueo[0]      = '\0';
  configAuthTag[0]   = '\0';
  configAuthTagOK    = false;
  configAuthPendiente= false;
  tConfigAuth        = 0;
  configAuthError    = false;
  tConfigAuthError   = 0;
  bloqueoAuthPendiente = false;

  inicioMostro        = false;
  configMostro        = false;
  ambMostro           = false;
  puertasMostro       = false;
  alarmaMostro        = false;
  bloqueoMostro       = false;
  gestionMostro       = false;
  gestionScrollActivo = false;
  cambioClaveMostro   = false;
  mostrandoError      = false;
  esperandoOK         = false;
  esperandoTimeout    = false;
  gestInputState      = GEST_IDLE;

  // reaparezca al volver a CONFIG desde otro estado
  regState    = REG_IDLE;
  regBufLen   = 0;
  regBuf[0]   = '\0';
  regClaveLen = 0;
  regTag[0]   = '\0';
  regNombre[0]= '\0';
  regClave[0] = '\0';

  tLed  = millis();
  ledOn = false;
  if (nuevo != MONITOR_PUERTAS) {
    cicloAmbSinAlarma = false;
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(9600);

  SPI.begin();
  mfrc522.PCD_Init();
  // Sin módulo RTC — el reloj interno se configura desde GESTIÓN (tecla 5).

  dht.begin();
  servo.attach(SERVO_PIN);
  servo.write(90);

  pinMode(RELAY_PIN,     OUTPUT);
  pinMode(RED_PIN,       OUTPUT);
  pinMode(GREEN_PIN,     OUTPUT);
  pinMode(BLUE_PIN,      OUTPUT);
  pinMode(BUZZER_PIN,    OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  setColorOff();

  pinMode(HALL_PIN,      INPUT);
  pinMode(BOTON_PIN,     INPUT_PULLUP);
  pinMode(FUEGO_INT_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(BOTON_PIN),     ISR_Boton, FALLING);
  attachInterrupt(digitalPinToInterrupt(FUEGO_INT_PIN), ISR_Fuego, FALLING);

  lcd.begin(16, 2);
  lcd.clear();
  lcd.print(F(" SistemaSeguri. "));
  lcd.setCursor(0, 1);
  lcd.print(F(" v22 H.Manual "));

  eepromCargar();

  lcd.clear();
  cambiarEstado(INICIO);
}

// ============================================================================
// LOOP PRINCIPAL
// ============================================================================
void loop() {
  actualizarRelojInterno();

  // cada T_SENSOR_GLOBAL (500ms). Debe ir ANTES de verificarEmergenciaFuego y
  // verificarPMVAlarma para que ambas usen siempre datos frescos.
  if (millis() - tSensorGlobal >= T_SENSOR_GLOBAL) {
    actualizarSensoresGlobal();
    tSensorGlobal = millis();
    debugSensores();
    debugEstadoActual();
  }

  // Verificar emergencias globalmente antes de cualquier estado
  if (verificarEmergenciaFuego()) return;
  if (verificarPMVAlarma())       return;

  bool botonLocal = false;
  noInterrupts();
  botonLocal = flagBoton;
  interrupts();

  // Botón activa flujo de auth CONFIG desde INICIO
  if (botonLocal) {
    noInterrupts(); flagBoton = false; interrupts();
    if (estadoActual == INICIO && !configAuthPendiente) {
      configAuthPendiente = true;
      configAuthTagOK     = false;
      configAuthTag[0]    = '\0';
      tConfigAuth         = millis();
      resetClave();
      lcd.clear();
      lcd.print(F("CONFIG: Tag Ger."));
      lcd.setCursor(0, 1);
      lcd.print(F("*=cancel  12s   "));
    }
  }

  gestionarServoCierre();

  // ── mostrar error CONFIG_AUTH con flag propio antes de volver a INICIO ──
  if (configAuthError) {
    if (millis() - tConfigAuthError >= T_MSG_ERROR) {
      configAuthError = false;
      inicioMostro    = false;  // forzar redibujado de INICIO limpio
    }
    return;  // no procesar más hasta que expire el mensaje
  }  // ── flujo CONFIG_AUTH con timeout, tecla única por ciclo ──
  if (configAuthPendiente && estadoActual == INICIO) {

    char key = keypad.getKey();

    // Cancelación explícita con '*'
    if (key == '*') {
      configAuthPendiente = false;
      configAuthTagOK     = false;
      configAuthTag[0]    = '\0';
      resetClave();
      inicioMostro = false;
      return;
    }

    if (millis() - tConfigAuth >= T_CONFIG_AUTH) {
      configAuthPendiente = false;
      configAuthTagOK     = false;
      configAuthTag[0]    = '\0';
      resetClave();
      lcd.clear();
      lcd.print(F("CONFIG: timeout "));
      lcd.setCursor(0, 1);
      lcd.print(F("Vuelve a INICIO "));
      configAuthError    = true;
      tConfigAuthError   = millis();
      inicioMostro       = false;
      return;
    }

    // Paso 1: leer tag de Gerente
    if (!configAuthTagOK) {
      char tagBuf[USR_TAG_LEN];
      if (leerTag(tagBuf, sizeof(tagBuf))) {
        bool esGerente = false;
        for (int i = 0; i < totalUsuarios; i++) {
          if (strncmp(usuarios[i].tag, tagBuf, USR_TAG_LEN) == 0 &&
              usuarios[i].rol == ROL_GERENTE) {
            esGerente = true;
            break;
          }
        }
        if (esGerente) {
          strncpy(configAuthTag, tagBuf, USR_TAG_LEN - 1);
          configAuthTag[USR_TAG_LEN-1] = '\0';
          configAuthTagOK = true;
          resetClave();
          lcd.clear();
          lcd.print(F("CONFIG: Tag OK  "));
          lcd.setCursor(0, 1);
          lcd.print(F("Ingrese clave:  "));
          tone(BUZZER_PIN, 1000, 100);
          tConfigAuth = millis();  // rearmar timeout para paso 2
        } else {
          lcd.clear();
          lcd.print(F("Solo Gerente    "));
          lcd.setCursor(0, 1);
          lcd.print(F("CONFIG denegado "));
          tone(BUZZER_PIN, 400, 300);
          configAuthPendiente = false;
          configAuthError     = true;
          tConfigAuthError    = millis();
          inicioMostro        = false;
        }
      }
      return;
    }

    // Paso 2: clave de Gerente — pasar tecla ya leída
    procesarTeclaClaveConKey(key, true);
    if (claveConfirmada) {
      bool claveOK = false;
      for (int i = 0; i < totalUsuarios; i++) {
        if (strncmp(usuarios[i].tag, configAuthTag, USR_TAG_LEN) == 0 &&
            strncmp(claveIngresada, usuarios[i].clave, USR_CLAVE_LEN) == 0 &&
            usuarios[i].rol == ROL_GERENTE) {
          claveOK = true;
          usuarioActivo = &usuarios[i];
          break;
        }
      }
      resetClave();
      configAuthPendiente = false;
      if (claveOK) {
        cambiarEstado(CONFIG);
        return;
      } else {
        lcd.clear();
        lcd.print(F("Clave incorrecta"));
        lcd.setCursor(0, 1);
        lcd.print(F("CONFIG denegado "));
        tone(BUZZER_PIN, 400, 400);
        configAuthTagOK     = false;
        configAuthTag[0]    = '\0';
        configAuthError     = true;
        tConfigAuthError    = millis();
        inicioMostro        = false;
      }
    }
    return;
  }  // ── flujo re-auth Gerente desde BLOQUEO → CONFIG ──
  if (bloqueoAuthPendiente && estadoActual == BLOQUEO) {
    char key = keypad.getKey();

    // Cancelación con '*'
    if (key == '*') {
      bloqueoAuthPendiente = false;
      bloqueoMostro        = true;
      tEstado              = millis();
      return;
    }
    // Timeout si no se autentica en 12 s
    if (millis() - tBloqueoAuth >= T_CONFIG_AUTH) {
      bloqueoAuthPendiente = false;
      bloqueoMostro        = true;
      tEstado              = millis();
      return;
    }

    if (!configAuthTagOK) {
      char tagBuf[USR_TAG_LEN];
      if (leerTag(tagBuf, sizeof(tagBuf))) {
        bool esGerente = false;
        for (int i = 0; i < totalUsuarios; i++) {
          if (strncmp(usuarios[i].tag, tagBuf, USR_TAG_LEN) == 0 &&
              usuarios[i].rol == ROL_GERENTE) {
            esGerente = true;
            break;
          }
        }
        if (esGerente) {
          strncpy(configAuthTag, tagBuf, USR_TAG_LEN - 1);
          configAuthTag[USR_TAG_LEN-1] = '\0';
          configAuthTagOK = true;
          resetClave();
          lcd.clear();
          lcd.print(F("Gerente OK      "));
          lcd.setCursor(0, 1);
          lcd.print(F("Clave CONFIG:   "));
          tone(BUZZER_PIN, 1000, 100);
          tBloqueoAuth = millis();
        } else {
          lcd.clear();
          lcd.print(F("Solo Gerente    "));
          lcd.setCursor(0, 1);
          lcd.print(F("CONFIG denegado "));
          tone(BUZZER_PIN, 400, 300);
          bloqueoAuthPendiente = false;
          configAuthTag[0]     = '\0';
          configAuthTagOK      = false;
          bloqueoMostro        = true;
          tEstado              = millis();
        }
      }
      return;
    }

    procesarTeclaClaveConKey(key, true);
    if (claveConfirmada) {
      bool claveOK = false;
      for (int i = 0; i < totalUsuarios; i++) {
        if (strncmp(usuarios[i].tag, configAuthTag, USR_TAG_LEN) == 0 &&
            strncmp(claveIngresada, usuarios[i].clave, USR_CLAVE_LEN) == 0 &&
            usuarios[i].rol == ROL_GERENTE) {
          claveOK = true;
          usuarioActivo = &usuarios[i];
          break;
        }
      }
      resetClave();
      bloqueoAuthPendiente = false;
      configAuthTag[0]     = '\0';
      configAuthTagOK      = false;
      if (claveOK) {
        intentosFallidos = 0;
        alarmasConsec    = 0;
        cambiarEstado(CONFIG);
      } else {
        lcd.clear();
        lcd.print(F("Clave incorrecta"));
        lcd.setCursor(0, 1);
        lcd.print(F("BLOQUEO activo  "));
        tone(BUZZER_PIN, 400, 400);
        bloqueoMostro = true;
        tEstado       = millis();
      }
    }
    return;
  }

  switch (estadoActual) {
    case INICIO:            estadoInicio();           break;
    case CONFIG:            estadoConfig();           break;
    case MONITOR_AMBIENTAL: estadoMonitorAmbiental(); break;
    case MONITOR_PUERTAS:   estadoMonitorPuertas();   break;
    case ALARMA:            estadoAlarma();           break;
    case BLOQUEO:           estadoBloqueo();          break;
    case GESTION:           estadoGestion();          break;
    case CAMBIO_CLAVE:      estadoCambioClave();      break;
  }
}

// ============================================================================
// ESTADO: INICIO
// ============================================================================
/**
 * @brief Gestiona el estado INICIO: muestra IDLE con RTC, espera RFID + clave.
 * @details Autenticación de 2 factores: tag RFID obligatorio + clave de 4-8 dígitos.
 *          Tras 3 intentos fallidos, transiciona a BLOQUEO. Acceso correcto abre la
 *          cerradura y transiciona a MONITOR_AMBIENTAL después de T_ACCESO_OK ms.
 */
void estadoInicio() {
  if (!inicioMostro) {
    lcd.clear();
    lcd.print(F("IDLE "));
    imprimirHoraInternaLCD();
    lcd.print(F("       "));
    lcd.setCursor(0, 1);
    lcd.print(F("RFID + Clave    "));
    setColor(false, false, true);
    resetClave();
    accesoOK       = false;
    mostrandoError = false;
    esperandoOK    = false;
    inicioMostro   = true;
  }

  if (mostrandoError) {
    if (millis() - tMsgError >= T_MSG_ERROR) {
      mostrandoError = false;
      if (intentosFallidos >= 3) {
        strncpy(causaAlarma, "INTENTOS", sizeof(causaAlarma)-1);
        causaAlarma[sizeof(causaAlarma)-1] = '\0';
        cambiarEstado(BLOQUEO);
        return;
      }
      inicioMostro = false;
    }
    return;
  }

  if (accesoOK) {
    if (millis() - tAccesoOK >= T_ACCESO_OK) {
      cambiarEstado(MONITOR_AMBIENTAL);
    }
    return;
  }

  char tagBuf[USR_TAG_LEN];
  if (leerTag(tagBuf, sizeof(tagBuf))) {
    strncpy(tagLeido, tagBuf, USR_TAG_LEN - 1);
    tagLeido[USR_TAG_LEN - 1] = '\0';
    bool encontrado = false;
    for (int i = 0; i < totalUsuarios; i++) {
      if (strncmp(usuarios[i].tag, tagLeido, USR_TAG_LEN) == 0) {
        encontrado = true;
        lcd.clear();
        lcd.print(F("Hola, "));
        lcd.print(usuarios[i].nombre);
        lcd.setCursor(0, 1);
        lcd.print(F("Clave (4-8):    "));
        resetClave();
        tone(BUZZER_PIN, 1000, 100);
        break;
      }
    }
    if (!encontrado) {
      lcd.clear();
      lcd.print(F("Tag desconocido "));
      lcd.setCursor(0, 1);
      lcd.print(F("                "));
      tagLeido[0] = '\0';
      tone(BUZZER_PIN, 400, 300);
    }
    return;
  }

  procesarTeclaClave(true);

  if (claveConfirmada) {
    bool autorizado = false;
    int  idxUsuario = -1;

    for (int i = 0; i < totalUsuarios; i++) {
      bool tagOK;
      if (tagLeido[0] == '\0') {
        tagOK = false;
      } else {
        tagOK = (strncmp(usuarios[i].tag, tagLeido, USR_TAG_LEN) == 0);
      }
      if (tagOK && strncmp(claveIngresada, usuarios[i].clave, USR_CLAVE_LEN) == 0) {
        idxUsuario = i;
        autorizado = true;
        break;
      }
    }

    resetClave();
    tagLeido[0] = '\0';

    if (autorizado) {
      if (!horarioPermitido(usuarios[idxUsuario])) {
        lcd.clear();
        lcd.print(F("Fuera de turno  "));
        lcd.setCursor(0, 1);
        lcd.print(NOMBRE_ROL[usuarios[idxUsuario].rol]);
        tone(BUZZER_PIN, 400, 500);
        mostrandoError = true;
        tMsgError = millis();
        return;
      }

      usuarioActivo = &usuarios[idxUsuario];
      intentosFallidos = 0;
      alarmasConsec    = 0;

      if (personasPresentes < 20) personasPresentes++;

      usuarios[idxUsuario].usosClave++;
      // usosClave vive en RAM durante la sesión. Solo se graba cuando llega a 4
      // (cambio obligatorio de clave) o cuando se registra un usuario nuevo.

      if (usuarios[idxUsuario].usosClave >= 4) {
        setColor(true, true, false);
        lcd.clear();
        lcd.print(F("Cambio clave    "));
        lcd.setCursor(0, 1);
        lcd.print(F("obligatorio     "));
        tone(BUZZER_PIN, 800, 200);
        iniciarCambioClave(idxUsuario);
        return;
      }

      setColor(false, true, false);
      lcd.clear();
      lcd.print(F("Bienvenido/a    "));
      lcd.setCursor(0, 1);
      lcd.print(usuarioActivo->nombre);
      tone(BUZZER_PIN, 1200, 80);

      abrirCerradura();
      accesoOK  = true;
      tAccesoOK = millis();

    } else {
      intentosFallidos++;
      setColor(true, false, false);
      lcd.clear();
      lcd.print(F("ERR: Incorrecto "));
      lcd.setCursor(0, 1);
      lcd.print(F("Intentos: "));
      lcd.print(intentosFallidos);
      lcd.print(F("/3      "));
      tone(BUZZER_PIN, 400, 400);
      mostrandoError = true;
      tMsgError = millis();
    }
  }
}

// ============================================================================
// ESTADO: CONFIG
// ============================================================================
/**
 * @brief Gestiona el estado CONFIG: permite registrar y ver usuarios (solo Gerente).
 * @details Valida rol Gerente al entrar. Sub-FSM de registro no-bloqueante.
 *          Timeout de T_CONFIG ms sin actividad vuelve a INICIO.
 */
void estadoConfig() {
  if (!usuarioActivo || usuarioActivo->rol != ROL_GERENTE) {
    lcd.clear();
    lcd.print(F("CONFIG denegado "));
    lcd.setCursor(0, 1);
    lcd.print(F("Req. Gerente    "));
    cambiarEstado(INICIO);
    return;
  }

  if (!configMostro) {
    lcd.clear();
    lcd.print(F("CONF "));
    imprimirHoraInternaLCD();
    lcd.print(F("       "));
    lcd.setCursor(0, 1);
    lcd.print(F("1=Reg  2=Ver usr"));
    setColor(false, true, true);
    tConfig      = millis();
    configMostro = true;
    gestionScrollActivo = false;
  }

  if (gestionScrollActivo) {
    if (millis() - tGestionScroll >= 2000UL) {
      gestionPagUsr++;
      if (gestionPagUsr >= totalUsuarios) {
        gestionScrollActivo = false;
        configMostro = false;
        tConfig = millis();
      } else {
        lcd.clear();
        lcd.print(usuarios[gestionPagUsr].nombre);
        lcd.setCursor(0, 1);
        lcd.print(NOMBRE_ROL[usuarios[gestionPagUsr].rol]);
        lcd.print(F(" "));
        lcd.print(usuarios[gestionPagUsr].horaIni);
        lcd.print(F("-"));
        lcd.print(usuarios[gestionPagUsr].horaFin);
        lcd.print(F("h     "));
        tGestionScroll = millis();
      }
    }
    return;
  }

  // T_CONFIG no debe interrumpir un flujo de registro a mitad del camino.
  if (regState != REG_IDLE) {
    procesarRegistroUsuario();
    return;
  }

  if (millis() - tConfig >= T_CONFIG) {
    cambiarEstado(INICIO);
    return;
  }

  char key = keypad.getKey();
  if (!key) return;

  if (key == '#') { cambiarEstado(INICIO); return; }

  if (key == '1') {
    if (totalUsuarios >= MAX_USUARIOS) {
      lcd.clear();
      lcd.print(F("Limite usr lleno"));
      tone(BUZZER_PIN, 400, 300);
      tConfig = millis();
      configMostro = false;
    } else {
      regState   = REG_RFID;
      regBufLen  = 0;
      regBuf[0]  = '\0';
      regTag[0]  = '\0';
      regNombre[0] = '\0';
      regClave[0]  = '\0';
      regClaveLen  = 0;
      regRol       = 0;
      regHoraIni   = 0;
      regHoraFin   = 23;
      tRegRFID   = millis();
      tRegPaso   = millis();
      lcd.clear();
      lcd.print(F("Acerque tag RFID"));
      lcd.setCursor(0, 1);
      lcd.print(F("(10s timeout)   "));
    }
  }

  if (key == '2') {
    if (totalUsuarios == 0) {
      lcd.clear();
      lcd.print(F("Sin usuarios    "));
      tConfig = millis();
      configMostro = false;
    } else {
      gestionPagUsr = 0;
      gestionScrollActivo = true;
      tGestionScroll = millis();
      lcd.clear();
      lcd.print(usuarios[0].nombre);
      lcd.setCursor(0, 1);
      lcd.print(NOMBRE_ROL[usuarios[0].rol]);
      lcd.print(F(" "));
      lcd.print(usuarios[0].horaIni);
      lcd.print(F("-"));
      lcd.print(usuarios[0].horaFin);
      lcd.print(F("h     "));
    }
  }
}

/**
 * @brief Sub-FSM de registro de usuario. Gestiona todos los pasos del flujo
 *        (RFID → Nombre → Clave → Rol → HoraIni → HoraFin → DONE) de forma no-bloqueante.
 */
// ── helper interno: abortar registro con mensaje ──
static inline void abortarRegistro(const __FlashStringHelper* msg) {
  lcd.clear();
  lcd.print(msg);
  lcd.setCursor(0, 1);
  lcd.print(F("Registro cancel."));
  regState    = REG_IDLE;
  regBufLen   = 0;
  regBuf[0]   = '\0';
  regClaveLen = 0;
  configMostro = false;
  tConfig      = millis();
}

void procesarRegistroUsuario() {
  switch (regState) {
    case REG_RFID: {
      if (millis() - tRegRFID >= 10000UL) {
        abortarRegistro(F("Timeout RFID.   "));
        return;
      }
      char tagBuf[USR_TAG_LEN];
      if (leerTag(tagBuf, sizeof(tagBuf))) {
        for (int i = 0; i < totalUsuarios; i++) {
          if (strncmp(usuarios[i].tag, tagBuf, USR_TAG_LEN) == 0) {
            abortarRegistro(F("Tag ya existe!  "));
            return;
          }
        }
        strncpy(regTag, tagBuf, USR_TAG_LEN - 1);
        regTag[USR_TAG_LEN-1] = '\0';
        regState  = REG_NOMBRE;
        regBufLen = 0;
        regBuf[0] = '\0';
        tRegPaso  = millis();
        lcd.clear();
        lcd.print(F("Nombre(# conf): "));
        lcd.setCursor(0, 1);
      }
      break;
    }

    case REG_NOMBRE: {
      if (millis() - tRegPaso >= T_REG_PASO) { abortarRegistro(F("Timeout nombre. ")); return; }
      char key = keypad.getKey();
      if (!key) return;
      tRegPaso = millis();  // reiniciar timeout en cada tecla
      if (key == '#') {
        if (regBufLen == 0) {
          snprintf(regNombre, USR_NOMBRE_LEN, "User%d", totalUsuarios + 1);
        } else {
          strncpy(regNombre, regBuf, USR_NOMBRE_LEN - 1);
          regNombre[USR_NOMBRE_LEN-1] = '\0';
        }
        regState  = REG_CLAVE;
        regBufLen = 0;
        regBuf[0] = '\0';
        tRegPaso  = millis();
        lcd.clear();
        lcd.print(F("Clave(4-8,#ok): "));
        lcd.setCursor(0, 1);
      } else if (key == '*') {
        regBufLen = 0; regBuf[0] = '\0';
        lcd.setCursor(0, 1); lcd.print(F("                ")); lcd.setCursor(0, 1);
      } else if (regBufLen < USR_NOMBRE_LEN - 1) {
        regBuf[regBufLen++] = key;
        regBuf[regBufLen]   = '\0';
        lcd.print(key);
      }
      break;
    }

    case REG_CLAVE: {
      if (millis() - tRegPaso >= T_REG_PASO) { abortarRegistro(F("Timeout clave.  ")); return; }
      char key = keypad.getKey();
      if (!key) return;
      tRegPaso = millis();
      if (key == '#') {
        if (regClaveLen >= 4) {
          regClave[regClaveLen] = '\0';
          regState  = REG_ROL;
          regBufLen = 0;
          regBuf[0] = '\0';
          tRegPaso  = millis();
          lcd.clear();
          lcd.print(F("Rol:0=Op 1=Seg  "));
          lcd.setCursor(0, 1);
          lcd.print(F("2=Coord 3=Ger:  "));
        }
      } else if (key == '*') {
        regClaveLen = 0; regClave[0] = '\0';
        lcd.setCursor(0, 1); lcd.print(F("                ")); lcd.setCursor(0, 1);
      } else if (key >= '0' && key <= '9' && regClaveLen < 8) {
        regClave[regClaveLen++] = key;
        lcd.print('*');
      }
      break;
    }

    case REG_ROL: {
      if (millis() - tRegPaso >= T_REG_PASO) { abortarRegistro(F("Timeout rol.    ")); return; }
      char key = keypad.getKey();
      if (!key) return;
      tRegPaso = millis();
      if (key >= '0' && key <= '3') {
        regRol    = key - '0';
        regState  = REG_HORA_INI;
        regBufLen = 0;
        regBuf[0] = '\0';
        tRegPaso  = millis();
        lcd.clear();
        lcd.print(F("Hora inicio(#ok)"));
        lcd.setCursor(0, 1);
      }
      break;
    }

    case REG_HORA_INI: {
      if (millis() - tRegPaso >= T_REG_PASO) { abortarRegistro(F("Timeout hora ini")); return; }
      char key = keypad.getKey();
      if (!key) return;
      tRegPaso = millis();
      if (key == '#') {
        regBuf[regBufLen] = '\0';
        int v = (regBufLen > 0) ? atoi(regBuf) : 0;
        regHoraIni = (uint8_t)constrain(v, 0, 23);
        regState  = REG_HORA_FIN;
        regBufLen = 0;
        regBuf[0] = '\0';
        tRegPaso  = millis();
        lcd.clear();
        lcd.print(F("Hora fin (#ok): "));
        lcd.setCursor(0, 1);
      } else if (key >= '0' && key <= '9' && regBufLen < 2) {
        regBuf[regBufLen++] = key;
        lcd.print(key);
      }
      break;
    }

    case REG_HORA_FIN: {
      if (millis() - tRegPaso >= T_REG_PASO) { abortarRegistro(F("Timeout hora fin")); return; }
      char key = keypad.getKey();
      if (!key) return;
      tRegPaso = millis();
      if (key == '#') {
        regBuf[regBufLen] = '\0';
        int v = (regBufLen > 0) ? atoi(regBuf) : 23;
        regHoraFin = (uint8_t)constrain(v, 0, 23);
        regState   = REG_DONE;
      } else if (key >= '0' && key <= '9' && regBufLen < 2) {
        regBuf[regBufLen++] = key;
        lcd.print(key);
      }
      break;
    }

    case REG_DONE: {
      int i = totalUsuarios;
      strncpy(usuarios[i].nombre,   regNombre, USR_NOMBRE_LEN - 1);
      usuarios[i].nombre[USR_NOMBRE_LEN-1] = '\0';
      strncpy(usuarios[i].tag,      regTag,    USR_TAG_LEN - 1);
      usuarios[i].tag[USR_TAG_LEN-1] = '\0';
      strncpy(usuarios[i].clave,    regClave,  USR_CLAVE_LEN - 1);
      usuarios[i].clave[USR_CLAVE_LEN-1] = '\0';
      usuarios[i].claveAnt[0] = '\0';
      usuarios[i].rol        = (Rol)regRol;
      usuarios[i].horaIni    = regHoraIni;
      usuarios[i].horaFin    = regHoraFin;
      usuarios[i].usosClave  = 0;
      totalUsuarios++;
      eepromGuardarUsuario(i);
      EEPROM.write(EEPROM_N_USR_ADDR, totalUsuarios);

      lcd.clear();
      lcd.print(F("Usuario guardado"));
      lcd.setCursor(0, 1);
      lcd.print(regNombre);
      tone(BUZZER_PIN, 1200, 80);

      regState     = REG_IDLE;
      configMostro = false;
      tConfig      = millis();
      break;
    }

    default: break;
  }
}

// ============================================================================
// RELAY DE CONFORT
// ============================================================================
/**
 * @brief Actualiza el relay y el LED RGB según el PMV actual y el umbral del usuario.
 * @details PMV > +0.5 → relay ON (AC). PMV < -0.7 → relay OFF (calefacción).
 *          Zona neutra → decide por temperatura fusionada vs umbral del rol ajustado
 *          por ocupación (-0.3 °C por cada 5 personas adicionales).
 */
void actualizarRelayConfort() {
  float umbralUsuario = UMBRAL_TEMP;
  if (usuarioActivo) umbralUsuario = TEMP_CONFORT[usuarioActivo->rol];

  int gruposPersonas = min(personasPresentes / 5, 4);
  umbralUsuario -= gruposPersonas * 0.3f;

  if (pmvActual > 0.5f) {
    digitalWrite(RELAY_PIN, HIGH);
    setColor(true, false, false);
  } else if (pmvActual < -0.7f) {
    digitalWrite(RELAY_PIN, LOW);
    setColor(false, false, true);
  } else {
    if (tempDHT > umbralUsuario) {
      digitalWrite(RELAY_PIN, HIGH);
      setColor(true, true, false);
    } else {
      digitalWrite(RELAY_PIN, LOW);
      setColor(false, true, false);
    }
  }
}

// ============================================================================
// ESTADO: MONITOR_AMBIENTAL
// ============================================================================
/**
 * @brief Gestiona el estado MONITOR_AMBIENTAL: lee sensores, calcula PMV y controla relay.
 * @details Lee DHT11 + LM35 con fusión de temperatura (70/30). Calcula PMV con ajuste
 *          de ocupación. Verifica alarmas (fuego, temp+luz, PMV alto) en cada ciclo.
 *          Tras T_MONITOR_AMB ms sin alarma, transiciona a MONITOR_PUERTAS.
 */
void estadoMonitorAmbiental() {
  if (!ambMostro) {
    // es la única fuente de verdad para tempDHT, pmvActual, luzLDR, valorFuego.
    tAmbiental = millis();
    ambMostro  = true;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("AMB T:"));
    lcd.print(tempDHT, 1);
    lcd.print(F("C"));
    if (fuegoDetectado()) {
      lcd.print(F(" FUEGO"));
    } else {
      lcd.print(F("      "));
    }
    lcd.setCursor(0, 1);
    lcd.print(F("H:"));
    lcd.print((int)humedad);
    lcd.print(F("% L:"));
    lcd.print(luzLDR);
    lcd.print(F(" P:"));
    if (pmvActual >= 0) lcd.print(F("+"));
    lcd.print(pmvActual, 1);

    actualizarRelayConfort();
  }

  // Alarma TEMP+LUZ en tiempo real (fuego y PMV ya manejados globalmente en loop())
  if (tempDHT > UMBRAL_TEMP && luzLDR > UMBRAL_LUZ) {
    strncpy(causaAlarma, "TEMP+LUZ", sizeof(causaAlarma)-1);
    causaAlarma[sizeof(causaAlarma)-1] = '\0';
    cambiarEstado(ALARMA);
    return;
  }

  // Actualizar LCD cuando actualizarSensoresGlobal() refresca los valores
  // Usamos tUltimaLecturaAmbiental como señal de que hay datos nuevos
  static unsigned long tUltimoDisplayAmb = 0;
  if (pmvValido && tUltimaLecturaAmbiental != tUltimoDisplayAmb) {
    tUltimoDisplayAmb = tUltimaLecturaAmbiental;

    actualizarRelayConfort();  // relay ajustado con dato fresco

    lcd.setCursor(6, 0);
    lcd.print(tempDHT, 1);
    lcd.print(F("C "));
    lcd.setCursor(2, 1);
    lcd.print((int)humedad);
    lcd.print(F("% L:"));
    lcd.print(luzLDR);
    lcd.print(F(" P:"));
    if (pmvActual >= 0) lcd.print(F("+"));
    lcd.print(pmvActual, 1);
    lcd.print(F("  "));
  }

  if (millis() - tAmbiental >= T_MONITOR_AMB) {
    cicloAmbSinAlarma = true;
    cambiarEstado(MONITOR_PUERTAS);
  }
}

// ============================================================================
// ESTADO: MONITOR_PUERTAS
// ============================================================================
/**
 * @brief Gestiona el estado MONITOR_PUERTAS: detecta aperturas combinadas Hall + Mic.
 * @details Cuenta eventos confirmados (flanco Hall + Mic en ventana 500 ms).
 *          3 eventos → GESTION. Evento sostenido > T_EVENTO_ALARMA → ALARMA.
 *          Tras T_MONITOR_PUERTAS ms, regresa a MONITOR_AMBIENTAL.
 */
void estadoMonitorPuertas() {
  static unsigned long tVentanaMic    = 0;
  static bool          ventanaMicActiva = false;

  if (!puertasMostro) {
    lcd.clear();
    lcd.print(F("DOOR H:- M:-   "));
    lcd.setCursor(0, 1);
    lcd.print(F("Det:"));
    lcd.print(deteccionesPuerta);
    lcd.print(F("/3          "));
    setColor(false, true, false);
    tPuertas      = millis();
    if (estadoAnterior != MONITOR_AMBIENTAL) {
      enEvento     = false;
      tEvento      = 0;
      hallAnterior = false;
    }
    ventanaMicActiva = false;
    tVentanaMic      = 0;
    puertasMostro = true;
  }

  char keyPuertas = keypad.getKey();
  if (keyPuertas == '*') {
    cambiarEstado(GESTION);
    return;
  }

  // Hall: debounce con T_HALL_FILTRO_MS (80ms) → detecta flanco de apertura.
  // Mic: T_MIC_FILTRO_MS (150ms) como antirruido — gap mínimo entre muestras que cuentan.
  // Ventana de combinación Hall+Mic: 500ms fija (T_VENTANA_HALL_MIC) — NO T_MIC_FILTRO_MS.
  // Evento sostenido → ALARMA sigue usando T_EVENTO_ALARMA (1800ms) sin cambios.
  const unsigned long T_VENTANA_HALL_MIC = 500UL;

  int  hallVal  = analogRead(HALL_PIN);
  int  micVal   = analogRead(MIC_PIN);
  bool hallAhora = (hallVal > HALL_UMBRAL);
  bool micAhora  = (micVal  > MIC_UMBRAL);

  lcd.setCursor(5, 0);
  lcd.print(hallAhora ? "1" : "0");
  lcd.setCursor(9, 0);
  lcd.print(micAhora  ? "1" : "0");

  if (!hallAhora && !micAhora) {
    if (tDeteccionesReset == 0) tDeteccionesReset = millis();
    if (deteccionesPuerta > 0 && (millis() - tDeteccionesReset >= T_DET_RESET)) {
      deteccionesPuerta = 0;
      tDeteccionesReset = 0;
      lcd.setCursor(4, 1);
      lcd.print(F("0"));
    }
  } else {
    tDeteccionesReset = 0;
  }

  // Variables estáticas movidas al inicio de la función e inicializadas al entrar

  // Flanco de subida del Hall → abrir ventana de 500 ms para esperar mic
  if (hallAhora && !hallAnterior) {
    if (millis() - tDebounceHall >= T_HALL_FILTRO_MS) {  // debounce Hall
      tDebounceHall    = millis();
      ventanaMicActiva = true;
      tVentanaMic      = millis();
      micConfirmado    = false;    // v19: reset confirmación en cada apertura
    }
  }
  hallAnterior = hallAhora;

  if (ventanaMicActiva) {
    if (micAhora && !micConfirmado &&
        (millis() - tMicUltimaLectura >= T_MIC_FILTRO_MS)) {
      tMicUltimaLectura = millis();
      micConfirmado     = true;    // mic confirmado en esta ventana

      ventanaMicActiva = false;
      deteccionesPuerta++;
      lcd.setCursor(4, 1);
      lcd.print(deteccionesPuerta);
      lcd.print(F("/3 "));
      tone(BUZZER_PIN, 900, 100);

      if (deteccionesPuerta >= 3) {
        deteccionesPuerta = 0;
        cambiarEstado(GESTION);
        return;
      }
    } else if (millis() - tVentanaMic >= T_VENTANA_HALL_MIC) {  // v19: usa 500ms
      // Ventana expiró sin mic → evento descartado
      ventanaMicActiva = false;
      micConfirmado    = false;
    }
  }

  bool eventoSostenido = (hallAhora || micAhora);
  if (eventoSostenido) {
    if (!enEvento) { enEvento = true; tEvento = millis(); }
    if (millis() - tEvento >= T_EVENTO_ALARMA) {
      enEvento = false; tEvento = 0;
      strncpy(causaAlarma, "PUERTA", sizeof(causaAlarma)-1);
      causaAlarma[sizeof(causaAlarma)-1] = '\0';
      cambiarEstado(ALARMA);
      return;
    }
  } else {
    enEvento = false;
    tEvento  = 0;
  }

  if (millis() - tPuertas >= T_MONITOR_PUERTAS) {
    if (cicloAmbSinAlarma) {
      alarmasConsec    = 0;
      cicloAmbSinAlarma = false;
    }
    cambiarEstado(MONITOR_AMBIENTAL);
  }
}

// ============================================================================
// ESTADO: ALARMA
// ============================================================================
/**
 * @brief Gestiona el estado ALARMA: activa buzzer, parpadea LED rojo, muestra causa.
 * @details Dura T_ALARMA_DUR ms. Si alarmasConsec >= 3, transiciona a BLOQUEO;
 *          si no, regresa a MONITOR_AMBIENTAL.
 */
void estadoAlarma() {
  if (!alarmaMostro) {
    alarmasConsec++;
    tAlarma      = millis();
    alarmaMostro = true;

    lcd.clear();
    lcd.print(F("ALR "));
    lcd.print((int)(T_ALARMA_DUR / 1000));
    lcd.print(F("s ["));
    lcd.print(alarmasConsec);
    lcd.print(F("/3]     "));
    lcd.setCursor(0, 1);
    lcd.print(F("Causa:"));
    lcd.print(causaAlarma);
    lcd.print(F("        "));

    ledOn = true;
    tLed  = millis();
    setColor(true, false, false);
    tone(BUZZER_PIN, 1500);
  }

  parpadeaLED(true, false, false, LED_ALARMA_ON, LED_ALARMA_OFF);
  // tone() ya iniciado en línea de entrada; no se interrumpe con el LED

  unsigned long transcurrido = millis() - tAlarma;
  if (transcurrido < T_ALARMA_DUR) {
    unsigned long restante = (T_ALARMA_DUR - transcurrido) / 1000UL;
    lcd.setCursor(0, 0);
    lcd.print(F("ALR "));
    if (restante < 10) lcd.print(F("0"));
    lcd.print(restante);
    lcd.print(F("s ["));
    lcd.print(alarmasConsec);
    lcd.print(F("/3]     "));
  }

  if (transcurrido >= T_ALARMA_DUR) {
    noTone(BUZZER_PIN);
    setColorOff();
    causaAlarma[0] = '\0';

    if (alarmasConsec >= 3) {
      cambiarEstado(BLOQUEO);
    } else {
      cambiarEstado(MONITOR_AMBIENTAL);
    }
  }
}

// ============================================================================
// ESTADO: BLOQUEO
// ============================================================================
/**
 * @brief Gestiona el estado BLOQUEO: sistema bloqueado por alarmas consecutivas.
 * @details Desbloqueo manual: RFID (Seguridad o Gerente) + clave → INICIO.
 *          (usuario está ingresando su clave), evitando interrumpir el proceso.
 *          (RFID + Clave) antes de ir a CONFIG. No va directo a CONFIG.
 */
void estadoBloqueo() {
  if (!bloqueoMostro) {
    lcd.clear();
    lcd.print(F("LOCK !!! "));
    lcd.print(causaAlarma[0] ? causaAlarma : "       ");
    lcd.setCursor(0, 1);
    lcd.print(F("Tag+Clave Seg/Ge"));
    noTone(BUZZER_PIN);
    bloqueoMostro = true;
    ledOn = false;
    tLed  = millis();
    resetClave();
  }

  parpadeaLED(true, false, false, LED_BLOQUEO_ON, LED_BLOQUEO_OFF);

  // (tagBloqueo[0] != '\0' significa que el usuario ya pasó RFID y está en clave)
  bool tagEnProceso = (tagBloqueo[0] != '\0');

  if (!tagEnProceso && !bloqueoAuthPendiente && !esperandoOK && !mostrandoError &&
      (millis() - tEstado >= T_BLOQUEO_AUTO)) {
    intentosFallidos = 0;
    alarmasConsec    = 0;
    lcd.clear();
    lcd.print(F("CONFIG: Ger req."));
    lcd.setCursor(0, 1);
    lcd.print(F("RFID+Clave      "));
    tone(BUZZER_PIN, 800, 200);
    bloqueoAuthPendiente = true;
    tBloqueoAuth         = millis();
    configAuthTag[0]     = '\0';
    configAuthTagOK      = false;
    resetClave();
    return;
  }

  if (mostrandoError && (millis() - tMsgError >= 800UL)) {
    mostrandoError = false;
    bloqueoMostro  = false;
  }

  // Paso 1: leer RFID si aún no hay tag
  if (tagBloqueo[0] == '\0') {
    char tagBuf[USR_TAG_LEN];
    if (leerTag(tagBuf, sizeof(tagBuf))) {
      bool tagValido = false;
      for (int i = 0; i < totalUsuarios; i++) {
        if (strncmp(usuarios[i].tag, tagBuf, USR_TAG_LEN) == 0 &&
            (usuarios[i].rol == ROL_SEGURIDAD || usuarios[i].rol == ROL_GERENTE)) {
          tagValido = true;
          break;
        }
      }
      if (tagValido) {
        strncpy(tagBloqueo, tagBuf, USR_TAG_LEN - 1);
        tagBloqueo[USR_TAG_LEN - 1] = '\0';
        tEstado = millis();
        lcd.setCursor(0, 1);
        lcd.print(F("Tag OK. Clave:  "));
        tone(BUZZER_PIN, 1000, 100);
      } else {
        lcd.setCursor(0, 1);
        lcd.print(F("Tag no autoriza."));
        tone(BUZZER_PIN, 400, 300);
      }
    }
    return;
  }

  // Paso 2: con tag válido, procesar clave
  procesarTeclaClave(false);
  if (claveConfirmada) {
    bool desbloqueado = false;
    for (int i = 0; i < totalUsuarios; i++) {
      if (strncmp(usuarios[i].tag, tagBloqueo, USR_TAG_LEN) == 0 &&
          strncmp(claveIngresada, usuarios[i].clave, USR_CLAVE_LEN) == 0 &&
          (usuarios[i].rol == ROL_SEGURIDAD || usuarios[i].rol == ROL_GERENTE)) {
        desbloqueado = true;
        resetClave();
        tagBloqueo[0]    = '\0';
        intentosFallidos = 0;
        alarmasConsec    = 0;
        // (indica fin del incidente; el conteo puede no ser exacto tras un bloqueo)
        lcd.clear();
        lcd.print(F("Desbloqueado    "));
        lcd.setCursor(0, 1);
        lcd.print(usuarios[i].nombre);
        esperandoOK = true;
        tMsgError   = millis();
        break;
      }
    }
    if (!desbloqueado) {
      resetClave();
      tagBloqueo[0] = '\0';
      lcd.setCursor(0, 1);
      lcd.print(F("Clave inv. Retry"));
    }
  }

  if (esperandoOK && (millis() - tMsgError >= T_MSG_ERROR)) {
    esperandoOK = false;
    cambiarEstado(INICIO);
  }
}

// ============================================================================
// ESTADO: GESTIÓN
// ============================================================================
/**
 * @brief Gestiona el estado GESTIÓN: ajuste de umbrales, consulta de usuarios y hora.
 * @details Acceso mínimo: ROL_COORDINADOR. Ajuste de umbrales: solo ROL_GERENTE.
 *          Sub-FSM no-bloqueante para entrada de valores numéricos.
 */
void estadoGestion() {
  if (!usuarioActivo || usuarioActivo->rol < ROL_COORDINADOR) {
    if (!mostrandoError) {
      lcd.clear();
      lcd.print(F("Acceso denegado "));
      lcd.setCursor(0, 1);
      if (!usuarioActivo) {
        lcd.print(F("Sin sesion activ"));
      } else {
        lcd.print(F("Rol insuficiente"));
      }
      mostrandoError = true;
      tMsgError = millis();
    }
    if (millis() - tMsgError >= T_MSG_ERROR) {
      mostrandoError = false;
      cambiarEstado(INICIO);
    }
    return;
  }

  if (!gestionMostro) {
    mostrandoError = false;
    lcd.clear();
    lcd.print(F("GEST [*=salir]  "));
    lcd.setCursor(0, 1);
    lcd.print(F("1=T 2=L 3=U 4=-P"));
    setColor(true, true, false);
    gestionMostro = true;
    gestionScrollActivo = false;
    gestInputState = GEST_IDLE;
  }

  if (mostrandoError) {
    if (millis() - tMsgError >= T_MSG_ERROR) {
      mostrandoError = false;
      gestionMostro  = false;  // redibuja menú limpio
    }
    return;
  }

  if (gestionScrollActivo) {
    if (millis() - tGestionScroll >= 2500UL) {
      gestionPagUsr++;
      if (gestionPagUsr >= totalUsuarios) {
        gestionScrollActivo = false;
        gestionMostro = false;
      } else {
        lcd.clear();
        lcd.print(gestionPagUsr + 1);
        lcd.print(F(":"));
        lcd.print(usuarios[gestionPagUsr].nombre);
        lcd.setCursor(0, 1);
        lcd.print(NOMBRE_ROL[usuarios[gestionPagUsr].rol]);
        lcd.print(F(" "));
        lcd.print(usuarios[gestionPagUsr].horaIni);
        lcd.print(F("-"));
        lcd.print(usuarios[gestionPagUsr].horaFin);
        lcd.print(F("h   "));
        tGestionScroll = millis();
      }
    }
    return;
  }

  if (gestInputState != GEST_IDLE) {
    bool fuegoEmerg = false;
    noInterrupts(); fuegoEmerg = flagFuego; interrupts();
    if (fuegoEmerg) {
      gestInputState = GEST_IDLE;
      gestionMostro  = false;
      return;
    }

    char k = keypad.getKey();
    if (!k) return;

    if (k == '#') {
      if (regBufLen > 0) {
        if (gestInputState == GEST_TEMP) {
          float v = atof(regBuf);
          if (v >= 10.0 && v <= 40.0) {
            UMBRAL_TEMP = v;
            EEPROM.put(EEPROM_UMBRAL_TEMP, UMBRAL_TEMP);
            lcd.clear();
            lcd.print(F("Umbral Temp:    "));
            lcd.setCursor(0, 1);
            lcd.print(UMBRAL_TEMP, 1);
            lcd.print(F("C             "));
          } else {
            lcd.clear();
            lcd.print(F("Valor inv.      "));
            lcd.setCursor(0, 1);
            lcd.print(F("Rango: 10-40 C  "));
          }
          gestInputState = GEST_IDLE;
          gestionMostro  = false;
          return;
        } else if (gestInputState == GEST_LUZ) {
          int v = atoi(regBuf);
          if (v >= 50 && v <= 900) {
            UMBRAL_LUZ = v;
            EEPROM.put(EEPROM_UMBRAL_LUZ, UMBRAL_LUZ);
            lcd.clear();
            lcd.print(F("Umbral Luz:     "));
            lcd.setCursor(0, 1);
            lcd.print(UMBRAL_LUZ);
            lcd.print(F(" ADC          "));
          } else {
            lcd.clear();
            lcd.print(F("Valor inv.      "));
            lcd.setCursor(0, 1);
            lcd.print(F("Rango: 50-900   "));
          }
          gestInputState = GEST_IDLE;
          gestionMostro  = false;
          return;
        } else if (gestInputState == GEST_HORA) {
          int v = atoi(regBuf);
          if (v >= 0 && v <= 23) {
            horaTempConfig = (uint8_t)v;
            regBufLen = 0;
            regBuf[0] = '\0';
            gestInputState = GEST_MINUTO;
            lcd.clear();
            lcd.print(F("Minuto actual: "));
            lcd.setCursor(0, 1);
            lcd.print(F("00-59 # conf   "));
            lcd.setCursor(0, 1);
          } else {
            lcd.clear();
            lcd.print(F("Hora invalida  "));
            lcd.setCursor(0, 1);
            lcd.print(F("Rango: 00-23   "));
            gestInputState = GEST_IDLE;
            gestionMostro  = false;
          }
          return;
        } else if (gestInputState == GEST_MINUTO) {
          int v = atoi(regBuf);
          if (v >= 0 && v <= 59) {
            horaActual = horaTempConfig;
            minutoActual = (uint8_t)v;
            tRelojInterno = millis();
            relojConfigurado = true;
            lcd.clear();
            lcd.print(F("Hora configurada"));
            lcd.setCursor(0, 1);
            imprimirHoraInternaLCD();
            lcd.print(F("            "));
          } else {
            lcd.clear();
            lcd.print(F("Min invalido    "));
            lcd.setCursor(0, 1);
            lcd.print(F("Rango: 00-59    "));
          }
          gestInputState = GEST_IDLE;
          gestionMostro  = false;
          return;
        }
      }
      return;
    }

    if (k == '*') {
      regBufLen = 0;
      regBuf[0] = '\0';
      lcd.setCursor(0, 1);
      lcd.print(F("                "));
      lcd.setCursor(0, 1);
      return;
    }

    bool esDigito = (k >= '0' && k <= '9');
    bool esPunto  = (k == 'A' && gestInputState == GEST_TEMP && regBufLen < 4);
    uint8_t maxLenEntrada = (gestInputState == GEST_HORA || gestInputState == GEST_MINUTO) ? 2 : 5;
    if ((esDigito || esPunto) && regBufLen < maxLenEntrada) {
      char c = esPunto ? '.' : k;
      regBuf[regBufLen++] = c;
      regBuf[regBufLen]   = '\0';
      lcd.print(c);
    }
    return;
  }

  char key = keypad.getKey();
  if (!key) return;

  if (key == '*') { cambiarEstado(INICIO); return; }

  if (key == '1') {
    if (!usuarioActivo || usuarioActivo->rol < ROL_GERENTE) {
      lcd.clear();
      lcd.print(F("Solo Gerente    "));
      lcd.setCursor(0, 1);
      lcd.print(F("opcion restringd"));
      mostrandoError = true;
      tMsgError = millis();
    } else {
      lcd.clear();
      lcd.print(F("Umbral Temp(C): "));
      lcd.setCursor(0, 1);
      lcd.print(F("A=punto # conf  "));
      lcd.setCursor(0, 1);
      regBufLen = 0; regBuf[0] = '\0';
      gestInputState = GEST_TEMP;
    }
  }

  if (key == '2') {
    if (!usuarioActivo || usuarioActivo->rol < ROL_GERENTE) {
      lcd.clear();
      lcd.print(F("Solo Gerente    "));
      lcd.setCursor(0, 1);
      lcd.print(F("opcion restringd"));
      mostrandoError = true;
      tMsgError = millis();
    } else {
      lcd.clear();
      lcd.print(F("Umbral Luz ADC: "));
      lcd.setCursor(0, 1);
      lcd.print(F("# para confirmar"));
      lcd.setCursor(0, 1);
      regBufLen = 0; regBuf[0] = '\0';
      gestInputState = GEST_LUZ;
    }
  }

  if (key == '3') {
    if (totalUsuarios == 0) {
      lcd.clear();
      lcd.print(F("Sin usuarios    "));
      lcd.setCursor(0, 1);
      lcd.print(F("Registra uno    "));
      mostrandoError = true;
      tMsgError = millis();
    } else {
      gestionPagUsr = 0;
      gestionScrollActivo = true;
      tGestionScroll = millis();
      lcd.clear();
      lcd.print(1); lcd.print(F(":")); lcd.print(usuarios[0].nombre);
      lcd.setCursor(0,1);
      lcd.print(NOMBRE_ROL[usuarios[0].rol]);
      lcd.print(F(" ")); lcd.print(usuarios[0].horaIni);
      lcd.print(F("-")); lcd.print(usuarios[0].horaFin); lcd.print(F("h   "));
    }
  }

  if (key == '4') {
    if (personasPresentes > 0) personasPresentes--;
    lcd.clear();
    lcd.print(F("Personas en sala"));
    lcd.setCursor(0, 1);
    lcd.print(F("Quedan: "));
    lcd.print(personasPresentes);
    lcd.print(F("        "));
    mostrandoError = true;
    tMsgError = millis();
    // gestionMostro se redibuja cuando el timer expira
  }

  if (key == '5') {
    // Sin RTC: configurar hora lógica manual HH:MM para validar horarios de usuarios
    lcd.clear();
    lcd.print(F("Hora actual:    "));
    lcd.setCursor(0, 1);
    lcd.print(F("00-23 # conf   "));
    lcd.setCursor(0, 1);
    regBufLen = 0;
    regBuf[0] = '\0';
    gestInputState = GEST_HORA;
  }
}
