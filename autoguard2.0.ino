/*
 * ==========================================================
 *  AutoGuard: asistencia al estacionamiento y alerta de proximidad
 * ==========================================================
 *  Placa:   Arduino UNO R3
 *  Sensor:  Ultrasónico HC-SR04
 *  Salidas: 3 LED (verde, amarillo, rojo) + buzzer pasivo
 *  Entrada: 1 pulsador para cambiar de modo
 *
 *  MODOS DE FUNCIONAMIENTO
 *   - ACTIVO (estacionar): según la distancia al obstáculo se
 *     enciende un LED distinto y el buzzer suena con más urgencia
 *     a medida que el obstáculo se acerca.
 *   - PASIVO (vigilancia): al entrar al modo se mide una distancia
 *     base (calibración). Si después se detecta algo más cerca que
 *     esa base (menos un margen), se dispara una alarma.
 *
 *  CONEXIONES (mapa de pines)
 *   HC-SR04 : VCC -> 5V | GND -> GND | TRIG -> D9 | ECHO -> D10
 *   LED verde    -> D2 (con resistencia de 220 ohm a GND)
 *   LED amarillo -> D3 (con resistencia de 220 ohm a GND)
 *   LED rojo     -> D4 (con resistencia de 220 ohm a GND)
 *   Buzzer       -> D8 (el otro pin a GND)
 *   Pulsador     -> D7 (el otro pin a GND, usa INPUT_PULLUP)
 */

// ==========================================================
// 1. DECLARACIÓN DE PINES
// ==========================================================
const int PIN_TRIG         = 9;   // Pin que envía el pulso de disparo al sensor
const int PIN_ECHO         = 10;  // Pin que recibe el eco del sensor
const int PIN_LED_VERDE    = 2;   // LED verde: distancia segura
const int PIN_LED_AMARILLO = 3;   // LED amarillo: precaución
const int PIN_LED_ROJO     = 4;   // LED rojo: peligro / alarma
const int PIN_BUZZER       = 8;   // Buzzer pasivo
const int PIN_BOTON        = 7;   // Pulsador de cambio de modo

// ==========================================================
// 2. CONSTANTES DE CONFIGURACIÓN (umbrales y tiempos)
// ==========================================================
// Umbrales del modo activo (en centímetros)
const int ZONA_VERDE    = 30;   // Más de 30 cm: espacio suficiente
const int ZONA_AMARILLA = 15;   // Entre 15 y 30 cm: precaución
const int ZONA_CRITICA  = 6;    // Menos de 6 cm: tono continuo

// Parámetros del modo pasivo
const int DIST_MAX_VIGILANCIA = 40;
const int MARGEN_MINIMO        = 2;   // Margen mínimo en cm para disparar la alarma
const int MARGEN_PORCENTAJE    = 25;  // El margen es el 25 % de la distancia base (si supera al mínimo)
const int LECTURAS_PARA_ALARMA = 2;   // Lecturas seguidas que confirman una detección         // Base por defecto si el sensor no recibe eco
const unsigned long DURACION_ALARMA = 3000; // La alarma sigue sonando 3 s tras la última detección

// Parámetros de medición y botón
const int MUESTRAS                  = 3;      // Mediciones que se promedian por lectura
const unsigned long TIMEOUT_ECHO    = 25000;  // Espera máxima del eco en microsegundos (~4 m)
const unsigned long TIEMPO_REBOTE   = 50;     // Anti-rebote del pulsador en milisegundos
// ==========================================================
// 3. VARIABLES GLOBALES Y TIPOS
// ==========================================================
enum Modo { MODO_ACTIVO, MODO_PASIVO };   // Los dos modos del sistema

Modo modoActual = MODO_ACTIVO;            // Arranca en modo estacionamiento
int distanciaBase = DIST_MAX_VIGILANCIA;  // Distancia de referencia del modo pasivo
int lecturasSeguidas = 0;     
bool alarmaActiva = false;                // true mientras la alarma pasiva está sonando
unsigned long ultimaDeteccion = 0;        // Momento de la última detección (millis)

int ultimoEstadoBoton = HIGH;             // Estado anterior del pulsador (HIGH = sin apretar)
unsigned long ultimoCambioBoton = 0;      // Momento del último cambio válido del pulsador

// ==========================================================
// 4. SETUP: se ejecuta una sola vez al encender
// ==========================================================
void setup() {
  Serial.begin(9600);                     // Monitor serie para ver distancias y modo

  pinMode(PIN_TRIG, OUTPUT);              // TRIG envía el pulso
  pinMode(PIN_ECHO, INPUT);               // ECHO recibe la respuesta

  pinMode(PIN_LED_VERDE, OUTPUT);         // LED como salidas
  pinMode(PIN_LED_AMARILLO, OUTPUT);
  pinMode(PIN_LED_ROJO, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);            // Buzzer como salida

  pinMode(PIN_BOTON, INPUT_PULLUP);       // Pulsador con resistencia interna: apretado = LOW

  apagarSalidas();                        // Empezamos con todo apagado
  Serial.println("AutoGuard iniciado - Modo ACTIVO");
}

// ==========================================================
// 5. LOOP: lógica principal, se repite continuamente
// ==========================================================
void loop() {
  // 5.1 Revisar si se apretó el pulsador para cambiar de modo
  if (botonPresionado()) {
    cambiarModo();
  }

  // 5.2 Medir la distancia (promedio de varias lecturas; -1 si no hay eco)
  int distancia = medirDistancia(MUESTRAS);

  // 5.3 Mostrar información por el monitor serie
  Serial.print(modoActual == MODO_ACTIVO ? "[ACTIVO]  " : "[PASIVO]  ");
  Serial.print("Distancia: ");
  if (distancia == -1) {
    Serial.println("sin eco");
  } else {
    Serial.print(distancia);
    Serial.println(" cm");
  }

  // 5.4 Ejecutar la lógica del modo en el que estamos
  if (modoActual == MODO_ACTIVO) {
    ejecutarModoActivo(distancia);
  } else {
    ejecutarModoPasivo(distancia);
  }
}

// ==========================================================
// 6. FUNCIONES DE MEDICIÓN
// ==========================================================

// Hace UNA medición con el HC-SR04. Devuelve cm o -1 si no recibió eco.
int medirUnaVez() {
  digitalWrite(PIN_TRIG, LOW);            // Aseguramos que TRIG arranca en bajo
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);           // Pulso de 10 microsegundos para disparar
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  // Mide cuánto tarda en volver el eco (con tiempo máximo de espera)
  long duracion = pulseIn(PIN_ECHO, HIGH, TIMEOUT_ECHO);

  if (duracion == 0) {                    // 0 significa que no volvió el eco
    return -1;
  }
  // Velocidad del sonido ~ 0,0343 cm/us; se divide por 2 por ir y volver
  return (int)(duracion * 0.0343 / 2);
}

// Promedia varias mediciones para reducir el ruido.
// Parámetro: cantidad de muestras. Devuelve cm o -1 si ninguna fue válida.
int medirDistancia(int muestras) {
  long suma = 0;                          // Acumulador de lecturas válidas
  int validas = 0;                        // Cuántas lecturas fueron válidas

  for (int i = 0; i < muestras; i++) {
    int lectura = medirUnaVez();
    if (lectura != -1) {                  // Solo promediamos lecturas con eco
      suma += lectura;
      validas++;
    }
    delay(25);                            // Pausa entre disparos para evitar ecos cruzados
  }

  if (validas == 0) {
    return -1;                            // Ninguna lectura válida
  }
  return suma / validas;                  // Promedio
}

// ==========================================================
// 7. FUNCIONES DE LOS MODOS
// ==========================================================

// MODO ACTIVO: asistencia al estacionamiento según la distancia.
void ejecutarModoActivo(int distancia) {
  if (distancia == -1 || distancia > ZONA_VERDE) {
    // Sin obstáculo cercano: LED verde y buzzer en silencio
    actualizarLeds(true, false, false);
    noTone(PIN_BUZZER);
  }
  else if (distancia > ZONA_AMARILLA) {
    // Precaución: LED amarillo y bip lento
    actualizarLeds(false, true, false);
    emitirAlerta(1000, 500);
  }
  else if (distancia > ZONA_CRITICA) {
    // Peligro: LED rojo y bip rápido
    actualizarLeds(false, false, true);
    emitirAlerta(1500, 150);
  }
  else {
    // Muy cerca: LED rojo y tono continuo (intervalo 0 = sin pausas)
    actualizarLeds(false, false, true);
    emitirAlerta(2000, 0);
  }
}

// MODO PASIVO: vigilancia con alarma si algo se acerca más que la distancia base.
void ejecutarModoPasivo(int distancia) {
  // El margen se adapta a la distancia base: 25 % de la base, con un mínimo de 2 cm
  int margen = distanciaBase * MARGEN_PORCENTAJE / 100;
  if (margen < MARGEN_MINIMO) {
    margen = MARGEN_MINIMO;
  }

  // Hay un objeto "nuevo" si se midió algo más cerca que la base menos el margen
  bool objetoCerca = (distancia != -1 && distancia < distanciaBase - margen);

  // Contamos lecturas seguidas para evitar falsas alarmas por ruido del sensor
  if (objetoCerca) {
    lecturasSeguidas++;
  } else {
    lecturasSeguidas = 0;
  }

  if (lecturasSeguidas >= LECTURAS_PARA_ALARMA) {
    alarmaActiva = true;                  // Se dispara la alarma
    ultimaDeteccion = millis();           // Guardamos cuándo fue
  }

  // La alarma se apaga sola si pasó el tiempo sin nuevas detecciones
  if (alarmaActiva && (millis() - ultimaDeteccion > DURACION_ALARMA)) {
    alarmaActiva = false;
  }

  if (alarmaActiva) {
    // Alarma: LED rojo y bips rápidos de tono agudo
    actualizarLeds(false, false, true);
    emitirAlerta(1800, 200);
  } else {
    // Vigilando: LED verde parpadeando lento
    actualizarLeds(false, false, false);
    digitalWrite(PIN_LED_VERDE, (millis() / 500) % 2);
    noTone(PIN_BUZZER);
  }
}

// Cambia entre modo activo y pasivo.
void cambiarModo() {
  apagarSalidas();

  if (modoActual == MODO_ACTIVO) {
    modoActual = MODO_PASIVO;
    Serial.println(">>> Cambio a modo PASIVO");
    calibrarModoPasivo();
  } else {
    modoActual = MODO_ACTIVO;
    Serial.println(">>> Cambio a modo ACTIVO");
  }
}

// Mide la distancia base del modo pasivo. LED amarillo = calibrando.
void calibrarModoPasivo() {
  actualizarLeds(false, true, false);     // Amarillo: calibrando
  Serial.println("Calibrando... alejate del sensor");
  delay(1500);                            // Tiempo para alejarse de la zona vigilada

  int base = medirDistancia(5);           // Más muestras para una base estable
  if (base == -1 || base > DIST_MAX_VIGILANCIA) {
    base = DIST_MAX_VIGILANCIA;           // Sin eco o muy lejos: usamos el valor por defecto
  }
  distanciaBase = base;
  alarmaActiva = false;

  Serial.print("Distancia base: ");
  Serial.print(distanciaBase);
  Serial.println(" cm");

  tone(PIN_BUZZER, 1200, 200);            // Bip corto de confirmación
  delay(250);
  actualizarLeds(false, false, false);
}

// ==========================================================
// 8. FUNCIONES AUXILIARES (salidas y entrada)
// ==========================================================

// Enciende o apaga cada LED según los parámetros recibidos.
void actualizarLeds(bool verde, bool amarillo, bool rojo) {
  digitalWrite(PIN_LED_VERDE, verde ? HIGH : LOW);
  digitalWrite(PIN_LED_AMARILLO, amarillo ? HIGH : LOW);
  digitalWrite(PIN_LED_ROJO, rojo ? HIGH : LOW);
}

// Hace sonar el buzzer sin bloquear el programa (usa millis en vez de delay).
// Parámetros: frecuencia en Hz e intervalo entre bips en ms.
// Si el intervalo es 0, el tono suena de forma continua.
void emitirAlerta(int frecuencia, int intervalo) {
  static unsigned long ultimoCambio = 0;  // Cuándo cambió por última vez el estado del bip
  static bool sonando = false;            // Si el buzzer está sonando en este momento

  if (intervalo == 0) {                   // Tono continuo
    tone(PIN_BUZZER, frecuencia);
    return;
  }

  if (millis() - ultimoCambio >= (unsigned long)intervalo) {
    ultimoCambio = millis();
    sonando = !sonando;                   // Alternamos entre sonar y silencio
    if (sonando) {
      tone(PIN_BUZZER, frecuencia);
    } else {
      noTone(PIN_BUZZER);
    }
  }
}

// Apaga todos los LED y el buzzer.
void apagarSalidas() {
  actualizarLeds(false, false, false);
  noTone(PIN_BUZZER);
}

// Detecta UNA pulsación del botón (con anti-rebote). Devuelve true al apretar.
bool botonPresionado() {
  int lectura = digitalRead(PIN_BOTON);
  bool presionado = false;

  // Solo consideramos el cambio si pasó el tiempo anti-rebote
  if (lectura != ultimoEstadoBoton && (millis() - ultimoCambioBoton > TIEMPO_REBOTE)) {
    ultimoCambioBoton = millis();
    if (lectura == LOW) {                 // LOW = botón apretado (por INPUT_PULLUP)
      presionado = true;
    }
    ultimoEstadoBoton = lectura;
  }
  return presionado;
}