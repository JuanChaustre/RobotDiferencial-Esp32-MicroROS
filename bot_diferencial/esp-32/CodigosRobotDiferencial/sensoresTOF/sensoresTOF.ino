#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// --- NUEVAS DIRECCIONES I2C PARA CADA SENSOR ---
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32

// --- PINES XSHUT PARA APAGAR/ENCENDER CADA SENSOR ---
const int xshut1 = 4;   // Sensor Izquierdo
const int xshut2 = 16;  // Sensor Central
const int xshut3 = 17;  // Sensor Derecho

// Crear objetos para cada sensor
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

// Variables de control de estado y tiempo
bool sensoresListos = false;
unsigned long ultimaLecturaExitosa = 0;
const unsigned long TIMEOUT_LIMITE = 1000; // 1 segundo sin respuesta exitosa fuerza reinicio de config

void inicializarSensores() {
  Serial.println("Iniciando secuencia de encendido de los ToF...");

  // PASO 1: Apagar todos los sensores poniendo XSHUT en LOW
  digitalWrite(xshut1, LOW);
  digitalWrite(xshut2, LOW);
  digitalWrite(xshut3, LOW);
  delay(10);

  // PASO 2: Encender y configurar el SENSOR 1 (Izquierdo)
  digitalWrite(xshut1, HIGH);
  delay(10);
  if (!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("¡Error: No se pudo iniciar el Sensor 1 (Izq)!"));
    sensoresListos = false;
    return;
  }
  Serial.print(F("Sensor 1 activo en 0x"));
  Serial.println(LOX1_ADDRESS, HEX);

  // PASO 3: Encender y configurar el SENSOR 2 (Central)
  digitalWrite(xshut2, HIGH);
  delay(10);
  if (!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("¡Error: No se pudo iniciar el Sensor 2 (Centro)!"));
    sensoresListos = false;
    return;
  }
  Serial.print(F("Sensor 2 activo en 0x"));
  Serial.println(LOX2_ADDRESS, HEX);

  // PASO 4: Encender y configurar el SENSOR 3 (Derecho)
  digitalWrite(xshut3, HIGH);
  delay(10);
  if (!lox3.begin(LOX3_ADDRESS)) {
    Serial.println(F("¡Error: No se pudo iniciar el Sensor 3 (Der)!"));
    sensoresListos = false;
    return;
  }
  Serial.print(F("Sensor 3 activo en 0x"));
  Serial.println(LOX3_ADDRESS, HEX);
  
  Serial.println("¡Todos los sensores listos y configurados!");
  sensoresListos = true;
  ultimaLecturaExitosa = millis(); // Actualizamos el cronómetro al iniciar con éxito
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Configurar pines XSHUT como salidas
  pinMode(xshut1, OUTPUT);
  pinMode(xshut2, OUTPUT);
  pinMode(xshut3, OUTPUT);

  // Inicializar bus I2C (GPIO 21 SDA, GPIO 22 SCL)
  Wire.begin();
  // --- PROTECCIÓN CONTRA CONGELAMIENTO ---
  // Si un sensor se desconecta, el bus no se bloqueará más de 50 milisegundos
  Wire.setTimeOut(50);
  // Bucle de arranque inicial
  while (!sensoresListos) {
    inicializarSensores();
    if (!sensoresListos) {
      Serial.println("Esperando conexión de los sensores ToF...");
      delay(2000); 
    }
  }
}

void loop() {
  // Si los sensores no están listos, intentamos reinicializarlos
  if (!sensoresListos) {
    Serial.println("Intentando recuperar la conexión con los sensores...");
    inicializarSensores();
    delay(1000);
    return; 
  }

  // --- CONTROL DE TIMEOUT ---
  if (millis() - ultimaLecturaExitosa > TIMEOUT_LIMITE) {
    Serial.println("¡Alerta: Timeout detectado en los sensores ToF! Reiniciando configuración...");
    sensoresListos = false; 
    return;
  }

  VL53L0X_RangingMeasurementData_t measure1;
  VL53L0X_RangingMeasurementData_t measure2;
  VL53L0X_RangingMeasurementData_t measure3;

  // Realizar mediciones de los tres sensores
  lox1.rangingTest(&measure1, false);
  lox2.rangingTest(&measure2, false);
  lox3.rangingTest(&measure3, false);

  // Si el código llegó aquí sin bloquearse, actualizamos el temporizador de vida
  ultimaLecturaExitosa = millis();

// --- APLICAR LÍMITE DE 700 mm (70 cm) ---
  int dist1 = (measure1.RangeStatus != 4 && measure1.RangeMilliMeter <= 700) ? measure1.RangeMilliMeter : 0;
  int dist2 = (measure2.RangeStatus != 4 && measure2.RangeMilliMeter <= 700) ? measure2.RangeMilliMeter : 0;
  int dist3 = (measure3.RangeStatus != 4 && measure3.RangeMilliMeter <= 700) ? measure3.RangeMilliMeter : 0;

  // Formato para el Serial Plotter (Nombre:Valor separados por comas)
  Serial.print("Izq:");
  Serial.print(dist1);
  Serial.print(",");
  Serial.print("Centro:");
  Serial.print(dist2);
  Serial.print(",");
  Serial.print("Der:");
  Serial.println(dist3);

  delay(50); // Un pequeño retraso para que el gráfico sea fluido
}