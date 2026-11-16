/************************************************************
MPU6500 FIFO Optimizado + OLED para Robot Móvil (ROS 2 Ready)
*************************************************************/
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6500-DMP.h>

// --- CONFIGURACIÓN DE LA PANTALLA OLED (128x32) ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1 
#define SCREEN_ADDRESS 0x3C 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#if defined(SAMD)
#define SerialPort SerialUSB
#else
#define SerialPort Serial
#endif

MPU6500_DMP imu;

// Variables para la impresión controlada en Serial (cada 500ms)
unsigned long tiempoAnteriorImpresion = 0;
const long intervaloImpresion = 500;

// Variables para el refresco de la OLED (cada 200ms)
unsigned long tiempoAnteriorOLED = 0;
const long intervaloOLED = 200;

// Variables globales para almacenar la última lectura limpia
float accelX = 0, accelY = 0, accelZ = 0;
float gyroZ = 0;
unsigned long timestampIMU = 0;

// Variables de Offsets (Sesgos de calibración)
float gyroZ_offset = 0;
float accelX_offset = 0, accelY_offset = 0, accelZ_offset = 0;

void calibrarIMU(void);
void printIMUDataForROS2(void);
void actualizarOLED(void);

void setup() 
{
  SerialPort.begin(115200);
  delay(2000); 
  SerialPort.println("Iniciando MPU6500 FIFO + OLED para Robot...");

  // Inicializar Pantalla OLED 128x32
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("Fallo al encontrar la pantalla OLED SSD1306"));
    for(;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Iniciando IMU..."));
  display.display();

  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-6500");
      delay(5000);
    }
  }

  SerialPort.println("¡MPU-6500 inicializado con éxito!");

  // Configuración de rangos óptimos para robótica móvil
  imu.setGyroFSR(2000); // 2000 dps
  imu.setAccelFSR(2);   // +/- 2g
  imu.setLPF(5);        // Filtro pasa-bajos de 5Hz

  // Tasa de muestreo de 100Hz
  imu.setSampleRate(100); 

  // CONFIGURACIÓN ESTRICTA PARA 2D: 
  imu.configureFifo(INV_Z_GYRO | INV_XYZ_ACCEL);

  // --- RUTINA DE CALIBRACIÓN INICIAL ---
  calibrarIMU();
}

void loop()
{
  if ( imu.fifoAvailable() >= 256)
  {
    while ( imu.fifoAvailable() > 0)
    {
      if ( imu.updateFifo() == INV_SUCCESS)
      {
        // Obtenemos valores y restamos los offsets calculados
        accelX = imu.calcAccel(imu.ax) - accelX_offset;
        accelY = imu.calcAccel(imu.ay) - accelY_offset;
        accelZ = imu.calcAccel(imu.az) - accelZ_offset;
        
        gyroZ  = imu.calcGyro(imu.gz) - gyroZ_offset;
        
        timestampIMU = imu.time;
      }
    }
  }

  unsigned long tiempoActual = millis();

  // 1. Impresión controlada en Serial cada 500ms
  if (tiempoActual - tiempoAnteriorImpresion >= intervaloImpresion) {
    tiempoAnteriorImpresion = tiempoActual;
    printIMUDataForROS2();
  }

  // 2. Actualización de la pantalla OLED cada 200ms
  if (tiempoActual - tiempoAnteriorOLED >= intervaloOLED) {
    tiempoAnteriorOLED = tiempoActual;
    actualizarOLED();
  }
  
  delay(10); 
}

void calibrarIMU(void) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("CALIBRANDO IMU..."));
  display.setCursor(0, 12);
  display.println(F("Mantenga quieto"));
  display.display();

  SerialPort.println("\n--------------------------------------------------");
  SerialPort.println("CALIBRANDO IMU: Mantén el robot TOTALMENTE QUIETO...");
  SerialPort.println("--------------------------------------------------");
  
  delay(1000); 

  int muestras = 200;
  float sumaGyroZ = 0;
  float sumaAccelX = 0, sumaAccelY = 0, sumaAccelZ = 0;
  int muestrasValidas = 0;

  while (muestrasValidas < muestras) {
    if (imu.fifoAvailable() > 0) {
      if (imu.updateFifo() == INV_SUCCESS) {
        sumaGyroZ  += imu.calcGyro(imu.gz);
        sumaAccelX += imu.calcAccel(imu.ax);
        sumaAccelY += imu.calcAccel(imu.ay);
        sumaAccelZ += imu.calcAccel(imu.az); 
        muestrasValidas++;
      }
    }
  }

  gyroZ_offset  = sumaGyroZ / muestras;
  accelX_offset = sumaAccelX / muestras;
  accelY_offset = sumaAccelY / muestras;
  accelZ_offset = (sumaAccelZ / muestras) - 1.0; // Descontar la gravedad

  SerialPort.println("¡Calibración completada con éxito!");
  SerialPort.println("--------------------------------------------------\n");

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("Calibracion OK!"));
  display.display();
  delay(1000);
}

void actualizarOLED(void) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Línea 1: Título o estado
  display.setCursor(0, 0);
  display.print(F("=== IMU DATA 2D ==="));

  // Línea 2: Velocidad Angular en Z (GyroZ)
  display.setCursor(0, 9);
  display.print(F("GZ: "));
  display.print(gyroZ, 1);
  display.print(F(" dps"));

  // Línea 3: Aceleración X
  display.setCursor(0, 18);
  display.print(F("AcX: "));
  display.print(accelX, 2);
  display.print(F(" g"));

  // Línea 4: Aceleración Y
  display.setCursor(64, 18);
  display.print(F("AcY: "));
  display.print(accelY, 2);
  display.print(F(" g"));

  display.display();
}

void printIMUDataForROS2(void)
{  
  SerialPort.println("--- [sensor_msgs/Imu Frame (2D Optimized)] ---");
  
  SerialPort.print("Angular Velocity (GyroZ) -> [z: ");
  SerialPort.print(gyroZ, 2); Serial.println("] dps");

  SerialPort.print("Linear Acceleration      -> [x: ");
  SerialPort.print(accelX, 2); Serial.print(", y: ");
  SerialPort.print(accelY, 2); Serial.print(", z: ");
  SerialPort.print(accelZ, 2); Serial.println("] g");
  
  SerialPort.print("Timestamp (ms)           -> ");
  SerialPort.println(timestampIMU);
  SerialPort.println("--------------------------------------------------\n");
}