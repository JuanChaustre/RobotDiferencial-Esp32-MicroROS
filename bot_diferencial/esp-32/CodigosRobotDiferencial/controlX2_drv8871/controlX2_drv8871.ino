/************************************************************
Control PID Doble con Encoders + OLED 128x32 para ESP32
*************************************************************/
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- CONFIGURACIÓN DE LA PANTALLA OLED ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1 // Reset pin # (-1 si comparte el pin de reset del ESP32)
#define SCREEN_ADDRESS 0x3C // Dirección I2C típica (puede ser 0x3D)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- PINES DEL DRV8871 (MOTOR IZQUIERDO) ---
const int pinIN1_Izq = 26; 
const int pinIN2_Izq = 25; 
const int ledChannel1 = 0;  
const int ledChannel2 = 1;  

// --- PINES DEL DRV8871 (MOTOR DERECHO) ---
const int pinIN1_Der = 18; 
const int pinIN2_Der = 19; 
const int ledChannel3 = 2;  
const int ledChannel4 = 3;  

const int freq = 30000;    
const int resolution = 8;  

// --- PINES Y CONFIGURACIÓN DE LOS ENCODERS ---
const int pinEncoderA_Izq = 34; 
const int pinEncoderB_Izq = 35; 

const int pinEncoderA_Der = 32; 
const int pinEncoderB_Der = 33; 

volatile long contadorPulsosIzq = 0;
volatile long contadorPulsosDer = 0;

unsigned long tiempoAnteriorEncoder = 0;

// RPMs y Límites
float inputRPM_Izq = 0.0;
float inputRPM_Der = 0.0;
const float minRPM = 35.0;

float pprEncoder = 11.0;          
float reductionRatio = 30.0;     

// --- VARIABLES DEL PID Y SETPOINTS (INDEPENDIENTES) ---
float setpointRPM_Izq = 0.0;   
float setpointRPM_Der = 0.0;   

float outputPWM_Izq = 0.0;      
float outputPWM_Der = 0.0;      

// Constantes PID Motor Izquierdo
float Kp_Izq = 2.0;
float Ki_Izq = 1.2;
float Kd_Izq = 0.02;

// Constantes PID Motor Derecho
float Kp_Der = 2.0;
float Ki_Der = 1.2;
float Kd_Der = 0.02;

// Variables de error Motor Izquierdo
float error_Izq = 0, errorAnterior_Izq = 0, integral_Izq = 0, derivada_Izq = 0;
static int contadorCruce_Izq = 0;

// Variables de error Motor Derecho
float error_Der = 0, errorAnterior_Der = 0, integral_Der = 0, derivada_Der = 0;
static int contadorCruce_Der = 0;

unsigned long tiempoAnteriorPID = 0;
const long intervaloPID = 50; 

unsigned long tiempoAnteriorImpresion = 0;
const long intervaloImpresion = 500;

unsigned long tiempoAnteriorOLED = 0;
const long intervaloOLED = 200; // Refresco de pantalla cada 200ms

// --- ISRs (INTERRUPCIONES DE LOS ENCODERS) ---
void IRAM_ATTR leerEncoderIzq() {
  if (digitalRead(pinEncoderB_Izq) == HIGH) {
    contadorPulsosIzq++;  
  } else {
    contadorPulsosIzq--;  
  }
}

void IRAM_ATTR leerEncoderDer() {
  if (digitalRead(pinEncoderB_Der) == LOW) {
    contadorPulsosDer++;  
  } else {
    contadorPulsosDer--;  
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Inicializar Pantalla OLED 128x32
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("Fallo al encontrar la pantalla OLED SSD1306"));
    for(;;); // Bloquear si falla la pantalla
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Iniciando Robot..."));
  display.display();
  delay(1000);

  // Configurar canales PWM para Motor Izquierdo
  ledcSetup(ledChannel1, freq, resolution);
  ledcSetup(ledChannel2, freq, resolution);
  ledcAttachPin(pinIN1_Izq, ledChannel1); 
  ledcAttachPin(pinIN2_Izq, ledChannel2); 

  // Configurar canales PWM para Motor Derecho
  ledcSetup(ledChannel3, freq, resolution);
  ledcSetup(ledChannel4, freq, resolution);
  ledcAttachPin(pinIN1_Der, ledChannel3); 
  ledcAttachPin(pinIN2_Der, ledChannel4); 

  // Configurar Encoders
  pinMode(pinEncoderA_Izq, INPUT_PULLUP);
  pinMode(pinEncoderB_Izq, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinEncoderA_Izq), leerEncoderIzq, RISING);

  pinMode(pinEncoderA_Der, INPUT_PULLUP);
  pinMode(pinEncoderB_Der, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinEncoderA_Der), leerEncoderDer, RISING);

  Serial.println("\n--- SISTEMA PID DOBLE + OLED (TRACCIÓN DIFERENCIAL) ---");
  imprimirMenu();
}

void loop() {
  unsigned long tiempoActual = millis();

  // --- 1. LEER COMANDOS DESDE EL MONITOR SERIAL ---
  if (Serial.available() > 0) {
    char comando = Serial.read();
    float valor = Serial.parseFloat();

    if (comando == 'a' || comando == 'A') {
      setpointRPM_Izq = valor;
      Serial.print("-> Izquierdo Setpoint: "); Serial.print(setpointRPM_Izq); Serial.println(" RPM");
    } 
    else if (comando == 'd' || comando == 'D') {
      setpointRPM_Der = valor*0.95;
      Serial.print("-> Derecho Setpoint: "); Serial.print(setpointRPM_Der); Serial.println(" RPM");
    }
    else if (comando == 's' || comando == 'S') {
      setpointRPM_Izq = valor;
      setpointRPM_Der = valor*0.95;
      Serial.print("-> AMBOS Setpoint establecidos a: "); Serial.print(valor); Serial.println(" RPM");
    }

  }

  // --- 2. CÁLCULO DE LA VELOCIDAD REAL (RPM) DE AMBOS MOTORES ---
  if (tiempoActual - tiempoAnteriorEncoder >= 100) { 
    unsigned long dtEncoder = tiempoActual - tiempoAnteriorEncoder;
    tiempoAnteriorEncoder = tiempoActual;

    noInterrupts();
    long pulsosT_Izq = contadorPulsosIzq;
    contadorPulsosIzq = 0;
    long pulsosT_Der = contadorPulsosDer;
    contadorPulsosDer = 0;
    interrupts();

    float pulsosTotalesRueda = pprEncoder * reductionRatio;
    float minutos = dtEncoder / 60000.0;

    inputRPM_Izq = ((float)pulsosT_Izq / pulsosTotalesRueda) / minutos;
    inputRPM_Der = ((float)pulsosT_Der / pulsosTotalesRueda) / minutos;
  }

  // --- 3. CÁLCULO DE LOS LAZOS PID (CADA 50ms) ---
  if (tiempoActual - tiempoAnteriorPID >= intervaloPID) {
    float dt = (tiempoActual - tiempoAnteriorPID) / 1000.0;
    tiempoAnteriorPID = tiempoActual;

    // ==========================================
    // MOTOR IZQUIERDO
    // ==========================================
    error_Izq = setpointRPM_Izq - inputRPM_Izq;

    if (abs(setpointRPM_Izq) < minRPM) { 
      integral_Izq = 0;
      errorAnterior_Izq = 0;
      outputPWM_Izq = 0;
      ledcWrite(ledChannel1, 0);
      ledcWrite(ledChannel2, 0);
    } 
    else {
      bool cambioSentido_Izq = (setpointRPM_Izq > 40.0 && inputRPM_Izq < -10.0) || (setpointRPM_Izq < -40.0 && inputRPM_Izq > 10.0);
      if (cambioSentido_Izq) {
        contadorCruce_Izq++;
        if (contadorCruce_Izq >= 3) {
          integral_Izq = 0;       
          errorAnterior_Izq = 0;
          contadorCruce_Izq = 0;
        }
      } else {
        contadorCruce_Izq = 0;
      }

      integral_Izq += error_Izq * dt;
      float limiteIntegral_Izq = 255.0 / Ki_Izq; 
      if (integral_Izq > limiteIntegral_Izq) integral_Izq = limiteIntegral_Izq;
      if (integral_Izq < -limiteIntegral_Izq) integral_Izq = -limiteIntegral_Izq;

      derivada_Izq = (error_Izq - errorAnterior_Izq) / dt;
      outputPWM_Izq = (Kp_Izq * error_Izq) + (Ki_Izq * integral_Izq) + (Kd_Izq * derivada_Izq);
      
      if (outputPWM_Izq > 255) outputPWM_Izq = 255;
      if (outputPWM_Izq < -255) outputPWM_Izq = -255;

      if (outputPWM_Izq >= 0) {
        ledcWrite(ledChannel1, (int)outputPWM_Izq);
        ledcWrite(ledChannel2, 0);
      } else {
        ledcWrite(ledChannel1, 0);
        ledcWrite(ledChannel2, (int)(-outputPWM_Izq)); 
      }
      errorAnterior_Izq = error_Izq;
    }

    // ==========================================
    // MOTOR DERECHO
    // ==========================================
    error_Der = setpointRPM_Der - inputRPM_Der;

    if (abs(setpointRPM_Der) < minRPM) { 
      integral_Der = 0;
      errorAnterior_Der = 0;
      outputPWM_Der = 0;
      ledcWrite(ledChannel3, 0);
      ledcWrite(ledChannel4, 0);
    } 
    else {
      bool cambioSentido_Der = (setpointRPM_Der > 40.0 && inputRPM_Der < -10.0) || (setpointRPM_Der < -40.0 && inputRPM_Der > 10.0);
      if (cambioSentido_Der) {
        contadorCruce_Der++;
        if (contadorCruce_Der >= 3) {
          integral_Der = 0;       
          errorAnterior_Der = 0;
          contadorCruce_Der = 0;
        }
      } else {
        contadorCruce_Der = 0;
      }

      integral_Der += error_Der * dt;
      float limiteIntegral_Der = 255.0 / Ki_Der; 
      if (integral_Der > limiteIntegral_Der) integral_Der = limiteIntegral_Der;
      if (integral_Der < -limiteIntegral_Der) integral_Der = -limiteIntegral_Der;

      derivada_Der = (error_Der - errorAnterior_Der) / dt;
      outputPWM_Der = (Kp_Der * error_Der) + (Ki_Der * integral_Der) + (Kd_Der * derivada_Der);
      
      if (outputPWM_Der > 255) outputPWM_Der = 255;
      if (outputPWM_Der < -255) outputPWM_Der = -255;

      if (outputPWM_Der >= 0) {
        ledcWrite(ledChannel3, (int)outputPWM_Der);
        ledcWrite(ledChannel4, 0);
      } else {
        ledcWrite(ledChannel3, 0);
        ledcWrite(ledChannel4, (int)(-outputPWM_Der)); 
      }
      errorAnterior_Der = error_Der;
    }
  }

  // --- 4. IMPRESIÓN NO BLOQUEANTE PARA EL PLOTTER SERIAL ---
  if (tiempoActual - tiempoAnteriorImpresion >= intervaloImpresion) {
    tiempoAnteriorImpresion = tiempoActual;

    Serial.print("Setpoint_Izq:"); Serial.print(setpointRPM_Izq);
    Serial.print(",RPM_Izq:"); Serial.print(inputRPM_Izq);
    Serial.print(",Setpoint_Der:"); Serial.print(setpointRPM_Der);
    Serial.print(",RPM_Der:"); Serial.print(inputRPM_Der);
    Serial.print(",MinLimit:-300,MaxLimit:300\n");
  }

  // --- 5. ACTUALIZACIÓN DE LA PANTALLA OLED (CADA 200ms) ---
  if (tiempoActual - tiempoAnteriorOLED >= intervaloOLED) {
    tiempoAnteriorOLED = tiempoActual;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // Línea 1: Motor Izquierdo
    display.setCursor(0, 0);
    display.print("L Set:"); 
    display.print(setpointRPM_Izq, 1);
    display.print(" RPM");

    // Línea 2: RPM Real Izquierdo
    display.setCursor(0, 8);
    display.print("L Act:"); 
    display.print(inputRPM_Izq, 1);
    display.print(" RPM");

    // Línea 3: Motor Derecho Setpoint
    display.setCursor(0, 16);
    display.print("R Set:"); 
    display.print(setpointRPM_Der, 1);
    display.print(" RPM");

    // Línea 4: RPM Real Derecho
    display.setCursor(0, 24);
    display.print("R Act:"); 
    display.print(inputRPM_Der, 1);
    display.print(" RPM");

    display.display();
  }
}

void imprimirMenu() {
  Serial.println("Comandos disponibles:");
  Serial.println("  A [valor] -> Cambiar Setpoint de AMBOS motores");
  Serial.println("  L [valor] -> Cambiar Setpoint solo del Motor Izquierdo");
  Serial.println("  D [valor] -> Cambiar Setpoint solo del Motor Derecho");
  Serial.println("--------------------------------------------------\n");
}