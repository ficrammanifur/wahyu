/*
 * Water Quality Monitor ESP32 - Versi Stabil
 * Perbaikan: pH, Turbidity, Serial stability, Flow Sensor
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ==================== PIN DEFINITIONS ====================
#define PH_PIN          32
#define TDS_PIN         33
#define ONE_WIRE_BUS    34
#define TURBIDITY_PIN   35
#define FLOW_PIN        18
#define BUZZER_PIN       2

// LCD
#define LCD_ADDR        0x27
#define LCD_COLS        20
#define LCD_ROWS        4

// ==================== CONSTANTS ====================
const float VREF = 3.3;
const int ADC_RES = 4095;

// pH Calibration (disesuaikan agar lebih akurat)
const float PH_V4 = 1.304;   // pH 4.00
const float PH_V7 = 1.048;   // pH 6.86  
const float PH_V9 = 0.835;   // pH 9.18

// Turbidity Calibration
const int TURB_CLEAR = 4095;   // Nilai ADC di air sangat jernih
const int TURB_DIRTY  = 3031;  // Nilai ADC di air keruh

// Flow
const float ML_PER_PULSE = 1000.0 / 922.0;   // 1.0846 mL/pulse

// Timing
const unsigned long SENSOR_INTERVAL = 800;
const unsigned long LCD_INTERVAL    = 1500;
const unsigned long SERIAL_INTERVAL = 4000;

// ==================== GLOBALS ====================
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

float temperatureC = 25.0;
float phValue = 7.0;
float tdsValue = 0.0;
int turbidityADC = 0;
float turbidityPercent = 0.0;
String turbidityStatus = "MENUNGGU";

// Flow
volatile unsigned long pulseCount = 0;
unsigned long lastPulseCount = 0;
float totalVolumeL = 0.0;

// Buffers
const int AVG_SAMPLES = 15;
float phBuffer[AVG_SAMPLES];
int phIdx = 0;
float phEMA = 1.1;

float tdsBuffer[AVG_SAMPLES];
int tdsIdx = 0;

int turbBuffer[AVG_SAMPLES];
int turbIdx = 0;

// Timing
unsigned long lastSensor = 0;
unsigned long lastLCD = 0;
unsigned long lastSerial = 0;

// ==================== ISR ====================
void IRAM_ATTR flowISR() {
  pulseCount++;
}

// ==================== HELPERS ====================
float getAvg(float* buf, int size) {
  float sum = 0;
  for(int i=0; i<size; i++) sum += buf[i];
  return sum / size;
}

float calculatePH(float voltage) {
  // Linear interpolation 3 titik
  if (voltage >= PH_V7) {
    float slope = (6.86 - 4.00) / (PH_V7 - PH_V4);
    return 4.0 + (voltage - PH_V4) * slope;
  } else {
    float slope = (9.18 - 6.86) / (PH_V9 - PH_V7);
    return 6.86 + (voltage - PH_V7) * slope;
  }
}

float calculateTDS(float voltage, float temp) {
  if (voltage < 0.1) return 0.0;
  float comp = 1.0 + 0.02 * (temp - 25.0);
  float vComp = voltage / comp;
  float tds = (133.42*vComp*vComp*vComp - 255.86*vComp*vComp + 857.39*vComp) * 0.5;
  return constrain(tds, 0, 9999);
}

void calculateTurbidity(int adc) {
  turbidityADC = adc;
  float percent = ((float)(adc - TURB_DIRTY) / (TURB_CLEAR - TURB_DIRTY)) * 100.0;
  turbidityPercent = constrain(percent, 0.0, 100.0);
  
  if (turbidityPercent >= 80) turbidityStatus = "SANGAT JERNIH";
  else if (turbidityPercent >= 50) turbidityStatus = "CUKUP JERNIH";
  else if (turbidityPercent >= 20) turbidityStatus = "AGAK KERUH";
  else turbidityStatus = "KERUH";
}

// ==================== READ FUNCTIONS ====================
void readAllSensors() {
  // Temperature
  sensors.requestTemperatures();
  float t = sensors.getTempCByIndex(0);
  if (t > -50 && t < 100) temperatureC = t;

  // pH
  int adcPH = analogRead(PH_PIN);
  float vPH = (adcPH * VREF) / ADC_RES;
  phBuffer[phIdx] = vPH;
  phIdx = (phIdx + 1) % AVG_SAMPLES;
  float avgPH = getAvg(phBuffer, AVG_SAMPLES);
  phEMA = 0.25 * avgPH + 0.75 * phEMA;
  phValue = calculatePH(phEMA);

  // TDS
  int adcTDS = analogRead(TDS_PIN);
  float vTDS = (adcTDS * VREF) / ADC_RES;
  tdsBuffer[tdsIdx] = vTDS;
  tdsIdx = (tdsIdx + 1) % AVG_SAMPLES;
  tdsValue = calculateTDS(getAvg(tdsBuffer, AVG_SAMPLES), temperatureC);

  // Turbidity
  int adcTurb = analogRead(TURBIDITY_PIN);
  turbBuffer[turbIdx] = adcTurb;
  turbIdx = (turbIdx + 1) % AVG_SAMPLES;
  calculateTurbidity(getAvg(turbBuffer, AVG_SAMPLES));  // wait, fix: use int avg
  // Better avg for int
  int avgTurb = 0;
  for(int i=0; i<AVG_SAMPLES; i++) avgTurb += turbBuffer[i];
  avgTurb /= AVG_SAMPLES;
  calculateTurbidity(avgTurb);

  // Flow
  noInterrupts();
  unsigned long pc = pulseCount;
  interrupts();
  if (pc > lastPulseCount) {
    totalVolumeL += (pc - lastPulseCount) * ML_PER_PULSE / 1000.0;
    lastPulseCount = pc;
  }
}

void updateLCD() {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("T:"); lcd.print(temperatureC,1); lcd.print("C  pH:"); lcd.print(phValue,2);
  lcd.setCursor(0,1); lcd.print("TDS:"); lcd.print((int)tdsValue); lcd.print("ppm");
  lcd.setCursor(0,2); lcd.print("Vol:"); lcd.print(totalVolumeL,2); lcd.print("L");
  lcd.setCursor(0,3); lcd.print("Clr:"); lcd.print((int)turbidityPercent); lcd.print("% "); lcd.print(turbidityStatus.substring(0,8));
}

void printSerial() {
  Serial.println("\n==========================================");
  Serial.printf("Temperature : %.2f C\n", temperatureC);
  Serial.printf("pH          : %.2f\n", phValue);
  Serial.printf("TDS         : %d ppm\n", (int)tdsValue);
  Serial.printf("Turbidity   : %d%% %s\n", (int)turbidityPercent, turbidityStatus.c_str());
  Serial.printf("Volume      : %.3f L\n", totalVolumeL);
  Serial.printf("Total Pulses: %lu\n", pulseCount);
  Serial.println("==========================================");
}

// Buzzer
void beep(int n) {
  for(int i=0; i<n; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(80);
    digitalWrite(BUZZER_PIN, LOW);
    delay(120);
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10);
  
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(FLOW_PIN, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flowISR, FALLING);
  
  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  
  sensors.begin();
  
  // Init buffers
  for(int i=0; i<AVG_SAMPLES; i++) {
    phBuffer[i] = 1.1;
    tdsBuffer[i] = 0.8;
    turbBuffer[i] = 3800;
  }
  
  lcd.setCursor(0,0);
  lcd.print("Water Quality Monitor");
  lcd.setCursor(0,2);
  lcd.print("Starting...");
  
  beep(2);
  delay(1500);
  
  Serial.println("=== SYSTEM STARTED ===");
  Serial.println("Kirim 'r' untuk reset volume");
}

// ==================== LOOP ====================
void loop() {
  unsigned long now = millis();

  if (now - lastSensor >= SENSOR_INTERVAL) {
    lastSensor = now;
    readAllSensors();
  }

  if (now - lastLCD >= LCD_INTERVAL) {
    lastLCD = now;
    updateLCD();
  }

  if (now - lastSerial >= SERIAL_INTERVAL) {
    lastSerial = now;
    printSerial();
  }

  // Serial Command
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') {
      noInterrupts();
      pulseCount = 0;
      lastPulseCount = 0;
      totalVolumeL = 0.0;
      interrupts();
      Serial.println(">>> VOLUME RESET <<<");
    }
  }

  delay(5);
}
