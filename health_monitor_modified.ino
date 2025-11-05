/*
 * ========================================
 * HEALTH MONITORING SYSTEM - FINAL VERSION
 * STM32F401CCU6 Black Pill - Arduino IDE
 * ========================================
 * 
 * FEATURES:
 * 1. Fixed cough detection with proper energy calculation
 * 2. Auto-clearing tremor detection
 * 3. Simplified priority-based display system
 * 4. Active health monitoring with smooth cycling
 */

#include <Wire.h>
#include <SPI.h>
#include <LiquidCrystal.h>

// ========== PIN DEFINITIONS ==========
#define I2C_SDA PB7
#define I2C_SCL PB8
LiquidCrystal lcd(PB6, PB1, PB2, PB3, PB4, PB5);
#define ECG_PIN PA1
#define LO_PLUS_PIN PA2
#define LO_MINUS_PIN PA3
#define MIC_PIN PA0
#define MPU_CS_PIN PA4

// ========== SENSOR ADDRESSES & REGISTERS ==========
#define MLX90614_ADDR 0x5A
#define MLX90614_TA 0x06
#define MLX90614_TOBJ1 0x07
#define MPU6500_WHO_AM_I 0x75
#define MPU6500_PWR_MGMT_1 0x6B
#define MPU6500_ACCEL_XOUT_H 0x3B

// ========== TIMING CONFIGURATION ==========
#define MAIN_LOOP_INTERVAL 50
#define LCD_UPDATE_INTERVAL 1000
#define TEMP_READ_INTERVAL 2000
#define ECG_SAMPLE_INTERVAL 2
#define MIC_SAMPLE_INTERVAL 5
#define MPU_SAMPLE_INTERVAL 20

// ========== DETECTION THRESHOLDS ==========
#define ECG_PEAK_THRESHOLD 150
#define ECG_REFRACTORY_PERIOD 300
#define MAX_BPM 200
#define MIN_BPM 40

// COUGH DETECTION PARAMETERS
#define MIC_BUFFER_SIZE 50                   // 50 samples @ 200Hz = 250ms window
#define CALIBRATION_COUGHS 5                 // Number of coughs for calibration
#define COUGH_MIN_DURATION 50                // Minimum 50ms for valid cough
#define COUGH_MAX_DURATION 1200              // Maximum 1200ms for valid cough
#define COUGH_COOLDOWN_MS 500                // 500ms between coughs
#define RAPID_RISE_THRESHOLD 60              // Rate of change threshold

// Fall Detection
#define FALL_THRESHOLD 2.5
#define FALL_TIME_WINDOW 500

// Tremor Detection
#define TREMOR_VARIANCE_MIN 0.05
#define TREMOR_VARIANCE_MAX 0.5
#define TREMOR_DETECTION_COUNT 3

// Temperature Alerts
#define FEVER_THRESHOLD 38.0
#define HYPOTHERMIA_THRESHOLD 35.0

// ========== GLOBAL VARIABLES ==========
uint32_t lastMainLoop = 0;
uint32_t lastLCDUpdate = 0;
uint32_t lastTempRead = 0;
uint32_t lastECGSample = 0;
uint32_t lastMicSample = 0;
uint32_t lastMPUSample = 0;

// Temperature
float bodyTemp = 0;
float ambientTemp = 0;
bool tempSensorConnected = false;

// ECG
uint16_t rawECG = 0;
int16_t filteredECG = 0;
int16_t ecgBaseline = 2048;
float currentBPM = 0;
float avgBPM = 0;
uint16_t totalBeats = 0;
bool ecgLeadsConnected = false;
bool ecgModuleDetected = false;  // Flag to track if ECG module is physically connected
uint32_t lastPeakTime = 0;
uint32_t peakIntervals[5] = {0};
uint8_t peakIntervalIndex = 0;
int16_t lastFilteredValue = 0;
uint16_t ecgFilterBuffer[5] = {0};
uint8_t ecgFilterIndex = 0;

// MICROPHONE VARIABLES
float micBuffer[MIC_BUFFER_SIZE];
uint16_t micBufferIndex = 0;
bool micBufferFilled = false;
uint32_t noiseFloor = 0;
uint32_t baselineCoughEnergy = 0;
uint32_t detectionThreshold = 0;
uint32_t lastCoughTime = 0;
uint16_t coughCount = 0;
bool coughInProgress = false;
uint32_t coughStartTime = 0;
float coughPeakAmplitude = 0;
uint32_t coughEnergySum = 0;
uint16_t coughSampleCount = 0;
bool isCalibrated = false;
uint16_t calibrationCoughCount = 0;
uint32_t calibrationEnergies[CALIBRATION_COUGHS];
uint32_t noiseCalibrationSum = 0;
uint16_t noiseCalibrationCount = 0;

// MPU6500
float accMag = 0;
float lastAccMag = 1.0;
float accBuffer[128];
int accBufferIndex = 0;
bool fallDetected = false;
uint32_t fallDetectedTime = 0;
uint8_t tremorDetectionCount = 0;
bool tremorActive = false;

// Display
uint8_t displayMode = 0;
uint32_t lastDisplaySwitch = 0;
uint8_t lastDisplayMode = 255;

// Alerts
bool alertFall = false;
bool alertTremor = false;
bool alertCough = false;
bool alertFever = false;
bool alertArrhythmia = false;

// Custom LCD Characters
byte heartChar[8] = {0b00000,0b01010,0b11111,0b11111,0b11111,0b01110,0b00100,0b00000};
byte alertChar[8] = {0b00100,0b01110,0b01110,0b01110,0b01110,0b00000,0b00100,0b00000};
byte tempChar[8] = {0b00100,0b01010,0b01010,0b01110,0b01110,0b11111,0b11111,0b01110};
byte waveChar[8] = {0b00000,0b00000,0b00100,0b01010,0b10001,0b00000,0b00000,0b00000};

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\n========================================");
  Serial.println("HEALTH MONITORING SYSTEM - FINAL VERSION");
  Serial.println("Active Health Monitor with Smart Display");
  Serial.println("========================================\n");
  
  // Initialize LCD
  delay(50);
  lcd.begin(16, 2);
  delay(50);
  
  lcd.createChar(0, heartChar);
  lcd.createChar(1, alertChar);
  lcd.createChar(2, tempChar);
  lcd.createChar(3, waveChar);
  delay(10);
  
  lcd.clear();
  delay(2);
  lcd.setCursor(0, 0);
  lcd.print("Health Monitor");
  delay(2);
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(1000);
  
  // Initialize I2C (MLX90614)
  Serial.println("[1/4] Initializing Temperature Sensor...");
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();
  Wire.setClock(100000);
  delay(100);
  tempSensorConnected = checkMLX90614();
  Serial.println(tempSensorConnected ? "✓ MLX90614 Connected" : "✗ MLX90614 Not Found");
  
  // Initialize ECG
  Serial.println("[2/4] Initializing ECG Sensor...");
  pinMode(ECG_PIN, INPUT_ANALOG);
  pinMode(LO_PLUS_PIN, INPUT);
  pinMode(LO_MINUS_PIN, INPUT);
  analogReadResolution(12);
  calibrateECGBaseline();
  Serial.println("✓ ECG Ready");
  
  // Initialize Microphone
  Serial.println("[3/4] Initializing Microphone...");
  pinMode(MIC_PIN, INPUT_ANALOG);
  Serial.println("✓ Microphone ready");
  Serial.println("\n╔════════════════════════════════════╗");
  Serial.println("║  STAY QUIET FOR 2 SECONDS         ║");
  Serial.println("║  Then COUGH 5 TIMES               ║");
  Serial.println("╚════════════════════════════════════╝\n");
  
  // Initialize MPU6500
  Serial.println("[4/4] Initializing Motion Sensor...");
  pinMode(MPU_CS_PIN, OUTPUT);
  digitalWrite(MPU_CS_PIN, HIGH);
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  mpuWriteByte(MPU6500_PWR_MGMT_1, 0x00);
  delay(100);
  uint8_t whoAmI = mpuReadByte(MPU6500_WHO_AM_I);
  Serial.print("MPU6500 WHO_AM_I: 0x");
  Serial.println(whoAmI, HEX);
  Serial.println(whoAmI == 0x70 ? "✓ MPU6500 Connected" : "✗ MPU6500 Not Found");
  
  lcd.clear();
  delay(2);
  lcd.setCursor(0, 0);
  lcd.print("Calibrating...");
  lcd.setCursor(0, 1);
  lcd.print("Stay quiet 2s");
}

// ========== MAIN LOOP ==========
void loop() {
  uint32_t currentTime = millis();
  
  // ECG Sampling (500Hz)
  if (currentTime - lastECGSample >= ECG_SAMPLE_INTERVAL) {
    lastECGSample = currentTime;
    processECG();
  }
  
  // Microphone Sampling (200Hz)
  if (currentTime - lastMicSample >= MIC_SAMPLE_INTERVAL) {
    lastMicSample = currentTime;
    processMicrophoneFixed();
  }
  
  // MPU6500 Sampling (50Hz)
  if (currentTime - lastMPUSample >= MPU_SAMPLE_INTERVAL) {
    lastMPUSample = currentTime;
    processMPU6500();
  }
  
  // Temperature Reading (0.5Hz)
  if (currentTime - lastTempRead >= TEMP_READ_INTERVAL) {
    lastTempRead = currentTime;
    processTemperature();
  }
  
  // Main Loop Processing (20Hz)
  if (currentTime - lastMainLoop >= MAIN_LOOP_INTERVAL) {
    lastMainLoop = currentTime;
    updateAlertStatus();
    
    // Auto-cycle display modes (3 seconds) - only when no alerts
    if (isCalibrated && !alertFall && !alertFever && !alertTremor && 
        !alertCough && !alertArrhythmia && currentTime - lastDisplaySwitch >= 3000) {
      lastDisplaySwitch = currentTime;
      displayMode = (displayMode + 1) % 3;  // Cycle through 0, 1, 2 (Summary, ECG, Vitals)
    }
  }
  
  // LCD Update (1Hz)
  if (currentTime - lastLCDUpdate >= LCD_UPDATE_INTERVAL) {
    lastLCDUpdate = currentTime;
    updateDisplay();
  }
}

// ========== MICROPHONE PROCESSING ==========
void processMicrophoneFixed() {
  uint32_t currentTime = millis();
  
  // Read microphone and convert to amplitude (remove DC bias)
  uint16_t micRaw = analogRead(MIC_PIN);
  float micAmplitude = abs((float)micRaw - 2048.0);
  
  // Store in circular buffer
  micBuffer[micBufferIndex] = micAmplitude;
  micBufferIndex++;
  if (micBufferIndex >= MIC_BUFFER_SIZE) {
    micBufferIndex = 0;
    micBufferFilled = true;
  }
  
  // Wait for buffer to fill
  if (!micBufferFilled) {
    return;
  }
  
  // Calculate window energy
  uint32_t windowEnergy = 0;
  for (int i = 0; i < MIC_BUFFER_SIZE; i++) {
    windowEnergy += (uint32_t)(micBuffer[i] * micBuffer[i]);
  }
  
  // Calculate rapid rise
  float rapidRise = 0;
  if (micBufferIndex >= 20) {
    float recentAvg = 0, olderAvg = 0;
    for (int i = 0; i < 10; i++) {
      int recentIdx = (micBufferIndex - 1 - i + MIC_BUFFER_SIZE) % MIC_BUFFER_SIZE;
      int olderIdx = (micBufferIndex - 11 - i + MIC_BUFFER_SIZE) % MIC_BUFFER_SIZE;
      recentAvg += micBuffer[recentIdx];
      olderAvg += micBuffer[olderIdx];
    }
    rapidRise = (recentAvg / 10.0) - (olderAvg / 10.0);
  }
  
  // ========== CALIBRATION MODE ==========
  if (!isCalibrated) {
    // Phase 1: Establish noise floor
    if (calibrationCoughCount == 0 && noiseCalibrationCount < 400) {
      noiseCalibrationSum += windowEnergy;
      noiseCalibrationCount++;
      
      if (noiseCalibrationCount == 400) {
        noiseFloor = noiseCalibrationSum / noiseCalibrationCount;
        Serial.print("Noise floor: ");
        Serial.print(noiseFloor);
        Serial.print(" | Current amplitude: ");
        Serial.println(micAmplitude);
        
        Serial.println("\n╔════════════════════════════════════╗");
        Serial.println("║  NOW COUGH 5 TIMES                ║");
        Serial.println("╚════════════════════════════════════╝\n");
        
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Cough 5x to");
        lcd.setCursor(0, 1);
        lcd.print("calibrate...");
        
        calibrationCoughCount = 1;
      }
      return;
    }
    
    // Phase 2: Collect calibration coughs
    if (calibrationCoughCount > 0 && calibrationCoughCount <= CALIBRATION_COUGHS) {
      uint32_t threshold = noiseFloor * 10;
      
      if (!coughInProgress && windowEnergy > threshold && rapidRise > RAPID_RISE_THRESHOLD) {
        coughInProgress = true;
        coughStartTime = currentTime;
        coughPeakAmplitude = micAmplitude;
        coughEnergySum = windowEnergy;
        coughSampleCount = 1;
        
        Serial.print(">>> Cough started | Energy: ");
        Serial.print(windowEnergy);
        Serial.print(" | Threshold: ");
        Serial.print(threshold);
        Serial.print(" | RapidRise: ");
        Serial.print((int)rapidRise);
        Serial.print(" (need >");
        Serial.print(RAPID_RISE_THRESHOLD);
        Serial.println(")");
      }
      else if (coughInProgress) {
        coughEnergySum += windowEnergy;
        coughSampleCount++;
        
        if (micAmplitude > coughPeakAmplitude) {
          coughPeakAmplitude = micAmplitude;
        }
        
        if (windowEnergy < threshold * 0.8) {
          uint32_t duration = currentTime - coughStartTime;
          
          if (duration >= COUGH_MIN_DURATION && duration <= COUGH_MAX_DURATION) {
            uint32_t avgCoughEnergy = coughEnergySum / coughSampleCount;
            
            calibrationEnergies[calibrationCoughCount - 1] = avgCoughEnergy;
            
            Serial.print("✓ Calibration cough #");
            Serial.print(calibrationCoughCount);
            Serial.print(" | Duration: ");
            Serial.print(duration);
            Serial.print("ms | Peak: ");
            Serial.print((int)coughPeakAmplitude);
            Serial.print(" | Energy: ");
            Serial.println(avgCoughEnergy);
            
            calibrationCoughCount++;
            
            if (calibrationCoughCount > CALIBRATION_COUGHS) {
              // Sort energies
              for (int i = 0; i < CALIBRATION_COUGHS - 1; i++) {
                for (int j = i + 1; j < CALIBRATION_COUGHS; j++) {
                  if (calibrationEnergies[i] > calibrationEnergies[j]) {
                    uint32_t temp = calibrationEnergies[i];
                    calibrationEnergies[i] = calibrationEnergies[j];
                    calibrationEnergies[j] = temp;
                  }
                }
              }
              
              baselineCoughEnergy = calibrationEnergies[CALIBRATION_COUGHS / 2];
              detectionThreshold = (uint32_t)(baselineCoughEnergy * 0.7);
              
              isCalibrated = true;
              
              Serial.println("\n╔════════════════════════════════════╗");
              Serial.println("║   ✓✓✓ CALIBRATION COMPLETE ✓✓✓   ║");
              Serial.println("╚════════════════════════════════════╝");
              Serial.print("Baseline cough energy: ");
              Serial.println(baselineCoughEnergy);
              Serial.print("Detection threshold: ");
              Serial.println(detectionThreshold);
              Serial.println("System ready for monitoring!");
              
              lcd.clear();
              lcd.setCursor(0, 0);
              lcd.print("Calibrated!");
              lcd.setCursor(0, 1);
              lcd.print("Monitoring...");
              delay(2000);
            }
          }
          else {
            Serial.print(">>> False alarm - Invalid duration (");
            Serial.print(duration);
            Serial.println("ms)");
          }
          
          coughInProgress = false;
        }
      }
    }
  }
  
  // ========== DETECTION MODE ==========
  else {
    if (currentTime - lastCoughTime < COUGH_COOLDOWN_MS) {
      return;
    }
    
    if (!coughInProgress && windowEnergy > detectionThreshold && rapidRise > RAPID_RISE_THRESHOLD) {
      coughInProgress = true;
      coughStartTime = currentTime;
      coughPeakAmplitude = micAmplitude;
      
      Serial.print("🔴 COUGH DETECTED! | Energy: ");
      Serial.print(windowEnergy);
      Serial.print(" | Threshold: ");
      Serial.print(detectionThreshold);
      Serial.print(" | RapidRise: ");
      Serial.println((int)rapidRise);
    }
    else if (coughInProgress) {
      if (micAmplitude > coughPeakAmplitude) {
        coughPeakAmplitude = micAmplitude;
      }
      
      if (windowEnergy < detectionThreshold * 0.8) {
        uint32_t duration = currentTime - coughStartTime;
        
        if (duration >= COUGH_MIN_DURATION && duration <= COUGH_MAX_DURATION) {
          coughCount++;
          lastCoughTime = currentTime;
          alertCough = true;
          
          Serial.print("   ✅ VALID COUGH | Duration: ");
          Serial.print(duration);
          Serial.print("ms | Peak: ");
          Serial.print((int)coughPeakAmplitude);
          Serial.print(" | Total coughs: ");
          Serial.println(coughCount);
        }
        else {
          Serial.print("   ❌ False alarm - Duration: ");
          Serial.print(duration);
          Serial.print("ms (");
          if (duration < COUGH_MIN_DURATION) {
            Serial.print("too short, min=");
            Serial.print(COUGH_MIN_DURATION);
          } else {
            Serial.print("too long, max=");
            Serial.print(COUGH_MAX_DURATION);
          }
          Serial.println("ms)");
        }
        
        coughInProgress = false;
      }
    }
  }
}

// ========== ECG PROCESSING ==========
void calibrateECGBaseline() {
  Serial.println("Calibrating ECG baseline...");
  uint32_t sum = 0;
  for (int i = 0; i < 100; i++) {
    sum += analogRead(ECG_PIN);
    delay(10);
  }
  ecgBaseline = sum / 100;
  Serial.print("ECG Baseline: ");
  Serial.println(ecgBaseline);
}

void processECG() {
  ecgLeadsConnected = (digitalRead(LO_PLUS_PIN) == LOW && digitalRead(LO_MINUS_PIN) == LOW);
  
  // Check if ECG module is physically connected by testing for activity
  static uint16_t unchangedReadings = 0;
  static uint16_t lastRawReading = 0;
  
  uint16_t testReading = analogRead(ECG_PIN);
  if (abs((int)testReading - (int)lastRawReading) < 5 && testReading > 2000 && testReading < 2100) {
    unchangedReadings++;
    if (unchangedReadings > 100) {  // 100 samples of no change = module not connected
      ecgModuleDetected = false;
    }
  } else {
    unchangedReadings = 0;
    ecgModuleDetected = true;
  }
  lastRawReading = testReading;
  
  if (!ecgLeadsConnected || !ecgModuleDetected) {
    currentBPM = 0;
    alertArrhythmia = false;  // Don't show arrhythmia if module not detected
    return;
  }
  
  rawECG = analogRead(ECG_PIN);
  
  // Moving average filter
  ecgFilterBuffer[ecgFilterIndex] = rawECG;
  ecgFilterIndex = (ecgFilterIndex + 1) % 5;
  
  uint32_t sum = 0;
  for (int i = 0; i < 5; i++) {
    sum += ecgFilterBuffer[i];
  }
  filteredECG = (sum / 5) - ecgBaseline;
  
  // Peak detection
  uint32_t currentTime = millis();
  if (filteredECG > ECG_PEAK_THRESHOLD && 
      lastFilteredValue <= ECG_PEAK_THRESHOLD &&
      (currentTime - lastPeakTime) > ECG_REFRACTORY_PERIOD) {
    
    uint32_t interval = currentTime - lastPeakTime;
    lastPeakTime = currentTime;
    totalBeats++;
    
    peakIntervals[peakIntervalIndex] = interval;
    peakIntervalIndex = (peakIntervalIndex + 1) % 5;
    
    // Calculate average BPM
    uint32_t sumIntervals = 0;
    uint8_t validIntervals = 0;
    for (int i = 0; i < 5; i++) {
      if (peakIntervals[i] > 0) {
        sumIntervals += peakIntervals[i];
        validIntervals++;
      }
    }
    
    if (validIntervals > 0) {
      float avgInterval = sumIntervals / (float)validIntervals;
      currentBPM = 60000.0 / avgInterval;
      
      if (currentBPM >= MIN_BPM && currentBPM <= MAX_BPM) {
        avgBPM = avgBPM * 0.8 + currentBPM * 0.2;
      }
    }
    
    // Check for arrhythmia (only if module is connected)
    if (validIntervals >= 3 && ecgModuleDetected) {
      float variance = 0;
      float mean = sumIntervals / (float)validIntervals;
      for (int i = 0; i < validIntervals; i++) {
        float diff = peakIntervals[i] - mean;
        variance += diff * diff;
      }
      variance /= validIntervals;
      
      if (variance > 10000) {
        alertArrhythmia = true;
      } else {
        alertArrhythmia = false;
      }
    }
  }
  
  lastFilteredValue = filteredECG;
}


// ========== MPU6500 PROCESSING ==========
void processMPU6500() {
  uint8_t buffer[6];
  mpuReadBytes(MPU6500_ACCEL_XOUT_H, buffer, 6);
  
  int16_t ax = (int16_t)(buffer[0] << 8 | buffer[1]);
  int16_t ay = (int16_t)(buffer[2] << 8 | buffer[3]);
  int16_t az = (int16_t)(buffer[4] << 8 | buffer[5]);
  
  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;
  
  accMag = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
  accMag = 0.8 * accMag + 0.2 * lastAccMag;
  lastAccMag = accMag;
  
  // Fall Detection
  if (accMag > FALL_THRESHOLD) {
    if (!fallDetected) {
      fallDetected = true;
      fallDetectedTime = millis();
      alertFall = true;
      Serial.println("⚠️ FALL DETECTED!");
    }
  } else if (fallDetected && millis() - fallDetectedTime > FALL_TIME_WINDOW) {
    fallDetected = false;
    alertFall = false;
    Serial.println("✓ Fall alert cleared");
  }
  
  // Store for tremor analysis
  accBuffer[accBufferIndex] = accMag;
  accBufferIndex = (accBufferIndex + 1) % 128;
  
  // Tremor Detection
  if (accBufferIndex == 0) {
    float mean = 0, variance = 0;
    for (int i = 0; i < 128; i++) mean += accBuffer[i];
    mean /= 128;
    for (int i = 0; i < 128; i++) {
      float diff = accBuffer[i] - mean;
      variance += diff * diff;
    }
    variance /= 128;
    
    if (variance > TREMOR_VARIANCE_MIN && variance < TREMOR_VARIANCE_MAX) {
      tremorDetectionCount++;
      if (tremorDetectionCount >= TREMOR_DETECTION_COUNT && !alertTremor) {
        tremorActive = true;
        alertTremor = true;
        Serial.println("⚠️ TREMOR DETECTED!");
      }
    } else {
      tremorDetectionCount = 0;
      tremorActive = false;
      if (alertTremor) {
        alertTremor = false;
        Serial.println("✓ Tremor cleared - motion stable");
      }
    }
  }
}

// ========== TEMPERATURE PROCESSING ==========
void processTemperature() {
  if (!tempSensorConnected) return;
  
  uint16_t rawObject = readRawTemp(MLX90614_TOBJ1);
  uint16_t rawAmbient = readRawTemp(MLX90614_TA);
  
  bodyTemp = rawToTemp(rawObject);
  ambientTemp = rawToTemp(rawAmbient);
  
  if (bodyTemp > FEVER_THRESHOLD) {
    alertFever = true;
    Serial.print("🌡️ FEVER: ");
    Serial.print(bodyTemp);
    Serial.println("°C");
  } else {
    alertFever = false;
  }
}

// ========== DISPLAY FUNCTIONS ==========
void updateDisplay() {
  if (!isCalibrated) {
    // Show calibration status
    lcd.setCursor(0, 0);
    if (calibrationCoughCount == 0) {
      lcd.print("Calibrating...  ");
      lcd.setCursor(0, 1);
      lcd.print("Stay quiet...   ");
    } else {
      lcd.print("Calibrating...  ");
      lcd.setCursor(0, 1);
      lcd.print("Coughs: ");
      lcd.print(calibrationCoughCount - 1);
      lcd.print("/");
      lcd.print(CALIBRATION_COUGHS);
      lcd.print("   ");
    }
    return;
  }
  
  // Clear screen if mode changed
  if (lastDisplayMode != displayMode) {
    lcd.clear();
    delay(2);
    lastDisplayMode = displayMode;
  }
  
  // PRIORITY 1: Critical Alerts (override everything)
  if (alertFall || alertFever || alertTremor) {
    displayAlerts();
    return;
  }
  
  // PRIORITY 2: Recent Cough (show for 3 seconds)
  if (alertCough) {
    displayCoughs();
    return;
  }
  
  // PRIORITY 3: Warning Alerts
  if (alertArrhythmia) {
    displayAlerts();
    return;
  }
  
  // PRIORITY 4: Normal cycling display
  switch (displayMode) {
    case 0: displaySummary(); break;
    case 1: displayECG(); break;
    case 2: displayVitals(); break;
    default: displayMode = 0; displaySummary(); break;
  }
}

void displaySummary() {
  lcd.write((uint8_t)0);
  lcd.print(" ");
  if (!ecgModuleDetected) {
    lcd.print("-");
  } else if (avgBPM > 0) {
    lcd.print((int)avgBPM);
  } else {
    lcd.print("--");
  }
  lcd.print(" ");
  lcd.print(" ");
  
  lcd.write((uint8_t)2);
  if (tempSensorConnected && bodyTemp > 0) {
    lcd.print((int)bodyTemp);
  } else {
    lcd.print("--");
  }
  lcd.print("C  ");
  
  lcd.setCursor(0, 1);
  lcd.print("C:");
  lcd.print(coughCount);
  lcd.print(" B:");
  lcd.print(totalBeats);
  lcd.print("      ");
}

void displayECG() {
  lcd.setCursor(0, 0);
  lcd.write((uint8_t)0);
  lcd.print(" ECG: ");
  if (!ecgModuleDetected) {
    lcd.print("N/A");
  } else if (ecgLeadsConnected) {
    if (avgBPM > 0) {
      lcd.print((int)avgBPM);
      lcd.print(" BPM");
    } else {
      lcd.print("Detecting");
    }
  } else {
    lcd.print("LEADS OFF");
  }
  lcd.print("  ");
  
  lcd.setCursor(0, 1);
  if (!ecgModuleDetected) {
    lcd.print("Module N/A      ");
  } else if (alertArrhythmia) {
    lcd.print("! IRREGULAR !  ");
  } else {
    lcd.print("Beats: ");
    lcd.print(totalBeats);
    lcd.print("      ");
  }
  lcd.print(" ECG: ");
  if (ecgLeadsConnected) {
    if (avgBPM > 0) {
      lcd.print((int)avgBPM);
      lcd.print(" BPM");
    } else {
      lcd.print("Detecting");
    }
  } else {
    lcd.print("LEADS OFF");
  }
  lcd.print("  ");
  
  lcd.setCursor(0, 1);
  if (alertArrhythmia) {
    lcd.print("! IRREGULAR !  ");
  } else {
    lcd.print("Beats: ");
    lcd.print(totalBeats);
    lcd.print("      ");
  }
}

void displayVitals() {
  lcd.setCursor(0, 0);
  lcd.write((uint8_t)2);
  lcd.print(" Temp: ");
  if (tempSensorConnected && bodyTemp > 0) {
    lcd.print(bodyTemp, 1);
    lcd.print("C");
  } else {
    lcd.print("--");
  }
  lcd.print("    ");
  
  lcd.setCursor(0, 1);
  lcd.print("Ambient: ");
  if (tempSensorConnected && ambientTemp > 0) {
    lcd.print((int)ambientTemp);
    lcd.print("C");
  } else {
    lcd.print("--");
  }
  lcd.print("    ");
}

void displayCoughs() {
  lcd.setCursor(0, 0);
  lcd.write((uint8_t)1);
  lcd.print(" COUGH!");
  lcd.print("        ");
  
  lcd.setCursor(0, 1);
  lcd.print("Count: ");
  lcd.print(coughCount);
  lcd.print(" Peak:");
  lcd.print((int)coughPeakAmplitude);
  lcd.print("  ");
}

void displayAlerts() {
  lcd.setCursor(0, 0);
  lcd.write((uint8_t)1);
  lcd.print(" ALERT!        ");
  
  lcd.setCursor(0, 1);
  if (alertTremor) {
    lcd.print("TREMOR!         ");
  } else if (alertFall) {
    lcd.print("FALL DETECTED!  ");
  } else if (alertFever) {
    lcd.print("FEVER!          ");
  } else if (alertArrhythmia) {
    lcd.print("IRREGULAR HR!   ");
  } 
  else {
    lcd.print("System Normal   ");
  }
}

void updateAlertStatus() {
  // Auto-clear fall alert after 10 seconds
  if (alertFall && millis() - fallDetectedTime > 10000) {
    alertFall = false;
    fallDetected = false;
    Serial.println("Fall alert auto-cleared");
  }
  
  // Auto-clear cough alert after 3 seconds
  if (alertCough && millis() - lastCoughTime > 3000) {
    alertCough = false;
  }
  
  // Tremor clears automatically in processMPU6500() when variance is normal
}

// ========== MLX90614 FUNCTIONS ==========
bool checkMLX90614() {
  Wire.beginTransmission(MLX90614_ADDR);
  return (Wire.endTransmission() == 0);
}
re  AWT
uint16_t readRawTemp(uint8_t reg) {
  Wire.beginTransmission(MLX90614_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;
  
  Wire.requestFrom(MLX90614_ADDR, (uint8_t)3);
  if (Wire.available() >= 2) {
    uint8_t lsb = Wire.read();
    uint8_t msb = Wire.read();
    Wire.read();
    return (msb << 8) | lsb;
  }
  return 0;
}

float rawToTemp(uint16_t raw) {
  return (raw * 0.02) - 273.15;
}

// ========== MPU6500 SPI FUNCTIONS ==========
uint8_t mpuReadByte(uint8_t reg) {
  digitalWrite(MPU_CS_PIN, LOW);
  SPI.transfer(reg | 0x80);
  uint8_t data = SPI.transfer(0x00);
  digitalWrite(MPU_CS_PIN, HIGH);
  return data;
}

void mpuReadBytes(uint8_t reg, uint8_t *buffer, uint8_t len) {
  digitalWrite(MPU_CS_PIN, LOW);
  SPI.transfer(reg | 0x80);
  for (uint8_t i = 0; i < len; i++) {
    buffer[i] = SPI.transfer(0x00);
  }
  digitalWrite(MPU_CS_PIN, HIGH);
}

void mpuWriteByte(uint8_t reg, uint8_t data) {
  digitalWrite(MPU_CS_PIN, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(data);
  digitalWrite(MPU_CS_PIN, HIGH);
}
