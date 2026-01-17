# 🏥 Health Monitoring System

<div align="center">

![Version](https://img.shields.io/badge/version-1.0-blue.svg)
![Platform](https://img.shields.io/badge/platform-STM32-green.svg)
![License](https://img.shields.io/badge/license-MIT-orange.svg)

*An intelligent real-time health monitoring system using STM32 Black Pill*

[Features](#-features) • [Hardware](#-hardware-requirements) • [Installation](#️-installation) • [Usage](#-usage) • [Connections](#-pin-connections)

</div>

---

## 📋 Overview

This project is a comprehensive health monitoring system built on the **STM32F401CCU6 Black Pill** microcontroller. It continuously monitors vital signs and environmental conditions, providing real-time alerts for potential health concerns.

### Key Capabilities

- 💓 **ECG Monitoring** - Real-time heart rate and arrhythmia detection
- 🌡️ **Temperature Sensing** - Non-contact body and ambient temperature measurement
- 🤧 **Cough Detection** - Intelligent audio-based cough tracking with calibration
- 🚨 **Fall Detection** - Accelerometer-based fall alert system
- 🤝 **Tremor Detection** - Motion variance analysis for tremor identification
- 📊 **LCD Display** - 16x2 display with auto-cycling health metrics

---

## ✨ Features

### 1. **ECG & Heart Rate Monitoring**
- Continuous heart rate measurement (40-200 BPM range)
- Lead-off detection for proper electrode connection
- Arrhythmia detection through R-R interval variance analysis
- Moving average filtering for noise reduction
- Beat counting and averaging

### 2. **Temperature Monitoring**
- Non-contact infrared temperature measurement using MLX90614
- Simultaneous body and ambient temperature tracking
- Automatic fever detection (>38.0°C)
- Hypothermia alert (<35.0°C)

### 3. **Intelligent Cough Detection**
- **Two-phase calibration system:**
  - Phase 1: 2-second noise floor establishment
  - Phase 2: 5 calibration coughs for personalized baseline
- Energy-based detection with rapid-rise analysis
- Duration validation (50-1200ms)
- Cooldown period to prevent false triggers
- Real-time cough counting

### 4. **Motion Analysis**
- **Fall Detection:** Detects sudden acceleration spikes (>2.5g)
- **Tremor Detection:** Analyzes motion variance patterns
- 128-sample circular buffer for continuous monitoring
- Automatic alert clearing when motion stabilizes

### 5. **Smart Display System**
- Priority-based alert display
- Auto-cycling through 3 display modes (3-second intervals):
  - Summary View (Heart rate, Temperature, Counts)
  - ECG Details (BPM, Total beats, Arrhythmia status)
  - Vital Signs (Body & Ambient temperature)
- Custom LCD characters for intuitive icons
- Real-time calibration status feedback

---

## 🔧 Hardware Requirements

### Microcontroller
- **STM32F401CCU6 Black Pill** (Cortex-M4, 84MHz)

### Sensors

| Component | Model | Interface | Purpose |
|-----------|-------|-----------|---------|
| Temperature Sensor | MLX90614 | I2C (0x5A) | Non-contact IR thermometer |
| ECG Module | AD8232 | Analog + Digital | Heart rate monitoring |
| Microphone | Electret/Analog | Analog | Cough detection |
| Motion Sensor | MPU6500 | SPI | Fall & tremor detection |

### Display & Interface
- **LCD:** 16x2 Character LCD (HD44780 compatible)
- **Power Supply:** 3.3V regulated (for STM32 and sensors)

---

## 📌 Pin Connections

### STM32 Black Pill Pinout

```
┌─────────────────────────────────────────┐
│         STM32F401CCU6 Black Pill        │
│                                         │
│  PB7  ────────── I2C SDA (MLX90614)     │
│  PB8  ────────── I2C SCL (MLX90614)     │
│                                         │
│  PA0  ────────── Microphone (Analog)    │
│  PA1  ────────── ECG Signal (Analog)    │
│  PA2  ────────── ECG LO+ (Digital)      │
│  PA3  ────────── ECG LO- (Digital)      │
│  PA4  ────────── MPU6500 CS (SPI)       │
│                                         │
│  PB6  ────────── LCD RS                 │
│  PB1  ────────── LCD Enable             │
│  PB2  ────────── LCD D4                 │
│  PB3  ────────── LCD D5                 │
│  PB4  ────────── LCD D6                 │
│  PB5  ────────── LCD D7                 │
│                                         │
│  3.3V ────────── Power Supply           │
│  GND  ────────── Ground                 │
└─────────────────────────────────────────┘
```

### Detailed Connection Table

#### MLX90614 Temperature Sensor (I2C)
| MLX90614 Pin | Black Pill Pin | Notes |
|--------------|----------------|-------|
| VDD | 3.3V | Power supply |
| VSS | GND | Ground |
| SDA | PB7 | I2C Data (with pull-up) |
| SCL | PB8 | I2C Clock (with pull-up) |

#### AD8232 ECG Module
| AD8232 Pin | Black Pill Pin | Notes |
|------------|----------------|-------|
| OUTPUT | PA1 | Analog ECG signal |
| LO+ | PA2 | Lead-off detection positive |
| LO- | PA3 | Lead-off detection negative |
| 3.3V | 3.3V | Power supply |
| GND | GND | Ground |

#### Microphone Module
| Microphone Pin | Black Pill Pin | Notes |
|----------------|----------------|-------|
| OUT | PA0 | Analog audio output |
| VCC | 3.3V | Power supply |
| GND | GND | Ground |

#### MPU6500 Motion Sensor (SPI)
| MPU6500 Pin | Black Pill Pin | Notes |
|-------------|----------------|-------|
| CS | PA4 | Chip Select |
| MOSI | PA7 | SPI Data Out (default SPI1) |
| MISO | PA6 | SPI Data In (default SPI1) |
| SCK | PA5 | SPI Clock (default SPI1) |
| VCC | 3.3V | Power supply |
| GND | GND | Ground |

#### 16x2 LCD Display
| LCD Pin | Black Pill Pin | Notes |
|---------|----------------|-------|
| RS | PB6 | Register Select |
| E | PB1 | Enable |
| D4 | PB2 | Data bit 4 |
| D5 | PB3 | Data bit 5 |
| D6 | PB4 | Data bit 6 |
| D7 | PB5 | Data bit 7 |
| VSS | GND | Ground |
| VDD | 5V | LCD power (with level shifters if needed) |
| V0 | 10kΩ pot | Contrast adjustment |
| RW | GND | Read/Write (tied to GND for write mode) |
| A | 5V | Backlight anode (with resistor) |
| K | GND | Backlight cathode |

> **Note:** If using a 5V LCD with the 3.3V STM32, consider using level shifters or a 3.3V compatible LCD to prevent damage.

---

## 🛠️ Installation

### Prerequisites
- Arduino IDE (1.8.x or later)
- STM32 Board Support Package for Arduino
- Required Libraries:
  - `Wire.h` (I2C communication)
  - `SPI.h` (SPI communication)
  - `LiquidCrystal.h` (LCD display)

### STM32 Board Setup

1. **Install STM32 Arduino Core:**
   - Open Arduino IDE
   - Go to `File` → `Preferences`
   - Add to "Additional Board Manager URLs":
     ```
     https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
     ```
   - Go to `Tools` → `Board` → `Boards Manager`
   - Search for "STM32" and install "STM32 MCU based boards"

2. **Board Configuration:**
   - Board: `Generic STM32F4 series`
   - Board part number: `Black Pill F401CC`
   - Upload method: `STM32CubeProgrammer (Serial)` or `STLink`
   - USB support: `CDC (generic Serial supersede U(S)ART)`
   - Optimize: `Smallest (-Os default)`

### Upload Instructions

1. Clone or download this repository
2. Open `health_monitor_modified.ino` in Arduino IDE
3. Select the correct board and port
4. Click **Upload**
5. For first-time programming:
   - Set BOOT0 jumper to 1
   - Press RESET
   - Upload sketch
   - Set BOOT0 back to 0
   - Press RESET

---

## 🚀 Usage

### Initial Setup

1. **Power on the system** - The LCD will display "Initializing..."
2. **Sensor detection** - System automatically detects connected sensors
3. **Calibration phase** (Critical for cough detection):

   ```
   Step 1: Stay quiet for 2 seconds
   ├── System establishes noise floor
   └── LCD shows: "Stay quiet 2s"

   Step 2: Cough 5 times
   ├── Cough naturally with pauses between
   ├── System calculates baseline cough energy
   └── LCD shows: "Coughs: X/5"
   ```

4. **Monitoring begins** - Display auto-cycles through health metrics

### Display Modes

#### Mode 1: Summary View
```
♥ 72  🌡️36C
C:12 B:432
```
- Heart rate, Temperature
- Cough count, Total beats

#### Mode 2: ECG Details
```
♥ ECG: 72 BPM
Beats: 432
```
- Real-time BPM
- Total beat counter

#### Mode 3: Vitals
```
🌡️ Temp: 36.5C
Ambient: 24C
```
- Body temperature
- Room temperature

### Alert Priority System

The display automatically shows alerts in this priority order:

1. **🚨 Critical Alerts** (Immediate display):
   - Fall detection
   - Fever (>38°C)
   - Active tremor

2. **⚠️ Recent Events** (3-second display):
   - Cough detected

3. **📊 Warnings**:
   - Irregular heart rate (arrhythmia)

4. **✅ Normal Operation**:
   - Auto-cycling through 3 display modes

### Serial Monitor Output

Connect to Serial Monitor (115200 baud) for detailed diagnostics:

```
========================================
HEALTH MONITORING SYSTEM - FINAL VERSION
Active Health Monitor with Smart Display
========================================

[1/4] Initializing Temperature Sensor...
✓ MLX90614 Connected
[2/4] Initializing ECG Sensor...
✓ ECG Ready
[3/4] Initializing Microphone...
✓ Microphone ready
[4/4] Initializing Motion Sensor...
MPU6500 WHO_AM_I: 0x70
✓ MPU6500 Connected

Noise floor: 12450
>>> Cough started | Energy: 125000
✅ VALID COUGH | Duration: 450ms | Total coughs: 1
⚠️ FALL DETECTED!
🌡️ FEVER: 38.2°C
```

---

## ⚙️ Configuration

### Adjustable Thresholds (in code)

#### ECG Parameters
```cpp
#define ECG_PEAK_THRESHOLD 150      // Adjust for ECG sensitivity
#define MAX_BPM 200                 // Maximum valid heart rate
#define MIN_BPM 40                  // Minimum valid heart rate
```

#### Cough Detection
```cpp
#define COUGH_MIN_DURATION 50       // Minimum cough duration (ms)
#define COUGH_MAX_DURATION 1200     // Maximum cough duration (ms)
#define RAPID_RISE_THRESHOLD 60     // Audio energy rise rate
#define CALIBRATION_COUGHS 5        // Number of calibration coughs
```

#### Motion Analysis
```cpp
#define FALL_THRESHOLD 2.5          // Acceleration threshold (g)
#define TREMOR_VARIANCE_MIN 0.05    // Minimum tremor variance
#define TREMOR_VARIANCE_MAX 0.5     // Maximum tremor variance
```

#### Temperature Alerts
```cpp
#define FEVER_THRESHOLD 38.0        // Fever temperature (°C)
#define HYPOTHERMIA_THRESHOLD 35.0  // Low temperature (°C)
```

### Timing Configuration
```cpp
#define LCD_UPDATE_INTERVAL 1000    // LCD refresh rate (ms)
#define TEMP_READ_INTERVAL 2000     // Temperature reading frequency
#define ECG_SAMPLE_INTERVAL 2       // ECG sampling rate (500Hz)
#define MIC_SAMPLE_INTERVAL 5       // Microphone sampling (200Hz)
```

---

## 📊 Technical Specifications

### Sampling Rates
| Sensor | Frequency | Purpose |
|--------|-----------|---------|
| ECG | 500 Hz | Heart rate detection |
| Microphone | 200 Hz | Cough energy analysis |
| MPU6500 | 50 Hz | Motion monitoring |
| Temperature | 0.5 Hz | Thermal monitoring |
| LCD Update | 1 Hz | Display refresh |

### Detection Algorithms

#### ECG Processing
- **Filtering:** 5-point moving average
- **Peak Detection:** Threshold-based with refractory period (300ms)
- **BPM Calculation:** Rolling 5-beat average
- **Arrhythmia:** R-R interval variance analysis (threshold: 10000)

#### Cough Detection
- **Window Size:** 50 samples @ 200Hz = 250ms
- **Energy Calculation:** Sum of squared amplitudes
- **Rapid Rise:** Compares recent vs. older 10-sample averages
- **Validation:** Duration check + cooldown period (500ms)

#### Fall Detection
- **Method:** Acceleration magnitude calculation
- **Threshold:** 2.5g (configurable)
- **Time Window:** 500ms confirmation
- **Auto-clear:** 10 seconds after detection

#### Tremor Detection
- **Buffer:** 128 accelerometer samples
- **Analysis:** Variance calculation on motion magnitude
- **Validation:** 3 consecutive positive detections
- **Auto-clear:** Immediate when variance normalizes

---

## 🔍 Troubleshooting

### Common Issues

| Problem | Possible Cause | Solution |
|---------|----------------|----------|
| LCD shows gibberish | Wrong contrast/timing | Adjust V0 pot, check connections |
| ECG shows "LEADS OFF" | Poor electrode contact | Moisten electrodes, check placement |
| Temperature reads 0°C | MLX90614 not detected | Check I2C connections, pull-ups |
| No cough detection | Poor calibration | Re-upload, ensure proper microphone |
| Fall alerts constantly | Threshold too low | Increase `FALL_THRESHOLD` value |
| MPU6500 not found | SPI connection issue | Verify SPI pins, check CS pin |

### Serial Debug Commands

Monitor serial output for real-time diagnostics:
- ECG baseline values
- Cough energy calculations
- Sensor connection status
- Alert triggers with timestamps

---

## 📈 Future Enhancements

- [ ] WiFi/Bluetooth data logging
- [ ] Mobile app integration
- [ ] Cloud-based health analytics
- [ ] Multiple user profiles
- [ ] Battery power with sleep modes
- [ ] SpO2 sensor integration
- [ ] Emergency SMS alerts
- [ ] Medication reminders

---

## 🤝 Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

### Development Guidelines
1. Maintain consistent code style
2. Comment complex algorithms
3. Test with actual hardware
4. Update documentation for new features

---

## 📄 License

This project is open-source and available under the MIT License.

---

## 🙏 Acknowledgments

- STM32duino community for Arduino core
- Sensor manufacturers for detailed datasheets
- Open-source contributors

---

## 📞 Support

For questions, issues, or suggestions:
- Open an issue on GitHub
- Check serial monitor output for debugging
- Review sensor datasheets for technical details

---

<div align="center">

**Built with ❤️ using STM32 Black Pill**

*Monitoring health, one heartbeat at a time*

</div>
