# Automatic Door Locking and Unlocking System

A sensor-based smart access control system that automatically locks and unlocks a door using **PIR**, **Ultrasonic (HC-SR04)**, and **IR Break-Beam** sensors — each simulated independently to identify the best sensor for different regions of a house.

## Overview

Traditional locks rely on manual keys or keypads, which can be lost, duplicated, or forgotten. This project replaces manual locking with sensor-based detection: when a person or object is detected near the door, an **SG90 micro servo** automatically actuates the lock, while a **0.96" OLED display (SSD1306, I2C)** shows real-time door status.

Three sensing approaches were implemented and simulated independently, using the same core sense → decide → actuate → display control loop, so their detection behavior could be fairly compared:

| Sensor | Simulation Platform |
|---|---|
| PIR Sensor | Tinkercad Circuits |
| Ultrasonic Sensor (HC-SR04) | Cirkit Designer |
| IR Break-Beam Sensor | Cirkit Designer |

## Objectives

- Design and simulate automatic door control using three different sensor technologies.
- Compare PIR, Ultrasonic, and IR sensors to identify the most appropriate sensor for different regions/entry points of a house.
- Provide real-time visual feedback of door status (Opened/Closed) via OLED.
- Control a servo motor representing the door's lock/unlock mechanism.

## Choosing the Right Sensor for the Right Region

The deciding factor isn't just detection range — it's how much uncontrolled foot traffic passes near the door. PIR and IR sensors can only detect *presence or motion*, not distance or intent, so on a high-traffic ground-floor door they'd falsely unlock for any passerby. Ultrasonic sensors measure exact distance, so a tight threshold (e.g. < 15 cm) ensures the door reacts only to someone genuinely standing at it.

| House Region | Recommended Sensor | Reason |
|---|---|---|
| Ground floor main entrance (high foot traffic) | Ultrasonic (HC-SR04) | Precise distance threshold prevents false unlocking from passing traffic |
| Garage door / gate | Ultrasonic (HC-SR04) | Detects exact vehicle distance before opening |
| Low-traffic interior room door | PIR | Controlled, known traffic — wide-area convenience matters more than precision |
| Narrow interior doorway / cabinet / locker | IR Break-Beam | Reliable ON/OFF beam detection in a low-traffic, single-file passage |
| Upper-floor balcony / low-footfall backyard | PIR | Not exposed to public traffic, so PIR's wider detection cone is not a security gap |

### Sensor Comparison

| Parameter | PIR Sensor | Ultrasonic Sensor | IR Break-Beam Sensor |
|---|---|---|---|
| Detection Principle | Passive infrared / body heat | Sound wave echo (time-of-flight) | IR light beam interruption |
| Range | ~6–7 m, wide cone | 2–400 cm, narrow beam | Few cm – few m (beam length) |
| Precision | Low (area-based) | High (exact distance in cm) | High (binary: blocked/clear) |
| False-Trigger Risk | High in busy areas | Low — distance threshold filters passersby | Low, but only within beam path |
| Affected By | Heat sources, pets, sunlight | Soft/sound-absorbing surfaces | Ambient IR light, misalignment |
| Ideal Region | Low-traffic interior, upper floors | Ground floor entrance, garages, gates | Narrow, low-traffic doorways |
| Cost | Low | Low–Medium | Low |

## Components

| Component | Qty | Purpose |
|---|---|---|
| Arduino Uno | 1 | Main microcontroller running the control logic |
| PIR Motion Sensor (HC-SR501 / Parallax PIR) | 1 | Detects human motion for the entrance configuration |
| Ultrasonic Sensor (HC-SR04) | 1 | Measures distance to approaching objects/vehicles |
| IR Break-Beam Sensor | 1 | Detects presence via beam interruption |
| SG90 Micro Servo Motor | 1 | Door locking/unlocking actuator |
| 0.96" OLED Display (SSD1306, I2C, 128×64) | 1 | Displays real-time door status and distance |
| Red & Green LEDs | 2 | Visual lock/unlock indicators (PIR configuration) |
| 220Ω Resistors | 2 | Current-limiting for LEDs |
| Breadboard | 1 | Prototyping and power distribution |
| Jumper Wires (M-M) | As required | Circuit connections |
| USB Cable / 5V Power Supply | 1 | Power source for Arduino Uno |

### Software Used

- **Tinkercad Circuits** — PIR-based circuit design and simulation
- **Cirkit Designer** — Ultrasonic and IR-based circuit design and simulation
- **Adafruit SSD1306** and **Adafruit GFX** libraries — OLED display driving over I2C
- **Servo.h** — servo motor control
- **Wire.h** — I2C communication with the OLED

## System Design

Each sensor variant follows the same control loop:

1. **Sense** — the active sensor continuously monitors for a person or object.
2. **Decide** — the Arduino evaluates the reading against a threshold (motion HIGH for PIR, distance < 15 cm for Ultrasonic, beam-broken LOW for IR).
3. **Act** — the servo rotates to 90° (open/unlocked) or returns to 0° (closed/locked).
4. **Display** — the OLED (or Serial Monitor, in early IR tests) shows "Door is Opened" or "Door is Closed", plus live distance for the ultrasonic variant.

## 1. PIR Sensor Variant (Tinkercad)

Designed for wide-area motion detection at a home's main entrance, using red/green LEDs alongside servo actuation.

**Circuit Connections**

| Component | Pin | Arduino Pin |
|---|---|---|
| PIR Sensor | OUT | Digital Pin 9 |
| PIR Sensor | VCC / GND | 5V / GND |
| Servo Motor | Signal | Digital Pin 6 |
| Servo Motor | VCC / GND | 5V / GND |
| Red LED | Anode (via 220Ω resistor) | Digital Pin 2 |
| Green LED | Anode (via 220Ω resistor) | Digital Pin 4 |

**Result:** Motion detected → servo rotates to 90°, green LED ON. No motion → servo returns to 0°, red LED ON.

## 2. Ultrasonic Sensor Variant (Cirkit Designer)

Suited to garages or gates, where the exact distance of an approaching object/vehicle must be known before actuating the door. Displays live measured distance on the OLED.

**Circuit Connections**

| Component | Pin | Arduino Pin |
|---|---|---|
| HC-SR04 Ultrasonic Sensor | Trig | Digital Pin 8 |
| HC-SR04 Ultrasonic Sensor | Echo | Digital Pin 7 |
| HC-SR04 Ultrasonic Sensor | VCC / GND | 5V / GND |
| Servo Motor | Signal | Digital Pin 3 |
| Servo Motor | VCC / GND | 5V / GND |
| OLED Display | SDA / SCL | A4 / A5 |
| OLED Display | VCC / GND | 5V / GND |

**Source Code**

```cpp
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Servo doorServo;

const int trigPin = 8;
const int echoPin = 7;
const int servoPin = 3;

long duration;
int distance;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  doorServo.attach(servoPin);
  doorServo.write(0);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
}

void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  if (distance > 0 && distance < 15) {
    doorServo.write(90); // open
    display.println("Opened ");
  } else {
    doorServo.write(0); // close
    display.println("Closed");
  }

  display.setCursor(0, 30);
  display.print("Dist:");
  display.print(distance);
  display.println(" cm");

  display.display();
  delay(200);
}
```

**Result:** Object beyond 15 cm → door closed (e.g. 117 cm tested). Object within 15 cm → door opens (e.g. 1 cm tested).

## 3. IR Break-Beam Sensor Variant (Cirkit Designer)

Best for narrow doorways, cabinets, or lockers where simple, reliable line-of-sight detection is preferred over an area-based sensor. Two versions were tested: an initial Serial Monitor version, and a refined OLED version.

**Circuit Connections**

| Component | Pin | Arduino Pin |
|---|---|---|
| IR Break-Beam Sensor | OUT | Digital Pin 7 |
| IR Break-Beam Sensor | VCC / GND | 5V / GND |
| Servo Motor | Signal | Digital Pin 3 |
| Servo Motor | VCC / GND | 5V / GND |
| OLED Display | SDA / SCL | A4 / A5 |
| OLED Display | VCC / GND | 5V / GND |

**Result:** Beam interrupted (LOW) → servo rotates to 90°, "Opened"/"Object detected — Opening door" shown. Beam clear (HIGH) → servo returns to 0°, "Closed"/"No object - Door closed" shown.

## Results Summary

| Sensor Variant | Trigger Condition | Door State Observed | Feedback Method |
|---|---|---|---|
| PIR | Motion detected (HIGH) | Servo → 90°, green LED ON | LED indicators |
| PIR | No motion (LOW) | Servo → 0°, red LED ON | LED indicators |
| Ultrasonic | Distance < 15 cm | Servo → 90°, "Opened" shown | OLED (with live distance) |
| Ultrasonic | Distance ≥ 15 cm | Servo → 0°, "Closed" shown | OLED (with live distance) |
| IR Break-Beam | Beam interrupted (LOW) | Servo → 90°, "Opened"/message printed | OLED / Serial Monitor |
| IR Break-Beam | Beam clear (HIGH) | Servo → 0°, "Closed"/message printed | OLED / Serial Monitor |

All three configurations were successfully simulated and correctly transitioned between locked/closed and unlocked/open states.

## Advantages

- Low-cost, easily reproducible design using widely available components
- Modular sensor approach — the same servo/OLED logic reuses across all three sensor types
- Real-time visual feedback improves usability over a purely mechanical lock
- Flexibility to select the most appropriate sensor per house region rather than one-size-fits-all

## Limitations

- PIR sensors can produce false triggers from pets or nearby heat sources
- Ultrasonic sensors may give inconsistent readings on soft or sound-absorbing surfaces
- IR break-beam sensors require precise physical alignment between transmitter and receiver
- Current design is simulation-based; real-world deployment requires weatherproofing and a manual override for safety

## Future Scope

- Keypad or RFID-based override for manual access
- Integration with a mobile app via Wi-Fi/Bluetooth (e.g., ESP32) for remote monitoring
- Battery backup with solar charging for outdoor deployments
- Data-logging feature to record entry/exit events for security auditing

## Conclusion

This project demonstrates an Automatic Door Locking and Unlocking System using three distinct sensing technologies — PIR, Ultrasonic, and IR break-beam — each simulated and validated on Tinkercad Circuits and Cirkit Designer. Sensor selection should be tailored to the specific region of a house: PIR for wide entrances, Ultrasonic for distance-critical locations like garages and gates, and IR break-beam for narrow, precise-detection doorways.

## References

- [Arduino Official Documentation](https://docs.arduino.cc)
- [Adafruit SSD1306 & GFX Library Documentation](https://learn.adafruit.com/monochrome-oled-breakouts)
- [Tinkercad Circuits](https://www.tinkercad.com/circuits)
- [Cirkit Designer](https://app.cirkitdesigner.com)
- HC-SR04 Ultrasonic Sensor Datasheet
- Parallax PIR Sensor (555-28027) Datasheet

## Author

**Pathan Mohammad Shoaib Khan**
B.Tech, Electronics and Communication Engineering
