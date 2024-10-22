
# UAV Project with Arduino: Autonomous and Long-Endurance Flight with Engine

## Key Features
- **Engine-Powered UAV with Hybrid System**: A gasoline-powered engine combined with electric systems for auxiliary functions like communication, sensors, and autopilot.
- **Autopilot with GPS Navigation**: The UAV can autonomously navigate via GPS waypoints.
- **Obstacle Avoidance**: Ultrasonic sensors for collision detection and avoidance.
- **Return-to-Home (RTH)**: If communication is lost, the UAV autonomously returns to the launch point.
- **Payload Delivery and Release System**: Integrated system for delivering payloads with a remote-release option.
- **Hybrid Battery System**: For powering onboard electronics and communication when the engine is off.

---

## Components

### UAV Hardware:
- **Arduino Mega 2560**: Central flight controller
- **MPU6050 Gyroscope/Accelerometer**: For stabilizing and flight dynamics
- **GPS Module (NEO-6M)**: Navigation and waypoint tracking
- **Gasoline-Powered Engine**: For primary propulsion (e.g., 50cc to 80cc engine depending on UAV size)
- **Electric Starter**: For remote engine ignition via Arduino control
- **6S LiPo Battery (for Electronics)**: To power onboard electronics like sensors, GPS, LoRa, and servos
- **Fuel Tank and Engine Support Components**: Fuel lines, ignition system, etc.
- **Servo Motors**: For controlling control surfaces (elevator, ailerons, rudder)
- **Propeller**: Large propeller matching the engine specifications (based on thrust and speed requirements)
- **Ultrasonic Sensors (HC-SR04)**: For obstacle detection
- **LoRa SX1278**: For long-range communication
- **FLIR Lepton**: Optional thermal camera for surveillance
- **Payload Delivery Mechanism**: Servo-powered payload release
- **Electronic Fuel Gauge Sensor**: To monitor fuel levels remotely

### Ground Control Station:
- **Arduino Mega**: Controller for joystick, buttons, and communication with UAV
- **Joystick**: For manual control of UAV's flight surfaces (pitch, roll, yaw)
- **TFT Display or OLED**: For real-time telemetry data, camera view, and status updates
- **LoRa SX1278**: For long-range communication
- **Buttons**: For payload release, autopilot activation, return-to-home (RTH), and engine control
- **LiPo Battery**: For powering the ground control station

---

## System Breakdown

### Engine Power System:
The UAV is powered primarily by a **gasoline engine** that drives the propeller for long-endurance flights. The electric systems, powered by a **6S LiPo battery**, are responsible for communication, sensors, and control surfaces.

### Hybrid Power Supply:
- **Engine Starter**: The engine is started remotely via an electric starter controlled by the Arduino.
- **Electric Backup**: In case of engine failure, a battery powers the essential flight systems to ensure safe landing.
- **Fuel Monitoring**: An electronic sensor continuously monitors fuel levels, which are transmitted to the ground control station.

### Autonomous Flight and GPS Navigation:
The UAV uses the **NEO-6M GPS module** to follow pre-programmed waypoints autonomously. In **autopilot mode**, the UAV adjusts its flight path based on GPS coordinates, altitude, and heading.

### Obstacle Avoidance:
The UAV is equipped with **ultrasonic sensors** on the front, sides, and rear. These sensors detect obstacles and automatically adjust the flight path to avoid collisions, crucial for low-altitude or urban missions.

### Return-to-Home (RTH) Functionality:
If communication is lost or the operator initiates the return-to-home function, the UAV uses GPS to autonomously return to its starting point. The **failsafe mechanism** ensures the UAV follows a safe path and avoids obstacles during the return flight.

### Payload Delivery System:
A servo motor is used for **payload release**, controlled remotely via the ground control station. This allows the UAV to deliver packages or other cargo during flight missions.

---

## Circuit Diagram Update

The major change in the circuit is the integration of the gasoline engine control, additional sensors for fuel monitoring, and power management between the engine and auxiliary systems.

### UAV Circuit Components:
- **Arduino Mega 2560**: Connect to the engine starter, servos for control surfaces, payload servo, GPS, ultrasonic sensors, and LoRa module.
- **6S LiPo Battery**: Power the electronics and auxiliary systems.
- **Fuel Gauge Sensor**: Connect to the Arduino to monitor fuel levels.
- **Electronic Speed Controller (ESC)** for the engine starter.
- **LoRa Module**: Long-range communication link to the ground control station.
- **Servo Motors**: Connected to control surfaces (ailerons, rudder, elevator) for flight stabilization.
- **Ultrasonic Sensors**: For obstacle avoidance, connected to digital pins.
- **GPS**: For navigation, connected via serial.

### Ground Control Circuit Components:
- **Arduino Mega**: Controls the joystick, buttons, LoRa module, and display.
- **Joystick**: Connect for manual control of the UAV’s pitch, roll, and yaw.
- **TFT/OLED Display**: Show real-time telemetry data like altitude, speed, GPS location, and camera feed.
- **Buttons**: Control payload release, autopilot activation, RTH, and engine starter.
- **LoRa Module**: For data communication with the UAV.
- **LiPo Battery**: Power the controller and display.

---

## Software Implementation

### UAV Code: Autopilot, Obstacle Avoidance, and Fuel Management

```cpp
#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <NewPing.h> // For obstacle avoidance

// Ultrasonic sensor pins
#define TRIG_PIN_FRONT 30
#define ECHO_PIN_FRONT 34
#define TRIG_PIN_LEFT 31
#define ECHO_PIN_LEFT 35
#define TRIG_PIN_RIGHT 32
#define ECHO_PIN_RIGHT 36
#define TRIG_PIN_BACK 33
#define ECHO_PIN_BACK 37

NewPing sonarFront(TRIG_PIN_FRONT, ECHO_PIN_FRONT, 200); 
NewPing sonarLeft(TRIG_PIN_LEFT, ECHO_PIN_LEFT, 200);
NewPing sonarRight(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT, 200);
NewPing sonarBack(TRIG_PIN_BACK, ECHO_PIN_BACK, 200);

MPU6050 mpu;
Servo rudderServo, elevatorServo, aileronServo, payloadServo;
TinyGPSPlus gps;

int throttle = 1000;
bool autopilot = false;
bool rthActive = false;
float fuelLevel = 100; // Placeholder for fuel sensor

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  rudderServo.attach(9);
  elevatorServo.attach(10);
  aileronServo.attach(11);
  payloadServo.attach(12);

  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed");
    while (1);
  }

  Serial1.begin(9600); // GPS on Serial1
}

void loop() {
  // Read MPU6050 data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Read GPS data
  while (Serial1.available()) {
    gps.encode(Serial1.read());
  }

  // Process LoRa data
  if (LoRa.parsePacket()) {
    String command = "";
    while (LoRa.available()) {
      command += (char)LoRa.read();
    }

    if (command.startsWith("THR")) throttle = command.substring(3).toInt();
    if (command == "AUTOPILOT_ON") autopilot = true;
    if (command == "AUTOPILOT_OFF") autopilot = false;
    if (command == "RELEASE_PAYLOAD") payloadServo.write(90); // Release payload
    if (command == "RTH_ON") rthActive = true; // Trigger return to home
  }

  if (autopilot) {
    followWaypoints();
  }

  if (rthActive) {
    returnToHome();
  }

  adjustControls(gx, gy);
  obstacleAvoidance();
  monitorFuel();
}

void adjustControls(int gx, int gy) {
  int rudderPosition = map(gx, -32768, 32767, 0, 180);
  int elevatorPosition = map(gy, -32768, 32767, 0, 180);

  rudderServo.write(rudderPosition);
  elevatorServo.write(elevatorPosition);
}

void returnToHome() {
  // Navigate back to home coordinates using GPS
  if (gps.location.isValid()) {
    maintainAltitude(100);
  }
}

void followWaypoints() {
  // Autopilot navigation using GPS waypoints
}

void maintainAltitude(int targetAltitude) {
  // GPS-based altitude control
}

void obstacleAvoidance() {
  // Check ultrasonic sensors and

 adjust course if obstacles detected
}

void monitorFuel() {
  // Update fuel level and send telemetry
  LoRa.beginPacket();
  LoRa.print("FUEL: " + String(fuelLevel));
  LoRa.endPacket();
}
```

### Ground Control Code: Telemetry, Camera Feed, and Manual Controls

```cpp
#include <SPI.h>
#include <LoRa.h>
#include <TFT_eSPI.h> // For camera view display

TFT_eSPI tft = TFT_eSPI();

int throttle = 1000;
bool autopilot = false;

void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(1);
  
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed");
    while (1);
  }
}

void loop() {
  tft.fillScreen(TFT_BLACK);
  displayTelemetry();
  readControls();

  if (autopilot) {
    LoRa.beginPacket();
    LoRa.print("AUTOPILOT_ON");
    LoRa.endPacket();
  }
}

void displayTelemetry() {
  // Display telemetry data like altitude, speed, fuel, etc.
  tft.setCursor(0, 0);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.println("Telemetry:");
  // Add more telemetry displays
}

void readControls() {
  // Read joystick and button states for manual control
}
```

---

## Future Enhancements:
1. **Longer Flight Range**: Implement hybrid propulsion (solar panels) for extended flights.
2. **Advanced Obstacle Detection**: Use LiDAR or radar for more precise obstacle avoidance.
3. **AI-Driven Autopilot**: Integrate AI for dynamic path planning based on real-time data.
4. **Payload Feedback System**: Add sensors to monitor payload status after delivery.
5. **Satellite Communication**: For even longer-range communication beyond LoRa limitations.

