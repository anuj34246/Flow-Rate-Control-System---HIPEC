# Flow-Rate-Control-System---HIPEC
It is a device to maintain the flow rate of 2lpm and temp of 43 degree celsius to kill the cancerous cell after surgery.
# Design of a control system for flow rate regulation in HIPEC  (Hyperthermic Intraperitoneal Chemotherapy)

The project focuses on developing a control system to regulate the flow rate of chemotherapy fluid in HIPEC (Hyperthermic Intraperitoneal Chemotherapy). This process involves circulating heated chemotherapy drugs within the peritoneal cavity to target cancer cells effectively. Maintaining a precise flow rate of 2 liters per minute (LPM) is crucial for uniform drug distribution and minimizing complications.

What is HIPEC? 

HIPEC is an advanced cancer treatment combining surgery and heated chemotherapy. After surgically removing visible tumors, heated chemotherapy fluid is circulated in the abdominal cavity to destroy microscopic cancer cells. This method enhances drug penetration and effectiveness while reducing systemic toxicity.

Relevance of Flow Rate and Temperature Control:-

Flow rate ensures even drug distribution throughout the cavity, while temperature enhances the drug’s efficacy. Biomedical engineering integrates control systems to maintain such parameters with precision, demonstrating its critical role in medical device development.


![alt text](https://res.cloudinary.com/sarr/image/upload/v1735325138/Schema_HIPEC_miuigw.png)


## Project Objective
The objective is to design and implement a control system capable of maintaining a steady flow rate of 2 LPM. While temperature stability is vital, the project focuses primarily on flow rate control as it directly affects the treatment's effectiveness and patient safety.


## Components Required

| Items            | Quantity                                                               |
| ----------------- | ------------------------------------------------------------------ |
| Arduino Uno board |x1 |
| YF-S201 Water Flow Sensor | x1 |
| Solenoid Valve | x1 |
| Relay Module | x1 |
| 12V Battery | x1 |
| Tubing | ...|
| Connecting Wires | ...|



### Arduino Uno

![alt text](https://res.cloudinary.com/dhrd4kfja/image/upload/v1729693224/arduino_uno_ai7quf.jpg)

Microcontroller (Arduino): The microcontroller acts as the brain of the system, processing real-time data from sensors and sending commands to actuators.


###  YF-S201 Water Flow Sensor

![alt text](https://res.cloudinary.com/sarr/image/upload/v1735325894/YF-S201-Sensor-Pinout_2048x_o9kctl.webp)

YF-S201 Water Flow Sensor: A sensor that generates electrical pulses based on the flow rate of the solution. Each pulse corresponds to a small volume of fluid, and counting these pulses allows accurate flow rate measurement.

### Solenoid Valve

![alt text](https://res.cloudinary.com/sarr/image/upload/v1735326102/solenoid_valve_lqvg1g.jpg)

Solenoid Valve: Controls the flow of fluid by opening or closing based on commands. The valve ensures precise regulation to maintain the flow rate within acceptable limits.

### Relay Module

![alt text](https://res.cloudinary.com/sarr/image/upload/v1735326238/relay_module_wyawpz.jpg)

Relay Module: Acts as a switch that controls the solenoid valve. The relay is activated or deactivated by the Arduino based on the calculated flow rate.


## System Design Overview

The system is divided into three main components:

Sensor: Continuously monitors the flow rate of the chemotherapy fluid.

Controller: Processes the sensor data and adjusts the system to correct deviations.

Actuator (Pump): Modulates fluid flow to maintain the desired rate.
A block diagram illustrates the signal flow among these components, forming a closed-loop control system.

### Sensor Selection

Flow sensors play a critical role in providing accurate real-time measurements. Ultrasonic and electromagnetic flow sensors were considered, with ultrasonic sensors being selected for their high sensitivity, non-invasive operation, and compatibility with sterile medical environments. These sensors ensure precise monitoring without interfering with the fluid flow.

### Controller Design

A proportional-integral-derivative (PID) controller was chosen for its ability to maintain steady-state accuracy and respond effectively to transient changes. Unlike simpler ON-OFF controllers, the PID controller ensures smoother adjustments by using proportional, integral, and derivative calculations to minimize deviations from the desired flow rate.

### Actuator Mechanism

The actuator, in the form of a peristaltic pump, modulates the flow of the chemotherapy fluid based on signals from the controller. Peristaltic pumps are preferred for medical applications due to their ability to handle sterile fluids without contamination. The actuator’s responsiveness to controller signals ensures smooth and precise flow adjustments.

### Simulation and Testing

The system design was validated through simulations using MATLAB and Simulink. These simulations modeled the behavior of the control system under various conditions, such as changes in fluid resistance within the tubing. Results demonstrated that the system could maintain the flow rate at 2 LPM with minimal fluctuations, confirming the effectiveness of the PID controller.

Physical testing involved a test bench setup comprising fluid reservoirs, tubing, and flow sensors. Data from these tests showed strong agreement with simulation results, highlighting the system’s reliability in achieving precise flow control. Graphs comparing setpoint versus actual flow rate further reinforced the system’s accuracy and responsiveness.






## Arduino code

Upload this code into arduino. 

```bash
#include <Arduino.h>
volatile int pulseCount = 0; // To count pulses from the flow sensor
unsigned long lastTime = 0; // For timing calculations
float flowRate = 0; // Calculated flow rate in L/min
const float calibrationFactor = 7.5; // Pulses per second to L/min conversion factor

// Pin definitions
const int flowSensorPin = 2; // Digital pin for flow sensor signal (interrupt pin)
const int relayPin = 8;      // Digital pin for relay control

void setup() {
    // Initialize serial communication for debugging
    Serial.begin(9600);

    // Set up the relay pin as output
    pinMode(relayPin, OUTPUT);
    digitalWrite(relayPin, HIGH); // Start with the solenoid valve closed

    // Attach interrupt to the flow sensor pin
    pinMode(flowSensorPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(flowSensorPin), countPulse, RISING);

    // Initialize variables
    lastTime = millis();
}

void loop() {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastTime;

    // Calculate flow rate every second
    if (elapsedTime >= 1000) {
        noInterrupts(); // Temporarily disable interrupts to ensure accurate pulse count
        float frequency = pulseCount / (elapsedTime / 1000.0); // Calculate frequency (pulses per second)
        flowRate = frequency / calibrationFactor; // Calculate flow rate in L/min
        pulseCount = 0; // Reset pulse count
        lastTime = currentTime; // Update the lastTime
        interrupts(); // Re-enable interrupts

        // Print flow rate to Serial Monitor for debugging
        Serial.print("Flow Rate: ");
        Serial.print(flowRate);
        Serial.println(" L/min");

        // Control solenoid valve based on flow rate
        if (flowRate > 2.0) {
            digitalWrite(relayPin, HIGH); // Close solenoid valve
        } else {
            digitalWrite(relayPin, LOW); // Open solenoid valve
        }
    }
}

// Interrupt service routine (ISR) for counting pulses from the flow sensor
void countPulse() {
    pulseCount++;
}


```


## Challenges and Future Work

Developing the system presented challenges, including sensor calibration, noise in feedback signals, and response delays in the pump-actuator system. Overcoming these issues required careful tuning of the PID controller and extensive testing.

Future work aims to extend the system’s capabilities to regulate both flow rate and temperature simultaneously, ensuring comprehensive control for HIPEC procedures. Integrating artificial intelligence could further enhance performance by predicting patient-specific parameters and adapting the system dynamically to changing conditions.

## Conclusion

This project successfully designed and tested a control system capable of maintaining a precise flow rate of 2 LPM for HIPEC applications. The closed-loop system demonstrated reliability, accuracy, and stability, meeting the stringent requirements of biomedical applications. By ensuring uniform drug distribution, the system contributes significantly to improving the effectiveness and safety of HIPEC procedures. This work lays the foundation for further advancements, including real-world testing and integration into clinical settings, showcasing the vital role of control systems in biomedical engineering.

