# Obstacle Detection and avoidance Robot using AVR microcontroller ATMEGA328P in Embedded C

Introduction:

The field of robotics has witnessed advancements, with the time in autonomous car. The development of obstacle detection and avoidance systems for driverless cars is one of the main applications. This report describes the design and development of an Arduino-based obstacle detection and avoidance car. The goal of this project is to develop an autonomous car that can intelligently recognize and avoid obstacles while navigating through its surroundings.
For this course work-mini project I used Ultra-sonic Sensor, Servo, Led, Buzzer, Potentiometer, DC motors and L298N H-bridge motor driver.

Components used in project:

•	1 x Arduino Uno (AtMega328p)
•	1 x Breadboard
•	1 x Ultra-sonic Sensor
•	1 x SG-90 Servo motor
•	1 x L298N H-bridge motor driver
•	1 x Buzzer
•	1 x Potentiometer
•	1 x 9v Battery
•	2 x Led
•	2 x DC motors
•	2 x wheels

<img width="451" alt="image" src="https://github.com/noor-akhunji/Obstacle-Detection-Robot-using-AVR-microcontroller-ATMEGA328P-in-Embedded-C/assets/84890896/4b9dcb68-46a6-448c-b6d4-ac21e79428d8">


Component	Module pin	AVR pin	Description
HC-SR04 Ultrasonic	Trig pin	PD2	Trig pin of ultrasonic sensor
	Echo pin	PD3	Echo pin of ultrasonic sensor
Motors	In1	PD4	Left motor first pin MLa 
	In2	PD5	Left motor second pin MLb
	In3	PD6	Right motor first pin MRa
	In4	PD7	Right motor second pin MLb
Buzzer	(+) pin	PB3	Buzzer pin for obstacle detection function
LED	(+) pin	PB5	Led Pin for obstacle detection function
Potentiometer	Signal pin	PC0	ADC value set pin for set brightness of Led
LED (Potentiometer ADC)	(+) pin	PB2	ADC output Led
Servo Motor	Signal pin	PB1	Servo motor performing rotate 0 to 180 degree
![image](https://github.com/noor-akhunji/Obstacle-Detection-Robot-using-AVR-microcontroller-ATMEGA328P-in-Embedded-C/assets/84890896/eb401edb-5dfa-49f7-ae63-eba3fe0d16dc)
