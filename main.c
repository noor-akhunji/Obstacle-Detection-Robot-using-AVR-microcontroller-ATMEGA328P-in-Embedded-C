
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sfr_defs.h>
#include <avr/common.h>
#include <avr/wdt.h>

// Define pin configurations
#define trigPin PD2  // Trig Pin Of HC-SR04
#define echoPin PD3  // Echo Pin Of HC-SR04
#define MLa PD4      // Left motor 1st pin
#define MLb PD5      // Left motor 2nd pin
#define MRa PD6      // Right motor 1st pin
#define MRb PD7      // Right motor 2nd pin
#define buzzerPin PB3 // Buzzer pin
#define ledPin PB5    // LED pin
#define knob_pin PC0  // A0 pin potentiometer
#define led_pin_pot PB2 // Pin led ADC
#define servoPin PB1	// servo pin 

// Variables for ultrasonic sensor
volatile long duration, distance;

// Function prototypes
void initADC();
uint16_t readADC(uint8_t channel);
void initTimer0();
void initTimer1();
void initUSART();
void transmitUSART(unsigned char data);
void transmitDistance();
void initInterrupt();
void setBrightness(uint8_t brightness);
void setServoAngle(uint8_t angle);
void init();

// External interrupt service routine for Echo pin
ISR(INT1_vect) {
	if (PIND & (1 << echoPin)) {
		// Rising edge
		TCNT1 = 0; // Reset Timer1 counter
		} else {
		// Falling edge
		duration = TCNT1 / 2; // Calculate distance in cm
		distance = duration / 29.1;
	}
}

// Initialize ADC for potentiometer reading
void initADC() {
	// Set the reference voltage to AVcc
	ADMUX |= (1 << REFS0);
	// Set prescaler to 128 (ADPS2, ADPS1, ADPS0 bits)
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	// Enable ADC
	ADCSRA |= (1 << ADEN);
}

// Read ADC value from the specified channel
uint16_t readADC(uint8_t channel) {
	// Select ADC channel
	ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
	// Start single conversion
	ADCSRA |= (1 << ADSC);
	// Wait for conversion to complete
	while (ADCSRA & (1 << ADSC));
	// Return ADC result
	return ADC;
}

// Initialize Timer0 for PWM control of LED
void initTimer0() {
	// Set up Timer0 in PWM mode for LED control
	TCCR0A |= (1 << WGM00) | (1 << WGM01) | (1 << COM0A1);
	TCCR0B |= (1 << CS01) | (1 << CS00); // Set prescaler to 64
}

// Initialize Timer1 for PWM control of servo
void initTimer1() {
	// Set up Timer1 in PWM mode for servo control
	TCCR1A |= (1 << WGM11) | (1 << COM1A1);
	TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11) | (1 << CS10); // Set prescaler to 64
}

// Initialize USART for serial communication
void initUSART() {
	// Set baud rate to 9600
	UBRR0H = (unsigned char)(103 >> 8);
	UBRR0L = (unsigned char)103;
	// Enable receiver and transmitter
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	// Set frame format: 8 data, 1 stop bit
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Transmit a character over USART
void transmitUSART(unsigned char data) {
	// Wait for empty transmit buffer
	while (!(UCSR0A & (1 << UDRE0)));
	// Put data into buffer, sends the data
	UDR0 = data;
}

// Transmit distance over USART
void transmitDistance() {
	char buffer[10];
	itoa(distance, buffer, 10);
	for (int i = 0; buffer[i] != '\0'; i++) {
		transmitUSART(buffer[i]);
	}
	transmitUSART('\n');
}

// Initialize external interrupt for Echo pin
void initInterrupt() {
	// Enable external interrupt for Echo pin (PD3)
	EICRA |= (1 << ISC11); // Falling edge generates interrupt
	EIMSK |= (1 << INT1);  // Enable external interrupt 1
}

// Set LED brightness based on potentiometer value
void setBrightness(uint8_t brightness) {
	OCR0A = brightness;
}

// Set servo angle based on potentiometer value
void setServoAngle(uint8_t angle) {
	OCR1A = (angle * 10) + 1000;
}

// Initialize all components
void init() {
	// Set up ports for output
	DDRB |= (1 << MLa) | (1 << MLb) | (1 << MRa) | (1 << MRb) | (1 << buzzerPin) | (1 << ledPin) | (1 << led_pin_pot);
	DDRD |= (1 << trigPin) | (1 << servoPin);
	// Set up port for input
	DDRC &= ~(1 << echoPin);

	// Initialize components
	initADC();
	initTimer0();
	initTimer1();
	initUSART();
	initInterrupt();

	// Enable global interrupts
	sei();
}

int main() {
	// Initialize components
	init();

	while (1) {
		// Ultrasonic sensor
		PORTD |= (1 << trigPin);
		_delay_us(10);
		PORTD &= ~(1 << trigPin);

		// Potentiometer
		uint16_t val = readADC(0);
		uint8_t brightness = val / 4;
		setBrightness(brightness);

		// if distance is > 35 then car moves forward 
		if (distance > 35) {
			// No obstacle detected
			PORTB &= ~((1 << buzzerPin) | (1 << ledPin));
			
			// set servo in front middle to detect obstacle infront
			setServoAngle(90);
			
			// move forward
			PORTD |= (1 << MRa) | (1 << MLa);
			PORTD &= ~((1 << MLb) | (1 << MRb));

			// if distance is < 30 then car stops and change direction
			} else if (distance < 30 && distance > 0) {
			
			// Obstacle detected turn on buzzer and led
			PORTB |= (1 << buzzerPin) | (1 << ledPin);

			// stop motors
			PORTD &= ~((1 << MRb) | (1 << MLa) | (1 << MLb) | (1 << MRa)); 
			_delay_ms(100);

			// move servo for checking obstacle from 0 to 180
			setServoAngle(0);
			_delay_ms(500);
			setServoAngle(180);
			_delay_ms(500);
			setServoAngle(90);
			_delay_ms(500);
			
			// move backward
			PORTD |= (1 << MRa) | (1 << MLa);
			PORTD &= ~((1 << MRb) | (1 << MLb));
			_delay_ms(500);
			
			// stop
			PORTD &= ~((1 << MRb) | (1 << MLa) | (1 << MLb) | (1 << MRa));
			_delay_ms(500);
			
			// move left
			PORTD |= (1 << MRb);
			PORTD &= ~((1 << MLa) | (1 << MLb) | (1 << MRa));
			_delay_ms(200);

			// Off blinking LED, buzzing sound
			PORTB &= ~((1 << buzzerPin) | (1 << ledPin));
		}

		_delay_ms(10);
	}

	return 0;
}
