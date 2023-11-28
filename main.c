#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define TRIG_PIN PD3
#define ECHO_PIN PD2
#define ML_A PD4
#define ML_B PD5
#define MR_A PD6
#define MR_B PD7
#define BUZZER_PIN PB3
#define LED_PIN PB5
#define SERVO_PIN PB1

volatile unsigned long duration;
volatile unsigned long distance;

void initUART() {
    // Set baud rate to 9600
    UBRR0H = 0;
    UBRR0L = 103;

    // Enable receiver and transmitter
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);

    // Set frame format: 8 data bits, 1 stop bit
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void initPWM() {
    // Set up Timer1 for PWM on SERVO_PIN (PB1)
    TCCR1A |= (1 << COM1A1) | (1 << WGM11) | (1 << WGM10);
    TCCR1B |= (1 << WGM13) | (1 << CS11);

    // Set SERVO_PIN (PB1) as output
    DDRB |= (1 << SERVO_PIN);
}

void initIO() {
    // Set motor control pins as output
    DDRC |= (1 << ML_A) | (1 << ML_B) | (1 << MR_A) | (1 << MR_B);

    // Set TRIG_PIN as output
    DDRD |= (1 << TRIG_PIN);

    // Set ECHO_PIN as input
    DDRD &= ~(1 << ECHO_PIN);

    // Set BUZZER_PIN and LED_PIN as output
    DDRB |= (1 << BUZZER_PIN) | (1 << LED_PIN);
}

void initInterrupt() {
    // Enable external interrupt on ECHO_PIN (PD2)
    EIMSK |= (1 << INT0);

    // Falling edge triggers interrupt
    EICRA |= (1 << ISC01);
}

void init() {
    initIO();
    initPWM();
    initUART();
    initInterrupt();
    sei(); // Enable global interrupts
}

void sendString(const char *str) {
    while (*str != '\0') {
        // Wait for empty transmit buffer
        while (!(UCSR0A & (1 << UDRE0)));

        // Put data into buffer, sends the data
        UDR0 = *str;

        str++;
    }
}

void sendDistance() {
    char buffer[10];
    itoa(distance, buffer, 10);
    sendString(buffer);
    sendString(" cm\r\n");
}

void moveForward() {
    PORTC = (1 << ML_B) | (1 << MR_B);
}

void stop() {
    PORTC = 0;
}

void moveBackward() {
    PORTC = (1 << ML_A) | (1 << MR_A);
}

void moveLeft() {
    PORTC = (1 << MR_B) | (1 << ML_A);
}

ISR(INT0_vect) {
    if (PIND & (1 << ECHO_PIN)) {
        // Rising edge, record the time
        TCNT1 = 0; // Reset Timer1
    } else {
        // Falling edge, calculate the duration
        duration = TCNT1;
        distance = (duration * 0.0343) / 2; // Speed of sound is approximately 343 meters/second

        // Perform obstacle avoidance logic
        if (distance > 15) {
            // No obstacle detected
            PORTB &= ~((1 << BUZZER_PIN) | (1 << LED_PIN));
            moveForward();
        } else if (distance > 0 && distance < 10) {
            // Obstacle detected
            PORTB |= (1 << BUZZER_PIN) | (1 << LED_PIN);
            stop();

            OCR1A = 150; // Move servo to 90 degrees
            _delay_ms(500);
            OCR1A = 600; // Move servo to 0 degrees
            _delay_ms(500);
            OCR1A = 300; // Move servo to 180 degrees
            _delay_ms(500);
            OCR1A = 150; // Move servo back to 90 degrees

            moveBackward();
            _delay_ms(500);
            stop();
            _delay_ms(100);

            moveLeft();
            _delay_ms(500);
        }
    }
}

int main(void) {
    init();

    while (1) {
        sendDistance();
        _delay_ms(10);
    }

    return 0;
}
