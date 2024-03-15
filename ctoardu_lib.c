#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdlib.h>
#include <avr/interrupt.h>

//#define F_CPU 16000000UL // CPU Frequency
#define TMR1_PS256x 0x04
#define TMR1_PS256 0x04


#define USART_BAUDRATE 9600 // Baud Rate
#define BAUD_PRESCALER (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#define ASYNCHRONOUS (0<<UMSEL00) // USART Mode Selection

#define DISABLED    (0<<UPM00)
#define EVEN_PARITY (2<<UPM00)
#define ODD_PARITY  (3<<UPM00)
#define PARITY_MODE  DISABLED // USART Parity Bit Selection

#define ONE_BIT (0<<USBS0)
#define TWO_BIT (1<<USBS0)
#define STOP_BIT ONE_BIT      // USART Stop Bit Selection

#define FIVE_BIT  (0<<UCSZ00)
#define SIX_BIT   (1<<UCSZ00)
#define SEVEN_BIT (2<<UCSZ00)
#define EIGHT_BIT (3<<UCSZ00)
#define DATA_BIT   EIGHT_BIT  // USART Data Bit Selection


// GPIO pins 
#define PORT_SIZE 8

#if defined( __AVR_ATmega324A__)

#define PA 37 // PA0 address
#define PB 40 // PB(0-4)
#define PB_5 1 // PB(5-7)
#define PC 19
#define PD 9 
#define PB_OFF 4

#elif defined(__AVR_ATmega328P__)

#define PB 8 // PB(0-5)
#define PC 14 // PC(0-5)
#define PD 0 // PD(0-7)
#define NEGATED_LOGIC 255
#endif

enum Color {
    WHITE,
    RED,
    RED_ORANGE,
    ORANGE,
    YELLOW_ORANGE,
    YELLOW,
    YELLOW_GREEN,
    GREEN,
    BLUE_GREEN,
    BLUE,
    BLUE_VIOLET,
    VIOLET,
    RED_VIOLET
    

};
enum Mode {
    INPUT,
    OUTPUT,
    INPUT_PULLUP     
};
enum Value{
    LOW,
    HIGH
};

#if defined(__AVR_ATmega324A__)

void pinMode(int8_t pin, enum Mode value) {
    if (pin >= PB_5 && pin <= PB_5 + 3) { // DDRB(5-7)
        if (value == INPUT) {
            DDRB &= ~(1 << (pin + PB_OFF)); // clear
        }
        else if (value == OUTPUT){
            DDRB |= (1 << (pin + PB_OFF)); // set
        }
        else if (value == INPUT_PULLUP) {
            DDRB &= ~(1 << (pin + PB_OFF)); // clear
            PORTB |= (1 << (pin + PB_OFF));  // input_pullup
        }
    }
    else 
    if (pin >= PB && pin <= PB + PB_OFF) { // DDRB(0-4)
        if (value == INPUT) {
            DDRB &= ~(1 << (pin - PB)); 
        }
        else if (value == OUTPUT){
            DDRB |= (1 << (pin - PB)); 
        }
        else if (value == INPUT_PULLUP) {
            DDRB &= ~(1 << (pin - PB)); 
            PORTB |= (1 << (pin - PB));  
        }
    } 
    else if (pin >= PD && pin <= PD + PORT_SIZE) { // DDRD
        if (value == INPUT) {
            DDRD &= ~(1 << (pin - PD)); 
        }
        else if (value == OUTPUT){
            DDRD |= (1 << (pin - PD)); 
        }
        else if (value == INPUT_PULLUP) {
            DDRD &= ~(1 << (pin - PD)); 
            PORTD |= (1 << (pin - PD)); 
        }
    }
    else if (pin >= PC && pin <= PC + PORT_SIZE) { // DDRC
        if (value == INPUT) {
            DDRC &= ~(1 << (pin - PC)); 
        }
        else if (value == OUTPUT){
            DDRC |= (1 << (pin - PC)); 
        }
        else if (value == INPUT_PULLUP) {
            DDRC &= ~(1 << (pin - PC)); 
            PORTC |= (1 << (pin - PC));  
        }
    }
    else if (pin >= PA - PORT_SIZE && pin <= PA) { // DDRA
        if (value == INPUT) {
            //DDRA &= ~(1 << (PA - pin));
        }
        else if (value == OUTPUT) {
          //  DDRA |= (1 << (PA - pin)); 
        }
        else if (value == INPUT_PULLUP) {
            // DDRA &= ~(1 << (PA - pin)); 
            // PORTA |= (1 << (PA - pin));  
        }
    }
}
void digitalWrite(int8_t pin, enum Value value) {
    if (pin >= PB_5 && pin <= PB_5 + 3) { // PORTB(5-7)
        if (value == LOW) {
            PORTB &= ~(1 << (pin + PB_OFF)); // Low
        }
        else if (value == HIGH){
            PORTB |= (1 << (pin + PB_OFF)); // HIGH
        }
        
    }
    else 
    if (pin >= PB && pin <= PB + PB_OFF) { // PORTB(0-4)
        if (value == LOW) {
            PORTB &= ~(1 << (pin - PB)); 
        }
        else if (value == HIGH){
            PORTB |= (1 << (pin - PB)); 
        }
       
    } 
    else if (pin >= PD && pin <= PD + PORT_SIZE) { // PORTD
        if (value == LOW) {
            PORTD &= ~(1 << (pin - PD)); 
        }
        else if (value == HIGH){
            PORTD |= (1 << (pin - PD)); 
        }
        
    }
    else if (pin >= PC && pin <= PC + PORT_SIZE) { // PORTC
        if (value == LOW) {
            PORTC &= ~(1 << (pin - PC)); 
        }
        else if (value == HIGH){
            PORTC |= (1 << (pin - PC)); 
        }
    }
    else if (pin >= PA - PORT_SIZE && pin <= PA) { // PORTA
        if (value == LOW) {
          //  PORTA &= ~(1 << (PA - pin));
        }
        else if (value == HIGH) {
           // PORTA |= (1 << (PA - pin)); 
        }
    }
}
int8_t digitalRead(int8_t pin) {
    if (pin >= PB_5 && pin <= PB_5 + 3) { // PINB(5-7)
          return PINB & (1 << (pin + PB_OFF));                        
    }
    else if (pin >= PB && pin <= PB + PB_OFF) { // PINB(0-4)
           return PINB & (1 << (pin - PB)); 
    } 
    else if (pin >= PD && pin <= PD + PORT_SIZE) { // PIND
           return PIND & (1 << (pin - PD)); 
    }
    else if (pin >= PC && pin <= PC + PORT_SIZE) { // PINC
           return PINC & (1 << (pin - PC)); 
    }
    else if (pin >= PA - PORT_SIZE && pin <= PA) { // PINA
           // return PINA & (1 << (PA - pin));
    }
    return -1; // error
}

//  void set_rgb(enum Color color) {

//     // PD5 - pin 14 - red
//     // PD7 - pin 16 - green
//     // PB3 - pin 43 - blue
//     int8_t red = 14;
//     int8_t green = 16;
//     int8_t blue = 43;

//     switch (color) {
//         case RED:
//             pinMode(red, OUTPUT);
//             digitalWrite(red, HIGH);
//             pinMode(green, OUTPUT);
//             digitalWrite(green, LOW);
//             pinMode(blue, OUTPUT);
//             digitalWrite(blue, LOW);
//         break;
//         case ORANGE:
            
//             pinMode(red, OUTPUT);
//             digitalWrite(red, HIGH);
//             analogWrite(green, 150);
//             pinMode(blue, OUTPUT);
//             digitalWrite(blue, LOW);
//         break;
//         case YELLOW:
//             pinMode(red, OUTPUT);
//             digitalWrite(red, HIGH);
//             pinMode(green, OUTPUT);
//             digitalWrite(green, HIGH);
//             pinMode(blue, OUTPUT);
//             digitalWrite(blue, LOW);

//         break;
//         case YELLOW_GREEN:
//             analogWrite(red, 13);
//             analogWrite(green, 152);
//             analogWrite(blue, 186);
//         break;
//         case GREEN:
//             pinMode(red, OUTPUT);
//             digitalWrite(red, LOW);
//             pinMode(green, OUTPUT);
//             digitalWrite(green, HIGH);
//             pinMode(blue, OUTPUT);
//             digitalWrite(blue, LOW);
//         break;

//         case BLUE_GREEN:
//             analogWrite(red, 0);
//             analogWrite(green, 255);
//             analogWrite(blue, 0);
//         break;

//         case BLUE:
//             pinMode(red, OUTPUT);
//             digitalWrite(red, LOW);
//             pinMode(green, OUTPUT);
//             digitalWrite(green, LOW);
//             pinMode(blue, OUTPUT);
//             digitalWrite(blue, HIGH);
//         break;

//         case BLUE_VIOLET:
//             analogWrite(red, 138);
//             analogWrite(green, 42);
//             analogWrite(blue, 226);

//         break;
//         case VIOLET:
//             analogWrite(red, 238);
//             analogWrite(green, 130);
//             analogWrite(blue, 28);
//         break;
//         case RED_VIOLET:
//             analogWrite(red, 199);
//             analogWrite(green, 21);
//             analogWrite(blue, 133);
//         case WHITE:
//             pinMode(red, OUTPUT);
//             digitalWrite(red, HIGH);
//             pinMode(green, OUTPUT);
//             digitalWrite(green, HIGH);
//             pinMode(blue, OUTPUT);
//             digitalWrite(blue, HIGH);
//         break;
        
//         default:
//             pinMode(red, OUTPUT);
//             digitalWrite(red, LOW);
//             pinMode(green, OUTPUT);
//             digitalWrite(green, LOW);
//             pinMode(blue, OUTPUT);
//             digitalWrite(blue, LOW);
//     }
    
// }

void turn_on_rgb(enum Color color) {
        
    switch(color) {   
        case RED:
            PORTD &= ~(1 << PD7);
        break;
        case GREEN:
            PORTB &= ~(1 << PB3);
        break;
        case BLUE:
            PORTD &= ~(1 << PD5);
        break;
        default:
            PORTD |= (1 << PD7);
            PORTD |= (1 << PD5);
            PORTB |= (1 << PB3);
    }
}

void turn_off_rgb(enum Color color) {
    switch(color) {   
        case RED:
            PORTD |= (1 << PD7);
        break;
        case GREEN:
            PORTB |= (1 << PB3);
        break;
        case BLUE:
            PORTD |= (1 << PD5);
        break;
        default:
            PORTD |= (1 << PD7);
            PORTD |= (1 << PD5);
            PORTB |= (1 << PB3);
    }
}

#elif defined(__AVR_ATmega328P__)

void pinMode(int8_t pin, enum Mode value) {
   
    if (pin >= PB && pin < PB + PORT_SIZE) { // DDRB(0-5)
        if (value == INPUT) {
            DDRB &= ~(1 << (pin - PB)); 
        }
        else if (value == OUTPUT){
            DDRB |= (1 << (pin - PB)); 
        }
        else if (value == INPUT_PULLUP) {
            DDRB &= ~(1 << (pin - PB)); 
            PORTB |= (1 << (pin - PB));  
        }
    } 
    else if (pin >= PD && pin < PD + PORT_SIZE) { // DDRD
        if (value == INPUT) {
            DDRD &= ~(1 << (pin - PD)); 
        }
        else if (value == OUTPUT){
            DDRD |= (1 << (pin - PD)); 
        }
        else if (value == INPUT_PULLUP) {
            DDRD &= ~(1 << (pin - PD)); 
            PORTD |= (1 << (pin - PD)); 
        }
    }
    else if (pin >= PC && pin < PC + PORT_SIZE) { // DDRC
        if (value == INPUT) {
            DDRC &= ~(1 << (pin - PC)); 
        }
        else if (value == OUTPUT){
            DDRC |= (1 << (pin - PC)); 
        }
        else if (value == INPUT_PULLUP) {
            DDRC &= ~(1 << (pin - PC)); 
            PORTC |= (1 << (pin - PC));  
        }
    }
}

void digitalWrite(int8_t pin, enum Value value) {
    
    if (pin >= PB && pin < PB + PORT_SIZE) { // PORTB(0-5)
        if (value == LOW) {
            PORTB &= ~(1 << (pin - PB)); 
        }
        else if (value == HIGH){
            PORTB |= (1 << (pin - PB)); 
        }
       
    } 
    else if (pin >= PD && pin < PD + PORT_SIZE) { // PORTD
        if (value == LOW) {
            PORTD &= ~(1 << (pin - PD)); 
        }
        else if (value == HIGH){
            PORTD |= (1 << (pin - PD)); 
        }
        
    }
    else if (pin >= PC && pin < PC + PORT_SIZE) { // PORTC
        if (value == LOW) {
            PORTC &= ~(1 << (pin - PC)); 
        }
        else if (value == HIGH){
            PORTC |= (1 << (pin - PC)); 
        }
    }
  
}
int8_t digitalRead(int8_t pin) {
    
    if (pin >= PB && pin < PB + PORT_SIZE) { // PINB(0-5)
           return PINB & (1 << (pin - PB)); 
    } 
    else if (pin >= PD && pin < PD + PORT_SIZE) { // PIND
           return PIND & (1 << (pin - PD)); 
    }
    else if (pin >= PC && pin < PC + PORT_SIZE) { // PINC(0-5)
           return PINC & (1 << (pin - PC)); 
    }
    
    return -1; // error
}

#endif




 



/*

PWM pins on atmega328p:
    OC0A - PD6 - 6
    OC0B - PD5 - 5
    OC1A - PB1 - 9
    OC1B - PB2 - 10
    OC2A - PB3 - 11
    OC2B - PD3 - 3

PWM pins on atmega324p:
    OC0A - PB3 - 43
    OC0B - PB4 - 44
    OC1A - PD5 - 14
    OC1B - PD4 - 13
    OC2A - PD7 - 16
    OC2B - PD6 - 15

*/


// NOTE: Do not use OC1A and OC1B the same time with timer 1 function 
// Pin 9 and 10 in atmega328p
void pwm_init(int8_t pin) {

    if (pin == 9 || pin == 10) {
    // Timer0
        TCCR1A |= (1 << WGM00) | (1 << WGM01); // Fast PWM
        TCCR1A |= (1 << COM0A1) | (1 << COM0B1); // Clear OC0A/OC0B on compare match, set OC0A/OC0B at BOTTOM
        TCCR1B |= (1 << CS00); // no prescaler
        
        if (pin == 9) {
            DDRB |= (1 << PB1);
        }
        else if (pin == 10) {
            DDRB |= (1 << PB2);
        }
    }
    if (pin == 6 || pin == 5) {
    // Timer1
        TCCR0A |= (1 << WGM10) | (1 << WGM11); // Fast PWM
        TCCR0A |= (1 << COM1A1) | (1 << COM1B1); // Clear OC1A/OC1B on compare match, set OC1A/OC1B at BOTTOM
        TCCR0B |= (1 << CS10); // No prescaler
        if (pin == 6) {
            DDRD |= (1 << PD6);
        }
        else if (pin == 5) {
            DDRD |= (1 << PD5);
        }
    }
    if (pin == 11 || pin == 3) {
    // Timer2
        TCCR2A |= (1 << WGM20) | (1 << WGM21); // Fast PWM
        TCCR2A |= (1 << COM2A1) | (1 << COM2B1); // Clear OC2A/OC2B on compare match, set OC2A/OC2B at BOTTOM
        TCCR2B |= (1 << CS20); // No prescaler
       if(pin == 11) {
            DDRB |= (1 << PB3);
        }
        else if (pin == 3) {
            DDRD |= (1 << PD3);
        }
    } 
}

#if defined(__AVR_ATmega324A__)

void analogWrite(int8_t pin, uint8_t value) {
    switch (pin) {
        case 16:
            OCR2A = value;
        break;
        case 44:
            OCR0B = value;
        break;
        case 43:
            OCR0A = value;
        break;
        case 14:
            OCR1A = value;
        break;
        case 13:
            OCR1B = value;
        break;
        case 15:
            OCR2B = value;
        break;
        default: 
        break;
    }
}

#elif defined(__AVR_ATmega328P__)

void analogWrite(int8_t pin, uint8_t value) {

    switch (pin) {
        case 9:
            OCR1A = value;
        break;
        case 10:
            OCR1B = value;
        break;
        case 11:
            OCR2A = value;
        break;
        case 6:
            OCR0A = value;
        break;
        case 5:
            OCR0B = value;
        break;
        case 3:
            OCR2B = value;
        break;
        default: 
            TCCR0A = 0;
        //     TCCR1A = 0; now used by timer 1 millis
            TCCR2A = 0;
        break;
    }

    pwm_init(pin);
}

#endif


void set_rgb(enum Color color) {

    // PD5 - pin 14 - red
    // PD7 - pin 16 - green
    // PB3 - pin 43 - blue
    int8_t red = 11;
    int8_t green = 10;
    int8_t blue = 9;

    switch (color) {
        case RED:
            analogWrite(red, NEGATED_LOGIC - 255);
            analogWrite(green, NEGATED_LOGIC - 0);
            analogWrite(blue, NEGATED_LOGIC - 0);
            
        break;
        case RED_ORANGE:
            analogWrite(red, NEGATED_LOGIC - 255);
            analogWrite(green, NEGATED_LOGIC - 83);
            analogWrite(blue, NEGATED_LOGIC - 73);
        break;
        case ORANGE:
            analogWrite(red, NEGATED_LOGIC - 255);
            analogWrite(green, NEGATED_LOGIC - 150);
            analogWrite(blue, NEGATED_LOGIC - 0);
        break;

        case YELLOW_ORANGE:
            analogWrite(red, NEGATED_LOGIC - 255);
            analogWrite(green, NEGATED_LOGIC - 174);
            analogWrite(blue, NEGATED_LOGIC - 66);
        break;

        case YELLOW:
            analogWrite(red, NEGATED_LOGIC - 255);
            analogWrite(green, NEGATED_LOGIC - 255);
            analogWrite(blue, NEGATED_LOGIC - 0);

        break;
        case YELLOW_GREEN:
            analogWrite(red, NEGATED_LOGIC - 13);
            analogWrite(green, NEGATED_LOGIC - 152);
            analogWrite(blue, NEGATED_LOGIC - 186);
        break;
        case GREEN:
            analogWrite(red, NEGATED_LOGIC - 0);
            analogWrite(green, NEGATED_LOGIC - 255);
            analogWrite(blue, NEGATED_LOGIC - 0);
        break;

        case BLUE_GREEN:
            analogWrite(red, NEGATED_LOGIC - 0);
            analogWrite(green, NEGATED_LOGIC - 255);
            analogWrite(blue, NEGATED_LOGIC - 0);
        break;

        case BLUE:
            analogWrite(red, NEGATED_LOGIC - 0);
            analogWrite(green, NEGATED_LOGIC - 0);
            analogWrite(blue, NEGATED_LOGIC - 255);
        break;

        case BLUE_VIOLET:
            analogWrite(red, NEGATED_LOGIC - 138);
            analogWrite(green, NEGATED_LOGIC - 42);
            analogWrite(blue, NEGATED_LOGIC - 226);

        break;
        case VIOLET:
            analogWrite(red, NEGATED_LOGIC - 238);
            analogWrite(green, NEGATED_LOGIC - 130);
            analogWrite(blue, NEGATED_LOGIC - 28);
        break;
        case RED_VIOLET:
            analogWrite(red, NEGATED_LOGIC - 199);
            analogWrite(green, NEGATED_LOGIC - 21);
            analogWrite(blue, NEGATED_LOGIC - 133);
        case WHITE:
            analogWrite(red, NEGATED_LOGIC - 255);
            analogWrite(green, NEGATED_LOGIC - 255);
            analogWrite(blue, NEGATED_LOGIC - 255);
        break;
        
        default:
            analogWrite(red, NEGATED_LOGIC - 0);
            analogWrite(green, NEGATED_LOGIC - 0);
            analogWrite(blue, NEGATED_LOGIC - 0);
    }
    
}

void USART0_Init()
{
	// Set Baud Rate
	UBRR0H = BAUD_PRESCALER >> 8;
	UBRR0L = BAUD_PRESCALER;
	
	// Set Frame Format
	UCSR0C = ASYNCHRONOUS | PARITY_MODE | STOP_BIT | DATA_BIT;
	
	// Enable Receiver and Transmitter
	UCSR0B = (1<<RXEN0) | (1<<TXEN0);
}

void USART0_TransmitPolling(uint8_t DataByte)
{
	while (( UCSR0A & (1<<UDRE0)) == 0) {}; // Do nothing until UDR is ready
	UDR0 = DataByte;
}


int8_t USART0_is_recv_ready() {
    return (UCSR0A & (1 << RXC0));
}

char USART0_receive()
{
    /* asteapta cat timp bufferul e gol */

    while (!(UCSR0A & (1 << RXC0)));

    /* returneaza datele din buffer */
    return UDR0;
}

void USART0_receive_data(char *str)
{
    char recv = '0';
    int index = 0;
    while (recv != '\n')
    {
        recv = USART0_receive();
        str[index] = recv;
        index++;
    }

    str[index] = '\0';
}

void print_terminal(char *str)
{
  while (*str)
  {
    USART0_TransmitPolling(*str);
    str++;
  }
}

void set_timer_seconds(uint16_t value) {
	TCCR1A = 0; 
	TCCR1B = 0;
	TCNT1 = 0;
	TCCR1B |= (1 << WGM12); // CTC mode
    TIMSK1 |= (1 << OCIE1A); // Enable Timer1 compare match A interrupt
    sei(); // Enable global interrupts
	
	// set prescaler to 1024
	TCCR1B |= (1 << CS12) | (1 << CS10);
	OCR1A = value;
	// formula: (16 * 10^6) / (1024 * 15624) - 1 = 1 // (1Hz) => OCR1A = 15624
}

volatile uint32_t counter = 0;

ISR(TIMER1_COMPA_vect) {
    counter++;
}

uint32_t get_seconds() {
	return counter;
}
