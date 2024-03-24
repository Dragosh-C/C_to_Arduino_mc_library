// header fo ctoardu_lib
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
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

void pinMode(int8_t pin, enum Mode value);
void digitalWrite(int8_t pin, enum Value value);
int digitalRead(int8_t pin); // !!! verify returning type

void set_rgb(enum Color color);
void turn_on_rgb(enum Color color);
void turn_off_rgb(enum Color color);

void USART0_Init();
void USART0_TransmitPolling(uint8_t DataByte);
int8_t USART0_ReceivePolling();
char USART0_Receive();
void USART0_receive_data(char *cmd);
int8_t USART0_is_recv_ready();
void print_terminal(char *str);


void analogWrite(int8_t pin, uint8_t value);
void pwm_init(int8_t pin);



// volatile int8_t countButtonPress = 0;
// volatile uint8_t buttonState = 0;
// volatile uint8_t lastButtonState = 0;
// volatile uint8_t buttonStable = 0;
// volatile uint32_t debounceTime = 0;
// volatile uint64_t miliseconds = 0;

// const uint32_t debounceDelay = 50; // debounce delay (in milliseconds)



// Pins atmega328p
enum Pins {
  P0 = PD0,
  P1 = PD1,
  P2 = PD2,
  P3 = PD3,
  P4 = PD4,
  P5 = PD5,
  P6 = PD6,
  P7 = PD7,
  P8 = PB0,
  P9 = PB1,
  P10 = PB2,
  P11 = PB3,
  P12 = PB4,
  P13 = PB5,
  P14 = PC0,
  P15 = PC1,
  P16 = PC2,
  P17 = PC3,
  P18 = PC4,
  P19 = PC5,
};


enum Pins currentPin;
void attach_interrupt_PCIE(uint8_t pin);
void configure_timer();
uint64_t millis();




// uint32_t get_seconds();
