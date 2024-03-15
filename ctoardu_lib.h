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

void set_timer_seconds(uint16_t value);

void analogWrite(int8_t pin, uint8_t value);
void pwm_init(int8_t pin);




uint32_t get_seconds();