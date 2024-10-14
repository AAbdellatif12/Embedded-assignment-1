// main.ino 

#include <avr/io.h>       // AVR input/output definitions
#include <util/delay.h>   // Including delay function
#include "uart.h"         // Including the UART header file 


void UART_init(void) {
    UBRR0H = (BRC >> 8);   // Setting the upper byte of the baud rate
    UBRR0L = BRC;          // Setting the lower byte of the baud rate
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);  // Enabling transmitter and receiver
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); 
}

void UART_transmit(unsigned char data) {
    while (!(UCSR0A & (1 << UDRE0))); 
    UDR0 = data; 
}

unsigned char UART_receive(void) {
    while (!(UCSR0A & (1 << RXC0)));  
    return UDR0;  
}

void UART_sendString(const char* str) {
    while (*str) {  
        UART_transmit(*str++);  
    }
}

// Pin definitions
#define BUTTON1_PIN PD2  // Button1 as PD2 (Pin D2 on the microcontroller)
#define BUTTON2_PIN PD3  // Button2 as PD3 (Pin D3 on the microcontroller)
#define LED_PIN PB5      // LED_PIN as PB5 (Pin B5 on the microcontroller)

// Function declarations
void init_io(void);              // Declaring a function to initialize I/O pins
void handle_buttons(void);       // Declaring a function to handle button inputs
void process_uart_commands(void);// Declaring a function to process UART commands

// Setup function
void setup() {
    UART_init(); // Initializing UART communication
    init_io();   // Initializing buttons and LED pins as input/output
}

void loop() {
    handle_buttons();       // Continuously check and handle button presses
    process_uart_commands();// Continuously check and process UART commands
    _delay_ms(300);          // delay 
}

// Initialize I/O for buttons and LED
void init_io(void) {
    DDRD &= ~((1 << BUTTON1_PIN) | (1 << BUTTON2_PIN)); // Setting buttons (D2, D3) as inputs
    PORTD |= (1 << BUTTON1_PIN) | (1 << BUTTON2_PIN);   // Enabling pull-up resistors for buttons
    DDRB |= (1 << LED_PIN);  // Setting LED (B5) as output
}

// Handle button presses and send UART messages
void handle_buttons(void) {
    static uint8_t button1_last_state = 1; // Variable to store the last state of button 1
    static uint8_t button2_last_state = 1; // Variable to store the last state of button 2

    uint8_t button1_state = PIND & (1 << BUTTON1_PIN); // Reading current state of button 1
    if (button1_state != button1_last_state) {  // Checking if button 1 state has changed
        if (button1_state == 0) {  // If button 1 is pressed 
            UART_sendString("button1_pressed\n"); // Send message via UART
        } else {  // If button 1 is released
            UART_sendString("button1_released\n"); // Send message via UART
        }
        button1_last_state = button1_state;  // Update last state of button 1
    }

    uint8_t button2_state = PIND & (1 << BUTTON2_PIN); // Reading current state of button 2
    if (button2_state != button2_last_state) {  // Check if button 2 state has changed
        if (button2_state == 0) {  // If button 2 is pressed
            UART_sendString("button2_pressed\n"); // Send message via UART
        } else {  // If button 2 is released
            UART_sendString("button2_released\n"); // Send message via UART
        }
        button2_last_state = button2_state;  // Update last state of button 2
    }
}

// Process UART commands to control the LED
void process_uart_commands(void) {
    if (UCSR0A & (1 << RXC0)) { 
        char command = UART_receive(); // Receive the incoming command

        if (command == '1') {  // If the command is '1'
            PORTB |= (1 << LED_PIN);   // Turn LED on
            UART_sendString("led_on\n"); // Send feedback message via UART
        } else if (command == '0') { // If the command is '0'
            PORTB &= ~(1 << LED_PIN);  // Turn LED off
            UART_sendString("led_off\n"); // Send feedback message via UART
        } else {
           UART_sendString("invalid command\n"); // Send feedback message via UART
        }
    }
}
