#define F_CPU 16000000UL 
#define UBRR_VAL 103
#include <avr/io.h>
#include <util/delay.h>
// #include "lcd.h"
#include <LiquidCrystal.h>
#include <Arduino.h>
#include "Macros.h"
LiquidCrystal lcd(A1,A2,A3,A4,A5,A6);

#define TRIGGER_PORT PORTA
#define TRIGGER_DDR DDRA
#define TRIGGER_PIN 0

#define ECHO_PORT PIND
#define ECHO_DDR DDRD
#define ECHO_PIN 2

void init_s() {
    // Set Trigger as output
    TRIGGER_DDR |= (1 << TRIGGER_PIN);
    // Set Echo as input
    ECHO_DDR &= ~(1 << ECHO_PIN);

    // Initialize Timer1
    TCCR1A = 0;  // Set Timer1 under normal mode
    TCCR1B = 0;
    TCCR1B |= (1 << CS11);  // Prescaler set to 8 (timer clock = system clock / 8)
}

void sendTriggerPulse() {
    TRIGGER_PORT |= (1 << TRIGGER_PIN);  // Trigger pin high
    _delay_us(10);  // Wait 10 microseconds
    TRIGGER_PORT &= ~(1 << TRIGGER_PIN);  // Trigger pin low
}

uint16_t measureEchoTime() {
    uint16_t count = 0;

    // Wait for echo pin to go high
    while (!(ECHO_PORT & (1 << ECHO_PIN)));

    // Start timer
    TCNT1 = 0;  // Clear timer count

    // Wait for echo pin to go low
    while (ECHO_PORT & (1 << ECHO_PIN)) {
        count = TCNT1;
        if (count > 65000) break;  // prevent infinite loop
    }

    return count;
}

float calculateDistance(uint16_t timerCount) {
    float distance;
    // Calculate distance in cm (timerCount times prescaler times sound speed factor / 2)
   distance = (timerCount * 0.25 * 0.034029);
   // distance=(timerCount*256+TCNT0)/900;
    return distance;
}
void uart_init(uint16_t ubrr);
void uart_tx(uint8_t d);
uint8_t uart_rx();
uint8_t c ,nom;
int speed;
int main(void) {
    init_s();
    uint16_t echoTime;
    float distance;
    SET_BIT(SREG,7);
  

  CLR_BIT(MCUCR,ISC10);
  SET_BIT(MCUCR,ISC11);

  
  SET_BIT(GICR,INT1);

    // c=0;
    /*
     Set fast PWM mode with non
     inverting output
     */
     TCCR0|=(1<<COM01)|(1<<WGM00)|(1<<WGM01);
     //select 1:1 prescaler
     TCCR0|=(1<<CS00);


    DDRC=0x01;
    PORTC=0xFD;
    DDRB|=(1<<3);
    
   uart_init(UBRR_VAL);
  sei();



    while (1) {
        sendTriggerPulse();
        echoTime = measureEchoTime();
        distance = calculateDistance(echoTime);

                		if (distance < 19.73){
                			nom++;
                      PORTC|=0x01;
        	           	_delay_ms(50);
                		}
                		else if (distance > 19.73){
                      CLR_BIT(PORTC,0);
        	          	_delay_ms(50);
                			nom=0;
                		}

                		if ( nom == 1)  {    c++;   }


        lcd.print(c);
        _delay_ms(200);  // Delay between readings
        lcd.clear();

        // if ((PINC&(1<<2))==0){
        // 	_delay_ms(50);
        	// if ((PINC&(1<<2))==0){
        	// 	// PORTC ^=0x01;
            //      lcd.clear();
            //      _delay_ms(200); 
            //      lcd.print("total price =");
            //      lcd.print(c*50);
            //       lcd.print("$");

        	// 	_delay_ms(300);
        	// }
        // }



        //if ((PINC&(1<<3))==0){
        	// _delay_ms(100);
        	if ((PINC&(1<<3))==0){
                c=0;
        		// lcd.print(c);
        	}
      //  }

         
        


        OCR0=speed; // 100 255

    }
}
ISR(INT1_vect){

  cli();
  lcd.clear();
                 _delay_ms(200); 
                 lcd.print("total price =");
                 lcd.print(c*50);
                  lcd.print("$");
                  _delay_ms(600);
 sei();
}
ISR(USART_RXC_vect)
{
    uint8_t rx = 0;
    rx = uart_rx();
    switch (rx)
        {
        case '0':
       // CLR_BIT(PORTC,1);
        speed=0;
       
            break;
        case '1':
        //SET_BIT(PORTC,1);
        speed=255;
        
            break;
            case '2':
       // CLR_BIT(PORTC,1);
        speed=150;
       
            break;
        default:
            break;
        }
}
void uart_init(uint16_t ubrr)
{
    UBRRH = (uint8_t)(ubrr >> 8);
    UBRRL = (uint8_t)ubrr;
    UCSRB = (1 << RXCIE) | (1 << RXEN) | (1 << TXEN);
    UCSRC = (1 << UCSZ1) | (1 << UCSZ0);
}

void uart_tx(uint8_t d)
{
    while (!(UCSRA & (1 << UDRE)));
    UDR = d;
}

uint8_t uart_rx()
{
    while (!(UCSRA & (1 << RXC)));
    return UDR;
}