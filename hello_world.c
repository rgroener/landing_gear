/*
LG Testprogramm
* grn
* Sept 17
* 
*/


#include <util/delay.h>
#include "u8g.h"
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <string.h>
#include <avr/eeprom.h>


#if defined(__AVR__)
#include <avr/interrupt.h>
#include <avr/io.h>
#endif

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL)))-1)
#define MAX_BUFFER 20
char c,d, received_byte;	

#define RELOAD_ENTPRELL 50
#define RELOAD_WAIT_DOWN 10
#define RELOAD_WAIT_UP 20
#define RELOAD_TRANS_DOWN 10
#define RELOAD_TRANS_UP 10
#define RELOAD_SAVE_TIMER 1

#define GEAR_DOWN	1
#define WAIT		2
#define RETRACT 	3	
#define GEAR_UP 	4
#define DEPLOY		5

#define STEP 50


#define MAIN	0
#define CHANGE	1
#define SET		2

u8g_t u8g;

uint8_t tast;
volatile uint8_t clock_takt, ms, ms10, ms100, sek, entprell;
volatile uint8_t save_timer;
uint8_t wait_down, wait_up, trans_up, trans_down;
volatile uint16_t lg_pos_down, lg_pos_up, servo_position;
uint8_t points, servo_speed;

uint8_t mode;
uint8_t eeprom_wait_down EEMEM;
uint8_t eeprom_wait_up EEMEM;
uint16_t eeprom_lg_pos_down EEMEM;
uint16_t eeprom_lg_pos_up EEMEM;
uint8_t eeprom_servo_speed EEMEM;


void uart_send_string(volatile char *s);
void uart_send_char(char c);  
void uart_string_buff_reset(void);

char *test_string[] = {"TEST"};
char *test_string2[]={"TESc"};
char string_buffer[10]= {"123"};

char buffer[11]={""};
uint32_t zaehler;
uint8_t led_r, led_g;
char *zbuffer, *z_buffstart;
uint8_t incomflag, incompos;
volatile uint8_t param;//menu states
volatile uint8_t show, set_param;
volatile uint8_t test;
#define UART_MAXSTRLEN 10//maximale stringlaenge


volatile uint8_t uart_str_complete = 0;     // 1 .. String komplett empfangen
volatile uint8_t uart_str_count = 0;
volatile char uart_string[UART_MAXSTRLEN + 1] = "";


/* 9600 baud / Geschwindikeit Uebertragung RS232 Schnittstelle*/
#define UART_BAUD_RATE      9600      
ISR(USART_RX_vect)
{
	received_byte = UDR0;
	UDR0 = received_byte;//Echo Byte
	show=1;
	uint8_t counter=0;
	uint16_t result=0;
	
	
	if(param!=0)
	{
		if(received_byte=='s')//write new value to variable
		{
			uart_str_complete=1;
			param=0;
		}
		
		if(received_byte=='p')
		{
			uart_str_complete=1;
			switch(param)
			{
				case 3:	lg_pos_down += STEP;
						break;
				case 4:	lg_pos_up += STEP;
						break;
			}
		}
		if(received_byte=='o')
		{
			uart_str_complete=1;
			switch(param)
			{
				case 3:	lg_pos_down -= STEP;
						break;
				case 4:	lg_pos_up -= STEP;
						break;
			}
		}
		/*Check if fine adjusting (',','.') is finished*/
		if(uart_str_complete==1)
		{
			if((received_byte=='\n') || (received_byte=='\r'))
			{
				param=0;
				uart_str_complete=0;
			}
			
		}
	
		if(uart_str_complete == 0) 	
		{
			if( received_byte != '\n' && received_byte != '\r' && uart_str_count < UART_MAXSTRLEN ) 
			{
				// wenn uart_string gerade in Verwendung, neues Zeichen verwerfen
				// Daten werden erst in uart_string geschrieben, wenn nicht String-Ende/max Zeichenlänge erreicht ist/string gerade verarbeitet wird
				uart_string[uart_str_count] = received_byte;
				uart_str_count++;
			}else 
			{
				uart_string[uart_str_count] = '\0';
				uart_str_count = 0;
														
				for(counter=0; counter<11;counter++)//string copy due to const char needed by atoi function
				{
					buffer[counter] = uart_string[counter];
				}
				result=atoi(buffer);
				switch(param)
				{
					case 1:	wait_down = result;
							break;
					case 2:	wait_up = result;
							break;
					case 3:	lg_pos_down = result;
							break;
					case 4:	lg_pos_up = result;
							break;
					case 5:	servo_speed = result;
							break;
				} 
				result=0;
				param=0;
				uart_str_complete = 0;
			}
		}
	}//end of param !=0
}//end of USART_rx 

void u8g_setup(void)
{  
 
 u8g_InitSPI(&u8g, &u8g_dev_ssd1306_128x64_hw_spi, U8G_PIN_NONE, U8G_PIN_NONE, PN(2, 1), PN(2, 0), U8G_PIN_NONE);   
//														PB5(sck)	PB3 (mosi) 		cs		a0
	
  
  /* flip screen, if required */
 // u8g_SetRot180(&u8g);

  /* assign default color value */
  if ( u8g_GetMode(&u8g) == U8G_MODE_R3G3B2 ) 
    u8g_SetColorIndex(&u8g, 255);     /* white */
  else if ( u8g_GetMode(&u8g) == U8G_MODE_GRAY2BIT )
    u8g_SetColorIndex(&u8g, 3);         /* max intensity */
  else if ( u8g_GetMode(&u8g) == U8G_MODE_BW )
    u8g_SetColorIndex(&u8g, 1);         /* pixel on */
}

void sys_init(void)
{
#if defined(__AVR__)
  /* select minimal prescaler (max system speed) */
  CLKPR = 0x80;
  CLKPR = 0x00;
#endif
}

void u8g_prepare(void) {
  u8g_SetFont(&u8g, u8g_font_helvR08);
  u8g_SetFontRefHeightExtendedText(&u8g);
  u8g_SetDefaultForegroundColor(&u8g);
  u8g_SetFontPosTop(&u8g);
}

uint8_t taster(uint8_t nr)
{
		switch(nr)
	{
			case 1:	if(!(PIND & (1<<PD4)))return 1;else return 0;break;
			case 2:	if(!(PIND & (1<<PD3)))return 1;else return 0;break;
			case 3:	if(!(PIND & (1<<PD2)))return 1;else return 0;break;
			default: return 0;
	}//end of switch* 
}//end of taster

void draw(void) 
{
	if(mode != GEAR_DOWN)
	{
		u8g_DrawStr(&u8g, 0,15, "wait_down");
		u8g_DrawStr(&u8g,77, 15, u8g_u8toa(RELOAD_WAIT_DOWN, 3));
			
		u8g_DrawStr(&u8g, 0,25, "wait_up");
		u8g_DrawStr(&u8g,77, 25, u8g_u8toa(RELOAD_WAIT_UP, 3));
		
		u8g_DrawStr(&u8g, 0,35, "lg_pos_dwn");
		u8g_DrawStr(&u8g,77, 35, u8g_u16toa(lg_pos_down, 5));
			
		u8g_DrawStr(&u8g, 0,45, "lg_pos_up");
		u8g_DrawStr(&u8g,77, 45, u8g_u16toa(lg_pos_up, 5));
	}
	switch(mode)
	{
		case GEAR_DOWN:	u8g_DrawStr(&u8g, 0,0, "Gear is down");
						u8g_DrawStr(&u8g, 0,15, "param");
						u8g_DrawStr(&u8g,77, 15, u8g_u8toa(param, 3));
			
					u8g_DrawStr(&u8g, 0,25, "test");
					u8g_DrawStr(&u8g,77, 25, u8g_u8toa(test, 3));	
						break;
		case WAIT:		u8g_DrawStr(&u8g, 0,0, "wait to retract");
						u8g_DrawStr(&u8g,77, 0, u8g_u8toa(wait_down, 3));
						
						break;
		case RETRACT:	u8g_DrawStr(&u8g, 0,0, "Gear retracting");
						switch(points)
						{
							case 1:	u8g_DrawStr(&u8g, 77,0, ">");break;
							case 2:	u8g_DrawStr(&u8g, 77,0, "=>");break;
							case 3:	u8g_DrawStr(&u8g, 77,0, "==>");break;
							case 4:	u8g_DrawStr(&u8g, 77,0, "===>");break;
							case 5:	u8g_DrawStr(&u8g, 77,0, "====>");break;
						}
						break;
		case GEAR_UP:	u8g_DrawStr(&u8g, 0,0, "gear is up");
						u8g_DrawStr(&u8g,77, 0, u8g_u8toa(wait_up, 3));
						break;
						
		case DEPLOY:	u8g_DrawStr(&u8g, 0,0, "Gear deploying");
						switch(points)
						{
							case 1:	u8g_DrawStr(&u8g, 77,0, ">");break;
							case 2:	u8g_DrawStr(&u8g, 77,0, "=>");break;
							case 3:	u8g_DrawStr(&u8g, 77,0, "==>");break;
							case 4:	u8g_DrawStr(&u8g, 77,0, "===>");break;
							case 5:	u8g_DrawStr(&u8g, 77,0, "====>");break;
						}					
						break;
	}//end of draw-switch
}//end of draw

void init_eeprom(void)
{
	/*Kontrolliert ob schon Speicherzelle in eeprom schon benutzt wurde oder noch FF ist.
	Falls Zelle noch nie benutzt wurde, wird der Defaultwert ins Eeprom geschrieben*/
	if(eeprom_read_byte(&eeprom_wait_down)==0xFF)
	{
		eeprom_write_byte(&eeprom_wait_down, 10);	
	}
	if(eeprom_read_byte(&eeprom_wait_up)==0xFF)
	{
		eeprom_write_byte(&eeprom_wait_up, 20);	
	}
	if(eeprom_read_word(&eeprom_lg_pos_down)==0xFF)
	{
		eeprom_write_word(&eeprom_lg_pos_down, 3100);	
	}
	if(eeprom_read_word(&eeprom_lg_pos_up)==0xFF)
	{
		eeprom_write_word(&eeprom_lg_pos_up, 5450);	
	}
	if(eeprom_read_byte(&eeprom_servo_speed)==0xFF)
	{
		eeprom_write_byte(&eeprom_servo_speed, 1);	
	}
}

ISR (TIMER0_COMPA_vect)
{
	clock_takt++;
	if(clock_takt == 83)//83 => 1ms
	{
		clock_takt=0;
		if(entprell != 0)entprell--;
		ms++;		
	}
	if(ms==10)//10ms
	{
		ms=0;
		ms10++;
		if(entprell != 0)entprell--;
		if(mode==RETRACT)
		{
			if((servo_position+servo_speed) <= lg_pos_up)//check if present positon plus next step ist still lower than lg_pos_up
			{
				servo_position += servo_speed;
			}else servo_position=lg_pos_up;//set servo positon to lg_pos_up
		}
		if(mode==DEPLOY)
		{
			if((servo_position-servo_speed) >= lg_pos_down)//check if present positon minus next step is still higher than lg_pos_down
			{
				servo_position -= servo_speed;
			}else servo_position=lg_pos_down;//set servo position to lg_pos_down
		}
	}
	if(ms10 == 10)//100ms
	{
		ms10=0;
		ms100++;
		
	}
	if(ms100 == 10) //seconds
	{
		sek++;
		ms100=0;
		if((mode==WAIT) && (wait_down != 0))wait_down--;	//decrease timer for gear down
		if((mode==RETRACT)&&(trans_up!=0))trans_up--;
		if((mode==GEAR_UP)&&(wait_up!=0))wait_up--;
		if((mode==DEPLOY)&&(trans_down!=0))trans_down--;
		if((mode==GEAR_DOWN)&&(save_timer!=0))save_timer--;
			
		if((mode==RETRACT) || (mode==DEPLOY))
		{
			points++;
			if(points==6)points=0;
		}
	}
	if(sek==60)sek=0;
}//endo of ISR
int main(void)
{
	DDRD &= ~((1<<PD2) | (1<<PD3) |(1<<PD4));	//Taster
	PORTD |= ((1<<PD2) | (1<<PD3) |(1<<PD4));	//Pullup für Taster aktivieren
	DDRD |= (1<<PD0);	//Front LED
	PORTD |= (1<<PD0);
	DDRB |= ((1<<PB1) |(1<<PB2));	// deklaraton Servoausgänge
	PORTB &= ~((1<<PB1) |(1<<PB2));	// init Servoausgänge
	
		
		// Timer 0 konfigurieren
	TCCR0A = (1<<WGM01); // CTC Modus
	TCCR0B |= (1<<CS00); // Prescaler 1
	TIMSK0 |= (1<<OCIE0A);
	OCR0A = 241;	//241 => 50us
	ICR1=0xC350; //TOP Wert bei 20Mhz

	//16 Bit PWM Initialisieren
	
	TCCR1A	|= ((1<<COM1A1) | (1<<COM1B1));		// PWM Kanaele freischalten; (Pins als Ausgang programmieren...)
	TCCR1A |= (1<<WGM11);						//
	TCCR1B	|=((1<<WGM12) | (1<<WGM13));		// Fast PWM
	TCCR1B |= (1<<CS11);						// clock/8			
	TCNT0=0;
	
	
	UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0);	//Turn on RX and TX circuits RXCIE0 enables Interrupt when byte received
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);	//8-Bit Char size
	UBRR0H = (BAUD_PRESCALE >> 8);	//load upper 8-Bits of baud rate value into high byte of UBRR0H
	UBRR0L = BAUD_PRESCALE;			//load lower 8-Bits of Baud rate into low byte of UBRR0L

	sei();		//globale Interrups freigeben
	
	sys_init();
	u8g_setup();
	u8g_prepare();

  
	//startwert_links=3689;	//Laptop 6620
	//startwert_rechts=3546;	//Laptop 3560
	
	mode = GEAR_DOWN;
	
	
	//speed_rechts=startwert_rechts;
	//speed_links=startwert_links;	//-11900
  
	ms=0;
	ms10 = 0;
	ms100=0;
	sek=0;
	entprell=0;
	
	/*	Read default values from eeprom and store it in the appropriate varable
	 * */
	wait_down = eeprom_read_byte(&eeprom_wait_down);
	wait_up = eeprom_read_byte(&eeprom_wait_up);
	lg_pos_down = eeprom_read_word(&eeprom_lg_pos_down);			
	lg_pos_up = eeprom_read_word(&eeprom_lg_pos_up);
	servo_speed = eeprom_read_byte(&eeprom_servo_speed);
				
	
	//wait_down	=	10;				//starts counting to zero when start button is pressed. If zero, gear starts to retract
	//wait_up		=	12;				//starts counting to zero when gear is fully retract. If zero, gear starts to deploy
	
	//trans_up	= 5;				//transition time while retracting
	//trans_down	= 5;				//transition time while deploying
	
	points = 0;						//visualisation on display during transition phase
	save_timer = RELOAD_SAVE_TIMER;
	incomflag=0;					//trigger char for new string detected
	incompos=0;						//position in array for new char
	show=0;
	param=0;
	set_param=0;
	servo_position=lg_pos_down;
	init_eeprom();
	test=0;

  for(;;)
  {  
	//uart_send_char('S');
	
	switch(mode)
	{
		case GEAR_DOWN:	// gear down, waiting for the start button to be pressed
						if(taster(2) && (entprell==0))	//Service Mode
						{
							entprell = RELOAD_ENTPRELL;
							mode = WAIT;
								
						}	
						if(param==0)
						{
							switch(received_byte)
							{
								case '1':	if(show)
											{
												uart_send_string("\n\r\n\rwait_down = ");
												param=1;
												show=0;//only print it once
											}break;
								case '2':	if(show)
											{
												uart_send_string("\n\r\n\rwait_up = ");
												
												param=2;
												show=0;//only print it once
											}break;
								case '3':	if(show)
											{
												uart_send_string("\n\r\n\renter new value or fine adjust with 'o' and 'p'");
												uart_send_string("\n\rpress 'o' to increase position");
												uart_send_string("\n\rpress 'p' to decrease position");
												uart_send_string("\n\r\n\rlg_pos_down = ");
												
												param=3;
												show=0;//only print it once
											}break;
								case '4':	if(show)
											{
												uart_send_string("\n\r\n\enter new value or fine adjust with 'o' and 'p'");
												uart_send_string("\n\rpress 'o' to increase position");
												uart_send_string("\n\rpress 'p' to decrease position");
												uart_send_string("\n\r\n\rlg_pos_up = ");
												param=4;
												show=0;//only print it once
											}break;	
								case '5':	if(show)
											{
												uart_send_string("\n\r\n\rservo_speed = ");
												param=5;
												show=0;//only print it once
											}break;					
								case 's':	if(show)
											{/*values will only be saved if they differ from the one stored in the eeprom*/
												uart_send_string("\n\r\n\rcurrent parameter saved to eeprom...");
												if(wait_down != eeprom_read_byte(&eeprom_wait_down))eeprom_write_byte(&eeprom_wait_down, wait_down);
												if(wait_up != eeprom_read_byte(&eeprom_wait_up))eeprom_write_byte(&eeprom_wait_up, wait_up);
												if(lg_pos_down != eeprom_read_word(&eeprom_lg_pos_down))eeprom_write_word(&eeprom_lg_pos_down, lg_pos_down);
												if(lg_pos_up != eeprom_read_word(&eeprom_lg_pos_up))eeprom_write_word(&eeprom_lg_pos_up, lg_pos_up);
												if(servo_speed != eeprom_read_byte(&eeprom_servo_speed))eeprom_write_byte(&eeprom_servo_speed, servo_speed);
												save_timer = RELOAD_SAVE_TIMER;
												show=0;//only print it once
											}
											if(save_timer==0)
											{
												param=0;
												show=1;
												received_byte='0'; //to make sure that default case will be next
												uart_send_string("\n\r\n\r\n\r\n\r\n\r\n\r");
											}break;		
								case 'h':	if(show)
											{
												uart_send_string("\n\r\n\rwait_down = delay before gear starts to retract");
												uart_send_string("\n\r\n\rwait_up = delay before gear starts to deploy");
												uart_send_string("\n\r\n\rlg_gear_down = position landing gear when fully deployed");
												uart_send_string("\n\r\n\rlg_gear_up = position landing gear when fully retracted");
												uart_send_string("\n\r\n\rservo_speed = speed of landing gear when moving");
												show=0;
											}break;				
								default:	if(show)//print new screen on serial
											{
												uart_send_string("\n\r\n\n\r\n\n\r\n\n\r\n\r");
												uart_send_string("**********************");
												uart_send_string("\n\r\n\rcurrent values \n\r\n\r");
												uart_send_string("**********************\n\r\n\r");
												uart_send_string("1: wait_down     =  ");
												uart_send_string(itoa(wait_down, buffer,10));
												uart_send_string("\n\r");
																				
												uart_send_string("2: wait_up       =  ");
												uart_send_string(itoa(wait_up, buffer,10));
												uart_send_string("\n\r");
																				
												uart_send_string("3: lg_pos_down   =  ");
												uart_send_string(itoa(lg_pos_down, buffer,10));
												uart_send_string("\n\r");
																			
												uart_send_string("4: lg_pos_up     =  ");
												uart_send_string(itoa(lg_pos_up, buffer,10));
												uart_send_string("\n\r");
												
												uart_send_string("5: servo_speed   =  ");
												uart_send_string(itoa(servo_speed, buffer,10));
												uart_send_string("\n\r");
															
												uart_send_string("\n\r\n\rtype 'nr' for parameter to change");
												uart_send_string("\n\rtype 's' to save parameter in eeprom");
												uart_send_string("\n\rtype 'h' for help\n\r");
												show=0;//only print it once
											}//end of if show
											break;
								}//end of switch received byte
						}//eof param=0
						if(param==3)servo_position=lg_pos_down;
						if(param==4)servo_position=lg_pos_up;
						if(param==0)servo_position=lg_pos_down;
						break;
		case WAIT:		//Startbutton has been pressed, waiting time started to run down
						if(wait_down==0)
						{
							mode = RETRACT;
							trans_up=RELOAD_TRANS_UP;
							points=0;
						}
						break;
		case RETRACT:	//Landingear starts to retract
						if(servo_position==lg_pos_up)
						{
							mode=GEAR_UP;
							
						}
						break;
		case GEAR_UP:	//Gear is up for selected time
						if(wait_up==0)
						{
							mode=DEPLOY;
							trans_down=RELOAD_TRANS_DOWN;
							points=0;
						}
						break;
		case DEPLOY:	if(servo_position==lg_pos_down)
						{
							mode = GEAR_DOWN;
							wait_up=eeprom_read_byte(&eeprom_wait_up);
							wait_down = eeprom_read_byte(&eeprom_wait_down);
						}					
						
						break;
		
	}//end of switch
	
	//OCR1A=lg_pos_down;	//=-Wert 0x05A6		
	OCR1B=servo_position;	//OCR1B=0x2D88;  
	
	
    u8g_FirstPage(&u8g);
    do
    {
      draw();
    } while ( u8g_NextPage(&u8g) );
    // u8g_Delay(150);
  }//End of for
}//End of main



void uart_send_char(char c)
{
	while((UCSR0A & (1<<UDRE0)) == 0){};
    UDR0 = c;
}
void uart_send_string(volatile char *s)
{
	while(*s != 0x00)
	{
		uart_send_char(*s);
		s++;
	}
}//end of send_string
