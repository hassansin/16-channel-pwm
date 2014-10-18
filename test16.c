/*
 * 	
 * main.c
 *
 * Created: 6/27/2012 4:09:22 PM
 *  Author: Hassan
 *   Email: mhrk63@gmail.com
 */ 

//unused: OCDR, OSCCAL ? , ICR1H,ICR1L,TCNT0,TCNT2,OCR2,,SPDR,,,TWBR,

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define CHANNEL 16 //no of channels
#define BASE_FREQ 50
//#define PWM_COUNT ADCL
volatile uint8_t PWM_COUNT;
//#define update_flag ADCH
volatile uint8_t update_flag;
#define PWM_PORT1 PORTA
#define PWM_PORT1_DDR DDRA
#define PWM_PORT2 PORTC
#define PWM_PORT2_DDR DDRC

#define USART_BAUDRATE 57600 // accetep values: 1M , 0.5M , 250k, 76.8k, 38.4k
#define BAUD_PRESCALE ((F_CPU / (USART_BAUDRATE * 16UL)) - 1)
#define BUFFER_SIZE CHANNEL*4
#define LOWBYTE ICR1L
#define HIGHBYTE ICR1H
#define WORD ICR1

volatile uint16_t PWM[CHANNEL]={2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000}; // main array that holds the PWM value of each channel 

//Double Buffering System								
volatile uint16_t BUFFER_1[2][CHANNEL];
volatile uint16_t BUFFER_2[2][CHANNEL];							


volatile uint16_t *P=&BUFFER_1[0][0]; // pointer to the buffer that is currently used to generate PWM
volatile uint8_t P_l;//buffer length
volatile uint16_t *P_BUF=&BUFFER_2[0][0];// pointer to the buffer that is free and is used to update new PWM
volatile uint8_t P_BUF_l;//buffer length
							
volatile uint8_t UART_BUFFER[BUFFER_SIZE];
volatile uint8_t head_index;
volatile uint8_t tail_index;
volatile uint8_t count;
volatile uint8_t COMMAND;


ISR(TIMER1_COMPB_vect){	
	PWM_PORT1 &=~ P[CHANNEL+PWM_COUNT];
	PWM_PORT2 &=~ P[CHANNEL+PWM_COUNT]>>8;
	PWM_COUNT++;
	if(PWM_COUNT==P_l)
		PWM_COUNT=0;
	OCR1B=P[PWM_COUNT];

}
ISR(TIMER1_COMPA_vect){
	PWM_PORT1=0xFF;
	PWM_PORT2=0xFF;
	if(update_flag) //update new PWM value if exists 
		{
		P=P_BUF;
		P_l=P_BUF_l;
		update_flag=0;
		}
}
void update_buffer(void){
	//select buffer array
	if(P==&BUFFER_1[0][0])
			P_BUF=&BUFFER_2[0][0];
	else
		P_BUF=&BUFFER_1[0][0];
	uint8_t flag=0,l=0;
	for(uint8_t i=0;i<CHANNEL;i++){
		flag=0;		
		for(signed char j=i-1;j>=0;j--){
			if(PWM[i]==PWM[j])
				{
				flag=1;
				break;
				}			
		}
		
		if(flag) 
			continue;
		
		P_BUF[l]=PWM[i];
		P_BUF[CHANNEL+l]=1<<i;
		
		for(uint8_t k=i+1;k<CHANNEL;k++){
			if(PWM[i]==PWM[k])
				{
				P_BUF[CHANNEL+l] |=1<<k;
				}					
		}
		l++;	
	}		
	
	for(uint8_t i=0;i<l;i++)
		for(uint8_t j=i+1;j<l;j++){
			if(P_BUF[j]<P_BUF[i])
				{
					uint16_t temp=P_BUF[j];
					uint16_t temp2=P_BUF[CHANNEL+j];
					
					P_BUF[j]=P_BUF[i];
					P_BUF[CHANNEL+j]=P_BUF[CHANNEL+i];
					
					P_BUF[i]=temp;
					P_BUF[CHANNEL+i]=temp2;
				}
		}
	update_flag=1;
	P_BUF_l=l;
	
}
void init_PWM(){
	PWM_PORT1_DDR=0xFF;
	PWM_PORT1=0xFF;
	PWM_PORT2_DDR=0xFF;
	PWM_PORT2=0xFF;	
	TCNT1=0;
	
	OCR1A= F_CPU/(BASE_FREQ*8)-1;
	OCR1B=P[0];	

	TCCR1B |= 1<<WGM12; // Mode 4	

	
	TIMSK |=1<<OCIE1A | 1<<OCIE1B;
	TCCR1B |= 1<< CS11; //fclk=2Mhz
	
	update_buffer();
	
	
}
void USART_Transmit(unsigned char Txdata){
	while ((UCSRA & (1 << UDRE)) == 0) 
		; 
    UDR = Txdata;
	}
void control_dcmotor(uint8_t ch){
	if(ch==0) //fast stop
		{
		PORTB &=~0x0F;
		//PWM[13]=39949;
		//PWM[12]=39949;
		//update_buffer();
		}
	else if(ch==1)//both forward
		{
		PORTB |=0b00001010;
		PORTB &=~0b00000101;				
		}
	else if(ch==2)//both reverse
		{
		PORTB &=~0b00001010;
		PORTB |= 0b00000101;
		}
	else if(ch==3)//left
		{
		PORTB |=0b00001000;
		PORTB &=~0b00000111;
		}
	else if(ch==4)//right
		{
		PORTB |=0b00000010;
		PORTB &=~0b00001101;
		}
	USART_Transmit(1);
}
void init_usart(void){
	//Data-8bit, stop-1, parity-none
	
   	UBRRH = (BAUD_PRESCALE >> 8);
   	UBRRL = BAUD_PRESCALE; 
	UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1); 
	UCSRB |= (1 << RXEN) | (1 << TXEN) | (1<<RXCIE) ;      	
}	
void send_pwm(void){
	for(uint8_t i=0;i<CHANNEL;i++){
		USART_Transmit(PWM[i]>>8);
		USART_Transmit(PWM[i]);
		
	}
}
ISR(USART_RXC_vect){
	uint8_t temp=UDR;
	//USART_Transmit(temp);	
	UART_BUFFER[tail_index++]=temp;
	if(tail_index==BUFFER_SIZE)
		tail_index=0;
	if(tail_index>=head_index)
		count=tail_index-head_index;
	else
		count=tail_index+BUFFER_SIZE-head_index;	
		
}
void update_all_channel(void){
	for(uint8_t i=0;i<CHANNEL;i++){
		HIGHBYTE=UART_BUFFER[head_index++];
		if(head_index==BUFFER_SIZE)
			head_index=0;
		
		LOWBYTE=UART_BUFFER[head_index++];
		if(head_index==BUFFER_SIZE)
			head_index=0;
		PWM[i]=WORD;								
		}		
		if(tail_index>=head_index)
			count=tail_index-head_index;
		else
			count=tail_index+BUFFER_SIZE-head_index;	

		COMMAND=0;
			
		if(UART_BUFFER[head_index++] != ';')
		{			
			if(head_index==BUFFER_SIZE)
				head_index=0;
			if(tail_index>=head_index)
				count=tail_index-head_index;
			else
				count=tail_index+BUFFER_SIZE-head_index;	
			return;
		}
	if(head_index==BUFFER_SIZE)
				head_index=0;
			if(tail_index>=head_index)
				count=tail_index-head_index;
			else
				count=tail_index+BUFFER_SIZE-head_index;
	update_buffer();	
}
//read buffer for any  two byte command
void read_buffer(void){	
	while(count){
		if(COMMAND==1 && count>=(2*CHANNEL+1))
			update_all_channel();
		else if(COMMAND==0){		
			uint8_t command;
			command=UART_BUFFER[head_index++];
			if(head_index==BUFFER_SIZE)
					head_index=0;
			if(tail_index>=head_index)
				count=tail_index-head_index;
			else
				count=tail_index+BUFFER_SIZE-head_index;	
					
				switch(command){
				case 'M':COMMAND=1;break;
				case 'f':control_dcmotor(1);break;
				case 'b':control_dcmotor(2);break;
				case 'l':control_dcmotor(3);break;
				case 'r':control_dcmotor(4);break;
				case 'h':control_dcmotor(0);break;
				case '?':send_pwm();break;
				}	
			}			
	}	
}

int main(void)
{	
	init_PWM();
	init_usart();
	DDRB |=0x0F;
	sei();
	
    while(1)
    {
		read_buffer();
    }
}