/*
 * press_control_t13.c
 *
 * Created: 01.07.2019 9:04:47
 * Author: Lepik
 * Copyright (c) 2019 Alexey K
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU 9600000L
#include <util/delay.h>

#define ADC_VREF_TYPE (0<<REFS0)

//Pin ADC
#define PRES_PIN PINB2
#define MAX_PIN PINB3
#define MIN_PIN PINB4

//Out pins
#define MOT_PIN PINB0

#define MOT_PIN_HI PORTB|=(1<<MOT_PIN)
#define MOT_PIN_LO PORTB&=~(1<<MOT_PIN)
#define MOT_PIN_DOWN (!(PINB & _BV(MOT_PIN)))
#define MOT_PIN_UP (PINB & _BV(MOT_PIN))

volatile uint16_t min_pres;
volatile uint16_t min_pres_sum;
volatile uint16_t max_pres;
volatile uint16_t max_pres_sum;
volatile uint16_t curr_pres;
volatile uint16_t curr_pres_sum;

volatile uint8_t tim_ms_250;

void init_ports(void)
{
	DDRB = (1<<DDB0)|(0<<DDB2)|(0<<DDB3)|(0<<DDB4);
	PORTB = (0<<MOT_PIN)|(0<<PRES_PIN)|(0<<MIN_PIN)|(0<<MAX_PIN);		
}

void init_adc(void)
{
	ADMUX|=(0<<MUX1)|(0<<MUX0)|(0<<REFS0);
	ADCSRA|=(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0); // ADC Prescaler Selections=64
	ADCSRA|=(1<<ADEN);// ADC is ON                                          
	DIDR0|=(1<<ADC1D)|(1<<ADC2D)|(1<<ADC3D);
}

ISR(TIM0_COMPA_vect)
{
	if (tim_ms_250 > 0){
		tim_ms_250--;
	}	
}

void Timer0_Init(void)
{
	TCCR0A |= (1<<WGM01); // set timer counter mode to CTC	
	TCCR0B|=(1<<CS01)|(1<<CS00); // set prescaler to 64
	OCR0A=0x95;
	TIMSK0|=(1<<OCIE0A); // enable Timer CTC interrupt
}

uint16_t read_adc(uint8_t ch)
{
	uint16_t ret;
	ret = 0;
	ADMUX = (0 << ADC_VREF_TYPE) + ch;
	// Delay needed for the stabilization of the ADC input voltage
	_delay_us(10);
	// Start the AD conversion
	ADCSRA|=(1<<ADSC);
	// Wait for the AD conversion to complete
	while(ADCSRA & (1<<ADSC)){}
	ADCSRA|=(1<<ADIF);
	ret = ADCL;
	ret += (ADCH<<8);	
	return ret;
}

void read_min_pres(void)
{	
	uint8_t adc_cou;	
	for(adc_cou = 0; adc_cou < 5; adc_cou++) {
		min_pres_sum += read_adc(2);
	}	
	min_pres = min_pres_sum / 5;
	min_pres_sum = 0;
}

void read_max_pres(void)
{
	uint8_t adc_cou;
	for(adc_cou = 0; adc_cou < 5; adc_cou++) {
		max_pres_sum += read_adc(3);
	}
	max_pres = max_pres_sum / 5;
	max_pres_sum = 0;	
}

void read_cur_pres(void)
{
	uint8_t adc_cou;
	for(adc_cou = 0; adc_cou < 5; adc_cou++) {
		curr_pres_sum += read_adc(1);
	}	
	curr_pres = curr_pres_sum / 5;
	curr_pres_sum = 0;
}

void init_var(void)
{	
	min_pres = 0;	
	max_pres = 0;
	curr_pres  = 0;
	min_pres_sum = 0;
	max_pres_sum = 0;
	curr_pres_sum = 0;
	tim_ms_250 = 250;
}

void read_pressure(void)
{
	if (tim_ms_250 == 0){
		read_cur_pres();
		read_min_pres();
		read_max_pres();		
		tim_ms_250 = 250;
	}		
}

void pump_on(void)
{
	uint8_t p_on;
	p_on = 1;
	while(p_on)
	{
		read_pressure();
		if (curr_pres > max_pres){
			MOT_PIN_HI;
		}else{
			p_on = 0;
			MOT_PIN_LO;
		}
	}
}

int main(void)
{	
	uint8_t first_loop;
	first_loop = 1;
	init_var();
	init_ports();
	init_adc();
	Timer0_Init();	
	sei();
    while(1)
    {
        read_pressure();
		while(first_loop)
		{
			read_pressure();			
			if (curr_pres > max_pres){
				MOT_PIN_HI;
			}else{
				first_loop = 0;
				MOT_PIN_LO;
			}
		}

		if ((MOT_PIN_DOWN) && (curr_pres > min_pres)){
			pump_on();
		}
    }
}
