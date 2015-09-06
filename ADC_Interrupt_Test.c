/*
 * ADC_Interrupt_Test.c
 *
 * Created: 2015/09/06 9:11:07
 *  Author: gizmo
 *
 * PORTB[PB1]     : Rotary Encoder SW
 * PORTB[PB2]     : Rotary Encoder A
 * PORTB[PB3]     : Rotary Encoder B
 *
 * PORTB[PB4]     : SW
 *
 * PORTB[PB5]     : LED
 *
 * PORTC[PC3]     : LED
 *
 * PORTD          : LEDx8
 *
 * PORTC[PC1..PC2]: POTx2
 *
 */ 

#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

volatile uint8_t pot_data[2];
volatile uint8_t pot_n;

ISR(ADC_vect)
{
	switch (pot_n) {
	case 0:
		pot_data[0] = ADCH;
		pot_n = 1;
		
		// リファレンス電圧: AVCC, 変換結果は左詰め, ADC2シングルエンド入力
		ADMUX = (1 << REFS0) | (1 << ADLAR) | (1 << MUX1);
		ADCSRA |= (1 << ADSC);		// Start Conversion
		break;
	case 1:
		pot_data[1] = ADCH;
		pot_n = 0;
		
		// リファレンス電圧: AVCC, 変換結果は左詰め, ADC1シングルエンド入力
		ADMUX = (1 << REFS0) | (1 << ADLAR) | (1 << MUX0);
		ADCSRA |= (1 << ADSC);		// Start Conversion
		break;
	}
}

int main(void)
{
	DDRB = 0x00;
	DDRC = 0x00;
	DDRD = 0x00;

	// LED
	DDRD |= 0xFF;
	
	// LED Check
	for (int i = 0; i <= 8; i++) {
		PORTD = (0xFF >> i);
		_delay_ms(100);
	}
	
	// Potentiometer
	// Enable the ADC and its interrupt feature
	// and set the ACD clock pre-scalar to clk/128
	ADCSRA = 0x8F;
	// ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	
	pot_n = 0;
	// リファレンス電圧: AVCC, 変換結果は左詰め, ADC1シングルエンド入力
	ADMUX = (1 << REFS0) | (1 << ADLAR) | (1 << MUX0);
	
	sei();				// Enable Global Interrupts
	
	ADCSRA |= (1 << ADSC);		// Start Conversion
	
	while(1) {
		PORTD = pot_data[0];
		_delay_ms(1000);
		PORTD = pot_data[1];
		_delay_ms(1000);
	}
}
