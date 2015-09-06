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

volatile uint8_t user_sw_rd;
volatile uint8_t user_sw_data;

//---------------------------------------------------------------------------------------------
// Pin Change Interrupt & Debounce
//
void init_switches()
{
	// Pin Change Interruptの有効化
	PCICR = (1 << PCIE0);
	PCMSK0 = (1 << PB4);
	
	// TIMER0 オーバーフロー割り込みの有効化
	TCCR0B = 0x00;	// Timer0停止
	TIMSK0 = (1 << TOIE0);
}

ISR (TIMER0_OVF_vect)
{
	// 割り込みごとにLEDを点滅（デバッグ用）
	PORTC ^= (1 << PC3);
	
	// Timer0を停止
	TCCR0B = 0x00;

	if ((~PINB & (1 << PB4)) == user_sw_rd) {
		//PORTD = user_sw_rd;
		// トグル動作
		if (user_sw_rd) {			
			user_sw_data = user_sw_data ? 0 : 1;
		}
	}
	
	// Pin Change Interruptの有効化
	PCICR = (1 << PCIE0);
}

ISR (PCINT0_vect)
{
	// Pin Change Interruptを無効化
	PCICR = 0x00;
	
	//PORTD = PINB;
	user_sw_rd = (~PINB & (1 << PB4));	
	
	// Timer0を起動
	TCCR0B = 0x05;	// プリスケーラ－:1024, 1/(8MHz/1024)=128us
	TCNT0 = 96;		// 128us*(256-96)=20.48ms
}

//---------------------------------------------------------------------------------------------
// ADC
//
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

//---------------------------------------------------------------------------------------------
// Main routine
//
int main(void)
{
	DDRB = 0x00;
	DDRC = 0x00;
	DDRD = 0x00;

	// LED
	DDRD |= 0xFF;
	DDRB |= (1 << PB5);
	DDRC |= (1 << PC3);
	
	// LED Check
	PORTB |= (1 << PB5);
	PORTC |= (1 << PC3);
	for (int i = 0; i <= 8; i++) {
		PORTD = (0xFF >> i);
		_delay_ms(100);
	}
	PORTB &= ~(1 << PB5);
	_delay_ms(100);
	PORTC &= ~(1 << PC3);
	
	// Switch input/pullup
	PORTB |= (1 << PB4);
	
	init_switches();
	
	// Potentiometer
	// Enable the ADC and its interrupt feature
	// and set the ACD clock pre-scalar to clk/128
	//ADCSRA = 0x8F;
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	
	pot_n = 0;
	// リファレンス電圧: AVCC, 変換結果は左詰め, ADC1シングルエンド入力
	ADMUX = (1 << REFS0) | (1 << ADLAR) | (1 << MUX0);
	
	sei();				// Enable Global Interrupts
	
	ADCSRA |= (1 << ADSC);		// Start Conversion
	
	while(1) {
		switch (user_sw_data) {
		case 0:
			PORTB &= ~(1 << PB5);
			PORTD = pot_data[0];
			break;
		case 1:
			PORTB |= (1 << PB5);
			PORTD = pot_data[1];
			break;
		}
	}
}
