#include <avr/io.h>
#include <avr/interrupt.h>

#include "BlowerRamping.h"

// globals for ADC averaging
volatile unsigned char aRunningAverageBuffer[MAX_ADC_HIST];
volatile unsigned char ucADCMeanValue;

// globals for key input
volatile unsigned char aKeyState[MAX_KEY_CHECKS];	// array that maintains bounce status
volatile unsigned char ucDebounceIndex;          	// pointer into aKeyState[]
volatile unsigned char ucDebouncedState;			// debounced state of the switch
volatile unsigned char ucOldDebouncedState;			// old state of the switch



int main()
{
    InitHardware();
    InitTimer1CompareAInt();
    while(1);
};

void InitHardware(void)
{
    // ==== Portpins ====
    // CPU LED is output
    DDR_LED_CPU = DDR_LED_CPU | _BV(BIT_LED_CPU);

    // Status LED is output
    DDR_LED_STAT = DDR_LED_STAT | _BV(BIT_LED_STAT);
    PORT_LED_STAT = PORT_LED_STAT | _BV(BIT_LED_STAT);  // LED off (active low)

    // external status is output
    DDR_LED_EXSTAT = DDR_LED_EXSTAT | _BV(BIT_LED_EXSTAT);
    PORT_LED_EXSTAT = PORT_LED_EXSTAT & ~_BV(BIT_LED_EXSTAT);  // ext. stat off

    // PWM pin is output
    DDR_PWM = DDR_PWM | _BV(BIT_PWM);
    PORT_PWM = PORT_PWM & ~_BV(BIT_PWM);    // turn off DAC
    
    // ==== ADC Stuff below ====
    // prepare ADC (disconnect digital port)
    DIDR0 = DIDR0 | _BV(ADC0D);

    // MUX & Ref
    ADMUX = 0x00;   // Channel0, VCC as reference

    // Auto Trigger Source = free running, left adjust 10bit 
    ADCSRB = _BV(ADLAR);

    // Prescaler 1:128, free running mode, enable ADC, 62,5KHz Sample Rate
    ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);

    // ==== PWM Output on OC0B (PA7) ====
    TCCR0A = (1<<COM0B1) | (1<<WGM01) | (1<<WGM00); // 0xA3 Mode 3 Fast-PWM TOP=0xFF
    TCCR0B = (1<<CS00);  // no prescaler -> 31250 Hz PWM frequency

    OCR0B = 0x00;

};

void InitTimer1CompareAInt(void)
{
    TIFR0 |= (1<<OCF1A);                // Interrupt Request loeschen
    TIMSK1 |= (1<<OCIE1A);              // Enable Output Compare A Interrupt

    OCR1AH = 124 >> 8;                // CompareTime (16bit) 
    OCR1AL = 124 & 0xFF;              // CompareTime (16bit) 

    TCNT1 = 0;
    TCCR1B   |= 1 << WGM12;             // CTC Mode 4 (16bit CTC)

    TCCR1B   |= (1<<CS01) | (1<<CS00);  // 64 prescaler, Timer0 Start
    sei();
}

 
// time management handler, called every 1ms via CTC mode interrupt
ISR(TIM1_COMPA_vect)
{
    static unsigned int uiBlinkDelay = 500;

    if(!(uiBlinkDelay--))
    {
        PORT_LED_CPU = PIN_LED_CPU ^ _BV(BIT_LED_CPU);
        uiBlinkDelay = 500;
    };
    DebounceKeys();
}

// ADC interrupt
ISR(ADC_vect)
{
    unsigned int uiLoop;
    unsigned int uiSumm;

    // read sample from ADH
    // we only have 8bit PWM, so 8bit ADC is sufficient
    volatile unsigned char ucCurrentSample = ADCH;

    // move all values one index to the front
    // value in Index 0 gets lost, MAX_ADC_HIST-1 gets free for
    // current sample

    for(uiLoop = 1; uiLoop < MAX_ADC_HIST; uiLoop++)
    {
        aRunningAverageBuffer[uiLoop-1] = aRunningAverageBuffer[uiLoop];
    };

    // current sample on last index
    aRunningAverageBuffer[(MAX_ADC_HIST-1)] = ucCurrentSample;

    uiSumm = 0;
    
    // add up all data
    for(uiLoop=0;uiLoop<MAX_ADC_HIST; uiLoop++)
    {
        uiSumm = uiSumm + aRunningAverageBuffer[uiLoop];
    };

    // update average
    ucADCMeanValue = (unsigned char)(uiSumm / MAX_ADC_HIST);
    OCR0B = ucADCMeanValue;
        
}


// Keyboard debouncing 
void DebounceKeys(void)
{
	unsigned int uiLoop;
    volatile unsigned char  ucTempState;		// needed for array checking

	ucOldDebouncedState = ucDebouncedState;		// save old value to check if something has changed

    // read hardware here
	aKeyState[ucDebounceIndex] = (~(PINA & _BV(BIT_EN_INP))) & _BV(BIT_EN_INP);// we only process a single key (low active), mask rest
	++ucDebounceIndex;

	ucTempState = 0xFF;

    // iterate through the last states and see if each
    // bit of each state is 1
	for(uiLoop=0; uiLoop < (MAX_KEY_CHECKS-1); uiLoop++)
		ucTempState = ucTempState & aKeyState[uiLoop];

	ucDebouncedState = ucTempState;

    // wrap around pointer 
	if(ucDebounceIndex >= MAX_KEY_CHECKS)
		ucDebounceIndex=0;

    // check if external enable input changed
    if((ucOldDebouncedState & _BV(BIT_EN_INP)) != (ucDebouncedState & _BV(BIT_EN_INP)))
    {
        // check if it was a low to high transition
        if(ucDebouncedState & _BV(BIT_EN_INP))
        {
            PORT_LED_EXSTAT = PIN_LED_EXSTAT ^ _BV(BIT_LED_EXSTAT);  // ext. stat off            
        };
    };
}
