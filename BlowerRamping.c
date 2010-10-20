/*

Main source file for blower ramping board
Software written for a ATTINY84 with internal RC Oscillator (8MHz)

Converts a 3 bit error code from the SWM frequncy inverter to
a morse style status code on a LED.

Ramps up to preset speed (via potentiometer) in 10 seconds and down
as well.

*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "BlowerRamping.h"

const char aFWID[] PROGMEM = "Blower Ramping ATtiny84 V1.0 (c) MRU 20.10.2010 17:21 Uhr";

// globals for ADC averaging
volatile unsigned char aRunningAverageBuffer[MAX_ADC_HIST];
volatile unsigned char ucADCMeanValue;

// globals for key input
volatile unsigned char aKeyState[MAX_KEY_CHECKS];	// array that maintains bounce status
volatile unsigned char ucDebounceIndex;          	// pointer into aKeyState[]
volatile unsigned char ucDebouncedState;			// debounced state of the switch
volatile unsigned char ucOldDebouncedState;			// old state of the switch

// globals for ramping
volatile unsigned char ucTargetSpeed;               // 8 bit speed value (0-100%)
volatile unsigned char ucCurrentSpeed;              // 8 bit speed value (0-100%)

volatile unsigned long long ullRampingValue;        // accumulator for integer arithmetics
volatile unsigned long long ullRampingTargetValue;  // target speed 
volatile long long llIncrement;                     // delta per 1ms step 

// error indicator (morse code)
volatile unsigned char ucErrorCode;

// state enums
volatile eStateType eRampingState;
volatile eMorseStateType eMorseState;

// main loop

int main()
{
    InitHardware();
    InitTimer1CompareAInt();

    eRampingState = eRampedDown;
    eMorseState = eMorsePause;

    OCR0B = 0;

    // wait 10ms to get a stable mean ADC
    _delay_ms(10.0);

    // set parameters
    ucCurrentSpeed = 0;
    ucTargetSpeed = ucADCMeanValue;

    ullRampingTargetValue = RAMP_TIME_TICKS * (unsigned long long)ucTargetSpeed;
    ullRampingValue = RAMP_TIME_TICKS * (unsigned long long)ucCurrentSpeed;

    llIncrement = (long long)(ucTargetSpeed-ucCurrentSpeed);

    // main loop
    while(1)
    {
        // check states

        // if we are in ramped down state, master enable is off
        if(eRampingState == eRampedDown)
            PORT_SWM_EN = PORT_SWM_EN | _BV(BIT_SWM_EN);

        // if we are ramping up, it would be wise to have the SWM enabled...
        // SWM is low active
        if(eRampingState == eRampingUp)
            PORT_SWM_EN = PORT_SWM_EN & ~(_BV(BIT_SWM_EN));

        // we are running at desired speed, let SWM enable active
        if(eRampingState == eRampedUp)
            PORT_SWM_EN = PORT_SWM_EN & ~(_BV(BIT_SWM_EN));

        // we are ramping down, external enable is already off,
        // but we have to keep SWM enable active till full stop of motor
        if(eRampingState == eRampingDown)
            PORT_SWM_EN = PORT_SWM_EN & ~(_BV(BIT_SWM_EN));
    };
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

    // SWM enable pin is output
    DDR_SWM_EN = DDR_SWM_EN | _BV(BIT_SWM_EN);
    PORT_SWM_EN = PORT_SWM_EN | _BV(BIT_SWM_EN);  // SWM enable off (is active low)

    // PWM pin is output
    DDR_PWM = DDR_PWM | _BV(BIT_PWM);
    PORT_PWM = PORT_PWM & ~_BV(BIT_PWM);    // turn off DAC

    // ERROR Pins on PB are input;
    DDRB = 0x00;

    // enable Pullups on PB0 to PB2
    PORTB = 0x07;
    
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

// ================================================================ 
// time management handler, called every 1ms via CTC mode interrupt
// ================================================================ 

ISR(TIM1_COMPA_vect)
{
    static unsigned int uiBlinkDelay = 500;
    static unsigned int uiMorseDelay = MORSE_SYNC_PAUSE;
    static unsigned char ucCode = 0;

    if(eRampingState == eRampingUp)
    {
        if(OCR0B == ucADCMeanValue)
        {
            eRampingState = eRampedUp;
        }
        else
        {
            // add delta
            ullRampingValue = ullRampingValue + llIncrement;

            // clip delta if greater then target
            if(ullRampingValue > ullRampingTargetValue)
                ullRampingValue = ullRampingTargetValue;

            // convert to DAC range
            OCR0B = (unsigned char)(ullRampingValue / RAMP_TIME_TICKS);
        }
    };
    
    if(eRampingState == eRampingDown)
    {
        if(OCR0B == 0)
        {
            eRampingState = eRampedDown;
        }
        else
        {
            // add delta
            ullRampingValue = ullRampingValue - llIncrement;

            // clip delta if greater then target
            if(ullRampingValue < ullRampingTargetValue)
                ullRampingValue = ullRampingTargetValue;

            // convert to DAC range
            OCR0B = (unsigned char)(ullRampingValue / RAMP_TIME_TICKS);
        };        
    };

    // in ramped up state we have direct control via the potentiometer
    if(eRampingState == eRampedUp)
        OCR0B = ucADCMeanValue;

    // read in error pins from SWM controller
    // we need only the lower 3 bits
    ucErrorCode = PINB & 0x07; 
        
    // ==== CPU LED blink code below ====
    if(!(uiBlinkDelay--))
    {
        PORT_LED_CPU = PORT_LED_CPU ^ _BV(BIT_LED_CPU);
        uiBlinkDelay = 500;
    };

    // ==========================
    // ==== morse code below ====
    // ==========================

    // LED long off to "sync user"
    if(eMorseState == eMorsePause)
    {
        PORT_LED_STAT = PORT_LED_STAT | _BV(BIT_LED_STAT);          // switch off LED (active low)
        PORT_LED_EXSTAT = PORT_LED_EXSTAT & ~(_BV(BIT_LED_EXSTAT)); // ext. stat off 

        // check if delay has expired
        if(!(uiMorseDelay--))
        {
            uiMorseDelay = MORSE_LONG_BIT;
            eMorseState = eMorseStart;
        };
    };

    // start bit (longer than the morse code)
    if(eMorseState == eMorseStart)
    {
        // switch on LEDs
        PORT_LED_STAT = PORT_LED_STAT & ~(_BV(BIT_LED_STAT));   // switch on LED (active low)
        PORT_LED_EXSTAT = PORT_LED_EXSTAT | _BV(BIT_LED_EXSTAT);// ext. stat on 

        // check if delay has expired
        if(!(uiMorseDelay--))
        {
            uiMorseDelay = MORSE_SYNC_PAUSE;// load delay counter
            eMorseState = eMorseDelay;      // next state
            ucCode = ucErrorCode;           // prepare static error counter for countdown
        };
    };

    if(eMorseState == eMorseDelay)
    {
        // LEDs off
        PORT_LED_STAT = PORT_LED_STAT | _BV(BIT_LED_STAT);          // switch off LED (active low)
        PORT_LED_EXSTAT = PORT_LED_EXSTAT & ~(_BV(BIT_LED_EXSTAT)); // ext. stat off 

        // check if delay has expired
        if(!(uiMorseDelay--))
        {
            uiMorseDelay = MORSE_SHORT_BIT;     // load delay counter
            eMorseState = eMorseCodeOutputOn;   // next state
            ucCode = ucErrorCode;               // prepare static error counter for countdown
        };
    };

    if((eMorseState == eMorseCodeOutputOn) || (eMorseState == eMorseCodeOutputOff))
    {
        // check if error code is != 0
        if(ucCode)
        {
            // check if we are in the "ON" Phase of morse
            if(eMorseState == eMorseCodeOutputOn)
            {
                // switch on LEDs
                PORT_LED_STAT = PORT_LED_STAT & ~(_BV(BIT_LED_STAT));   // switch on LED (active low)
                PORT_LED_EXSTAT = PORT_LED_EXSTAT | _BV(BIT_LED_EXSTAT);// ext. stat on 
        
                // check if delay has expired
                if(!(uiMorseDelay--))
                {
                    uiMorseDelay = MORSE_BIT_PAUSE;
                    eMorseState = eMorseCodeOutputOff;
                };
            };
            
            // check if we are in the "OFF" phase of morse
            if(eMorseState == eMorseCodeOutputOff)
            {
                // switch off LEDs
                PORT_LED_STAT = PORT_LED_STAT | _BV(BIT_LED_STAT);          // switch off LED (active low)
                PORT_LED_EXSTAT = PORT_LED_EXSTAT & ~(_BV(BIT_LED_EXSTAT)); // ext. stat off 
        
                // check if delay has expired
                if(!(uiMorseDelay--))
                {
                    uiMorseDelay = MORSE_SHORT_BIT;
                    eMorseState = eMorseCodeOutputOn;
                    ucCode--;
                };
            };
        }
        else
        {
                uiMorseDelay = MORSE_SYNC_PAUSE;
                eMorseState = eMorsePause;
        };
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
            // transition from eRampedDown to eRampingUp
            if(eRampingState == eRampedDown)
            {
                // ramp to current set point
                ucTargetSpeed = ucADCMeanValue;

                // from current speed
                ucCurrentSpeed = OCR0B;

                // calculate step sizes
                ullRampingTargetValue = RAMP_TIME_TICKS * (unsigned long long)ucTargetSpeed;
                ullRampingValue = RAMP_TIME_TICKS * (unsigned long long)ucCurrentSpeed;

                llIncrement = (long long)(ucTargetSpeed-ucCurrentSpeed);

                eRampingState = eRampingUp;
            }
            else
            // transition from eRampingDown to eRampingUp
            if(eRampingState == eRampingDown)
            {
                // ramp to current set point
                ucTargetSpeed = ucADCMeanValue;

                // from current speed
                ucCurrentSpeed = OCR0B;

                // calculate step sizes
                ullRampingTargetValue = RAMP_TIME_TICKS * (unsigned long long)ucTargetSpeed;
                ullRampingValue = RAMP_TIME_TICKS * (unsigned long long)ucCurrentSpeed;

                llIncrement = (long long)(ucTargetSpeed-ucCurrentSpeed);
                eRampingState = eRampingUp;
            };
        };

        // suspect it was a high to low transistion then...
        if(!(ucDebouncedState & _BV(BIT_EN_INP)))
        {
            if(eRampingState == eRampingUp)
            {
                // ramp to zero
                ucTargetSpeed = 0;

                // from current speed
                ucCurrentSpeed = OCR0B;

                // calculate step sizes
                ullRampingTargetValue = RAMP_TIME_TICKS * (unsigned long long)ucTargetSpeed;
                ullRampingValue = RAMP_TIME_TICKS * (unsigned long long)ucCurrentSpeed;

                llIncrement = (long long)(ucCurrentSpeed-ucTargetSpeed);

                eRampingState = eRampingDown;
            }
            else
            if(eRampingState == eRampedUp)
            {
                // ramp to zero
                ucTargetSpeed = 0;

                // from current speed
                ucCurrentSpeed = OCR0B;

                // calculate step sizes
                ullRampingTargetValue = RAMP_TIME_TICKS * (unsigned long long)ucTargetSpeed;
                ullRampingValue = RAMP_TIME_TICKS * (unsigned long long)ucCurrentSpeed;

                llIncrement = (long long)(ucCurrentSpeed-ucTargetSpeed);

                eRampingState = eRampingDown;
            };
        };
    };
}

