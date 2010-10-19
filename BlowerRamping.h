#ifndef BLOWERRAMPING_H
#define BLOWERRAMPING_H

// defines for LEDs and other peripherals 

// ENABLE INPUT PIN (from XPort Board)
#define BIT_EN_INP      (1)
#define PIN_EN_INP      PINA
#define PORT_EN_INP     PORTA
#define DDR_EN_INP      DDRA

// SWM ENABLE PIN
#define BIT_SWM_EN      (2)
#define PIN_SWM_EN      PINA
#define PORT_SWM_EN     PORTA
#define DDR_SWM_EN      DDRA

// STATUS EXTERN
#define BIT_LED_EXSTAT  (3)
#define PIN_LED_EXSTAT  PINA
#define PORT_LED_EXSTAT PORTA
#define DDR_LED_EXSTAT  DDRA

// CPU LED
#define BIT_LED_CPU     (4)
#define PIN_LED_CPU     PINA
#define PORT_LED_CPU    PORTA
#define DDR_LED_CPU     DDRA

// STATUS LED
#define BIT_LED_STAT    (5)
#define PIN_LED_STAT    PINA
#define PORT_LED_STAT   PORTA
#define DDR_LED_STAT    DDRA

// PWM DAC
#define BIT_PWM         (7)
#define PIN_PWM         PINA
#define PORT_PWM        PORTA
#define DDR_PWM         DDRA

// defines
#define MAX_ADC_HIST    (20)        // running average over n samples
#define MAX_KEY_CHECKS	(5)			// number of equal key samples needed to count a key as debounced
#define RAMP_TIME_TICKS (30000UL)   // number of 1ms timer ticks to ramp from current speed to target speed

typedef enum eState
{
    eRampedDown,
    eRampingUp,
    eRampedUp,
    eRampingDown,
    eMaxState
} eStateType;

#define MORSE_SYNC_PAUSE    (1000)
#define MORSE_BIT_PAUSE     (500)
#define MORSE_SHORT_BIT     (100)
#define MORSE_LONG_BIT      (300)

typedef enum eMorseState
{
    eMorsePause,
    eMorseStart,
    eMorseDelay,
    eMorseCodeOutputOn,
    eMorseCodeOutputOff,
    eMaxStateMorse
} eMorseStateType;
    
// prototypes
extern void InitHardware(void);
extern void InitTimer1CompareAInt(void);
extern void DebounceKeys(void);

#endif
