#include <avr/common.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>

#ifndef cbi
#define cbi(sfr,bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr,bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define heartbeat(msec) \
	sbi(PORTC,PINC7); \
	_delay_ms(msec); \
	cbi(PORTC,PINC7); \
	_delay_ms(msec);

/* ATmega16U4/32U4 Datasheet
 * 8.9 Watchdog Timer
 *
 * The code below is a small modification of the "wdt_enable()" macro included with avr-gcc.  The original macro
 * sets the WDE bit which puts the system into system-reset mode.  Since we simply need to bring the system out
 * of sleep mode, we set WDIE bit instead. */
static void __inline__
__attribute__ ((__always_inline__)) wdt_interrupt_enable(uint8_t value)
{
	if (_SFR_IO_REG_P (_WD_CONTROL_REG))
	{
		__asm__ __volatile__ (
				"in __tmp_reg__,__SREG__" "\n\t"
				"cli" "\n\t"
				"wdr" "\n\t"
				"out %0, %1" "\n\t"
				"out __SREG__,__tmp_reg__" "\n\t"
				"out %0, %2" "\n \t"
				: /* no outputs */
				: "I" (_SFR_IO_ADDR(_WD_CONTROL_REG)),
				"r" ((uint8_t)(_BV(_WD_CHANGE_BIT) | _BV(WDIE))),
				"r" ((uint8_t) ((value & 0x08 ? _WD_PS3_MASK : 0x00) |
						_BV(WDIE) | (value & 0x07)) )
				: "r0"
		);
	}
	else
	{
		__asm__ __volatile__ (
				"in __tmp_reg__,__SREG__" "\n\t"
				"cli" "\n\t"
				"wdr" "\n\t"
				"sts %0, %1" "\n\t"
				"out __SREG__,__tmp_reg__" "\n\t"
				"sts %0, %2" "\n \t"
				: /* no outputs */
				: "n" (_SFR_MEM_ADDR(_WD_CONTROL_REG)),
				"r" ((uint8_t)(_BV(_WD_CHANGE_BIT) | _BV(WDIE))),
				"r" ((uint8_t) ((value & 0x08 ? _WD_PS3_MASK : 0x00) |
						_BV(WDIE) | (value & 0x07)) )
				: "r0"
		);
	}
}


int main( void )
{
	static uint8_t wakeup_counter = 0;
	cli( );
	cbi(MCUSR, WDRF);
	wdt_disable( );

	DDRC = 1 << PINC7;

main_wakeup_loop:
	power_spi_disable( );
	power_adc_disable( );
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	if (0 == (wakeup_counter % 38)) wdt_interrupt_enable(WDTO_4S);
	else wdt_interrupt_enable(WDTO_8S);
	wdt_reset( );
	sleep_enable( );
	sei( );
	sleep_cpu( );

	heartbeat(25);

	sleep_disable( );
	wakeup_counter++;
	goto main_wakeup_loop;

	return 0;
}
