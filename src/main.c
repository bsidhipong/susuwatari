/*  Copyright (c) 2017 Bill Sidhipong <bill.sidhipong@atomicresearchlab.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

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

/* WDT - Watchdog timer in interrupt mode
 * The code below is a small modification of the "wdt_enable()" macro included with avr-gcc.  The original macro
 * sets the WDE bit which puts the system into system-reset mode.  Since we simply need to bring the system out
 * of sleep mode, we set WDIE bit instead. */
static __inline__
__attribute__ ((__always_inline__))
void wdt_interrupt_enable(const uint8_t value)
{
	__asm__ __volatile__ (
		"in __tmp_reg__, __SREG__" "\n\t"
		"cli" "\n\t"
		"sts %0, %1" "\n\t"
		"sts %0, %2" "\n \t"
		"out __SREG__, __tmp_reg__" "\n\t"
		"wdr" "\n\t"
		: /* no outputs */
		: "n" (_SFR_MEM_ADDR(_WD_CONTROL_REG)),
		"r" ((uint8_t)(_BV(_WD_CHANGE_BIT) | _BV(WDE))),
		"r" ((uint8_t) ((value & 0x08 ? _WD_PS3_MASK : 0x00) |
				_BV(WDIE) | (value & 0x07)) )
		: "r0"
	);
}

ISR( WDT_vect ) {
}

int main( void )
{
	static uint8_t wakeup_counter = 0;

	/* MCUSR: MCU Status Register
	 * • Bit 3 – WDRF: Watchdog Reset Flag
	 * This bit is set if a Watchdog Reset occurs. The bit is reset by a Power-on Reset, or
	 * by writing a logic zero to the flag.
	 *
	 * WDTCSR: Watchdog Timer Control Register
	 * • Bit 3 - WDE: Watchdog System Reset Enable
	 * WDE is overridden by WDRF in MCUSR. This means that WDE is always set when WDRF is set.
	 * To clear WDE, WDRF must be cleared first. This feature ensures multiple resets during 
	 * conditions causing failure, and a safe start-up after the failure.*/
	cbi(MCUSR, WDRF);
	wdt_disable( );

	/* Set up on-board red LED for diagnostics */
	sbi(DDRC, DDC7);

	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	/* Disable hardware subsystems we do not need while we enter low-power sleep mode. */
	power_spi_disable( );
	power_twi_disable( );
#if defined(__AVR_HAVE_PRR1_PRUSB)
	/* CLKSEL0: Clock Selection Register 0
	 * See section 6.11.1 for complete discussion on clock selection source for ATmega32u4 */
	sbi(USBCON, FRZCLK);
	cbi(PLLCSR, PLLE);
	sbi(CLKSEL0, RCE);
	loop_until_bit_is_set(CLKSTA, RCON);
	cbi(CLKSEL0, CLKS);
	cbi(CLKSEL0, EXTE);
	cbi(USBCON, USBE);
	power_usb_disable( );
#endif
	/* ADCSRA: ADC Control and Status Register A
	 * • Bit 7 – ADEN: ADC Enable
	 * Writing this bit to one enables the ADC. By writing it to zero, the ADC is turned off.
	 * Turning the ADC off while a conversion is in progress, will terminate this conversion. */
	cbi(ADCSRA, ADEN);
	/* PRR0: Power Reduction Register 0
	 * • Bit 0 - PRADC: Power Reduction ADC
	 * Writing a logic one to this bit shuts down the ADC. The ADC must be disabled before shut down.
	 * The analog comparator cannot use the ADC input MUX when the ADC is shut down. */
	power_adc_disable( );
	/* Disable timers on the ATmega */
#if defined(__AVR_HAVE_PRR_PRTIM0) || defined(__AVR_HAVE_PRR0_PRTIM0)
	power_timer0_disable( );
#endif
#if defined(__AVR_HAVE_PRR_PRTIM1) || defined(__AVR_HAVE_PRR0_PRTIM1)
	power_timer1_disable( );
#endif
#if defined(__AVR_HAVE_PRR_PRTIM2)
	power_timer2_disable( );
#endif
#if defined(__AVR_HAVE_PRR1_PRTIM3)
	power_timer3_disable( );
#endif
#if defined(__AVR_HAVE_PRR1_PRTIM4)
	power_timer4_disable( ); /* in avr-libc commit 2536, fix #50439 */
#endif
	/* Disable serial ports */
#if defined (__AVR_HAVE_PRR_PRUSART0)
	power_usart0_disable( );
#endif
#if defined (__AVR_HAVE_PRR1_PRUSART1)
	power_usart1_disable( );
#endif

main_wakeup_loop:
	if (0 == (wakeup_counter % 38)) wdt_interrupt_enable(WDTO_4S);
	else wdt_interrupt_enable(WDTO_8S);
	sleep_enable( );
	sei( );
	sleep_cpu( );
	heartbeat(25);

	sleep_disable( );
	wakeup_counter++;
	goto main_wakeup_loop;

	return 0;
}
