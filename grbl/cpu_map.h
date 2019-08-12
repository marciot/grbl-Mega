/*
  cpu_map.h - CPU and pin mapping configuration file
  Part of Grbl

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/* The cpu_map.h files serve as a central pin mapping selection file for different
   processor types or alternative pin layouts. This version of Grbl supports only the
   Arduino Mega2560. */

#ifndef cpu_map_h
#define cpu_map_h

#ifdef CPU_MAP_MINI_RAMBO_1_3A_BOARD // UltiMachine Mini Rambo 1.3A

  // Serial port interrupt vectors
  #define SERIAL_RX USART0_RX_vect
  #define SERIAL_UDRE USART0_UDRE_vect
  
  // Microstepping pins
  #define X_MS1_PIN   G, 0b00000010  /*  X Microstepping Bit 1 - Pin G1 */
  #define X_MS2_PIN   G, 0b00000001  /*  X Microstepping Bit 2 - Pin G0 */
  #define Y_MS1_PIN   K, 0b10000000  /*  Y Microstepping Bit 1 - Pin K7 */
  #define Y_MS2_PIN   G, 0b00000100  /*  Y Microstepping Bit 2 - Pin G2 */
  #define Z_MS1_PIN   K, 0b01000000  /*  Z Microstepping Bit 1 - Pin K6 */
  #define Z_MS2_PIN   K, 0b00100000  /*  Z Microstepping Bit 2 - Pin K5 */
  #define E0_MS1_PIN  K, 0b00001000  /* E0 Microstepping Bit 1 - Pin K3 */
  #define E0_MS2_PIN  K, 0b00010000  /* E0 Microstepping Bit 2 - Pin K4 */
  
  // Stepper voltage reference pins
  #define XY_REF_PIN  L, 0b00001000  /*  XY Reference PWM - Pin L3 OC5A  (Arduino Mega 46) */
  #define Z_REF_PIN   L, 0b00010000  /*   Z Reference PWM - Pin L4 OC5B  (Arduino Mega 45) */
  #define E_REF_PIN   L, 0b00100000  /*   E Reference PWM - Pin L5 OC5C  (Arduino Mega 44) */
    
  // 1/8 Prescaler, 8-bit Fast PWM mode
  #define REF_TCCRA_REGISTER  TCCR5A
  #define REF_TCCRB_REGISTER  TCCR5B
  #define REF_TCCRA_INIT_MASK ((1<<WGM50) | (1<<COM5A1) | (1<<COM5B1) | (1<<COM5C1))
  #define REF_TCCRB_INIT_MASK ((1<<WGM52) | (1<<CS51))
  #define REF_XY_OCR_REGISTER OCR5A
  #define REF_Z_OCR_REGISTER  OCR5B
  #define REF_E_OCR_REGISTER  OCR5C
  
  // Stepper current values
  #define MOTOR_CURRENT_PWM_RANGE 2000
  #define MOTOR_CURRENT_PWM(mA) 255L * mA / MOTOR_CURRENT_PWM_RANGE
  
  #define MOTOR_XY_CURRENT  1300 // mA
  #define MOTOR_Z_CURRENT   1630 // mA
  #define MOTOR_E_CURRENT   0    // mA

  // Define step pulse output pins. NOTE: All step bit pins must be on the same port.
  #define STEP_DDR      DDRC
  #define STEP_PORT     PORTC
  #define STEP_PIN      PINC
  #define X_STEP_BIT    0 // X Step - Pin C0
  #define Y_STEP_BIT    1 // Y Step - Pin C1
  #define Z_STEP_BIT    2 // Z Step - Pin C2
  #define STEP_MASK ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits

  // Define step direction output pins. NOTE: All direction pins must be on the same port.
  #define DIRECTION_DDR     DDRL
  #define DIRECTION_PORT    PORTL
  #define DIRECTION_PIN     PINL
  #define X_DIRECTION_BIT   7 // X Dir - Pin L1
  #define Y_DIRECTION_BIT   6 // Y Dir - Pin L0
  #define Z_DIRECTION_BIT   5 // Z Dir - Pin L2
  #define DIRECTION_MASK ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits

  // Define stepper driver enable/disable output pin.
  #define STEPPERS_DISABLE_DDR   DDRA
  #define STEPPERS_DISABLE_PORT  PORTA
  #define X_DISABLE_BIT 7 // X Enable - Pin A7
  #define Y_DISABLE_BIT 6 // Y Enable - Pin A6
  #define Z_DISABLE_BIT 5 // Z Enable - Pin A5
  #define STEPPERS_DISABLE_MASK ((1<<X_DISABLE_BIT)|(1<<Y_DISABLE_BIT)|(1<<Z_DISABLE_BIT))

  // Define homing/hard limit switch input pins and limit interrupt vectors.
  // NOTE: All limit bit pins must be on the same port
  #define LIMIT_DDR       DDRB
  #define LIMIT_PORT      PORTB
  #define LIMIT_PIN       PINB
  #define X_LIMIT_BIT     6 // X Limit Min - Pin B6
  #define Y_LIMIT_BIT     5 // Y Limit Min - Pin B5
  #define Z_LIMIT_BIT     4 // Z Limit Min - Pin B4
  #define LIMIT_INT       PCIE0  // Pin change interrupt enable pin
  #define LIMIT_INT_vect  PCINT0_vect
  #define LIMIT_PCMSK     PCMSK0 // Pin change interrupt register
  #define LIMIT_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits

  // Define spindle enable and spindle direction output pins.
  #define SPINDLE_ENABLE_DDR      DDRH
  #define SPINDLE_ENABLE_PORT     PORTH
  #define SPINDLE_ENABLE_BIT      4 // Pin G4 - P1 Pin 6
  #define SPINDLE_DIRECTION_DDR   DDRH
  #define SPINDLE_DIRECTION_PORT  PORTH
  #define SPINDLE_DIRECTION_BIT   3 // Pin G3 - P1 Pin 8

  // Define flood and mist coolant enable output pins.
  #define COOLANT_FLOOD_DDR   DDRH
  #define COOLANT_FLOOD_PORT  PORTH
  #define COOLANT_FLOOD_BIT   5 // FAN 0 - Pin H5
  #define COOLANT_MIST_DDR    DDRH
  #define COOLANT_MIST_PORT   PORTH
  #define COOLANT_MIST_BIT    3 // FAN 1 - Pin H3

  // Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
  // NOTE: All CONTROLs pins must be on the same port and not on a port with other input pins (limits).
  #define CONTROL_DDR       DDRJ
  #define CONTROL_PIN       PINJ
  #define CONTROL_PORT      PORTJ
  #define CONTROL_RESET_BIT         0  // Pin J0 - P2 Pin 7
  #define CONTROL_FEED_HOLD_BIT     1  // Pin J1 - P2 Pin 5
  #define CONTROL_CYCLE_START_BIT   2  // Pin J2 - P2 Pin 3
  #define CONTROL_SAFETY_DOOR_BIT   3  // Not connected
  #define CONTROL_INT       PCIE1  // Pin change interrupt enable pin
  #define CONTROL_INT_vect  PCINT1_vect
  #define CONTROL_PCMSK     PCMSK1 // Pin change interrupt register
  #define CONTROL_MASK      ((1<<CONTROL_RESET_BIT)|(1<<CONTROL_FEED_HOLD_BIT)|(1<<CONTROL_CYCLE_START_BIT)|(1<<CONTROL_SAFETY_DOOR_BIT))

  // Define probe switch input pin.
  #define PROBE_DDR       DDRF
  #define PROBE_PIN       PINF
  #define PROBE_PORT      PORTF
  #define PROBE_BIT       0  // THERM-0
  #define PROBE_MASK      (1<<PROBE_BIT)

  // Advanced Configuration Below You should not need to touch these variables
  // Set Timer up to use TIMER3B which is attached to HEAT-0
  #define SPINDLE_PWM_MAX_VALUE     1024.0 // Translates to about 1.9 kHz PWM frequency at 1/8 prescaler
  #ifndef SPINDLE_PWM_MIN_VALUE
    #define SPINDLE_PWM_MIN_VALUE   1   // Must be greater than zero.
  #endif
  #define SPINDLE_PWM_OFF_VALUE     0
  #define SPINDLE_PWM_RANGE         (SPINDLE_PWM_MAX_VALUE-SPINDLE_PWM_MIN_VALUE)
  #define SPINDLE_TCCRA_REGISTER		TCCR3A
  #define SPINDLE_TCCRB_REGISTER		TCCR3B
  #define SPINDLE_OCR_REGISTER	  	OCR3B
  #define SPINDLE_COMB_BIT			    COM3B1

  // 1/8 Prescaler, 16-bit Fast PWM mode
  #define SPINDLE_TCCRA_INIT_MASK ((1<<WGM40) | (1<<WGM41))
  #define SPINDLE_TCCRB_INIT_MASK ((1<<WGM42) | (1<<WGM43) | (1<<CS41))
  #define SPINDLE_OCRA_REGISTER   OCR3A // 16-bit Fast PWM mode requires top reset value stored here.
  #define SPINDLE_OCRA_TOP_VALUE  0x0400 // PWM counter reset value. Should be the same as PWM_MAX_VALUE in hex.

  // Define spindle output pins.
  #define SPINDLE_PWM_DDR   DDRE
  #define SPINDLE_PWM_PORT  PORTE
  #define SPINDLE_PWM_BIT   5 // HEAT-0

#endif
#endif