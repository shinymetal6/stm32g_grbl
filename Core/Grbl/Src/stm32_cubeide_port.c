/*
 * stm32_cubeide_port.c
 *
 *  Created on: Apr 28, 2020
 *      Author: fil
 */

#include "main.h"
#include "system.h"

system_t sys;
int32_t sys_position[N_AXIS];      // Real-time machine (aka home) position vector in steps.
int32_t sys_probe_position[N_AXIS]; // Last probe position in machine coordinates and steps.
volatile uint8_t sys_probe_state;   // Probing state value.  Used to coordinate the probing cycle with stepper ISR.
/*
 * author Paul use the same technique for ATC as used for the probe
 */
volatile uint8_t sys_tool_state;// toggle state value.  Used to coordinate the toggle cycle with stepper ISR.
volatile uint8_t sys_m6_state;// M6 state value.  Used to coordinate the M6 cycle with stepper ISR.
/*
 *
 */
volatile uint8_t sys_rt_exec_state;   // Global realtime executor bitflag variable for state management. See EXEC bitmasks.
volatile uint8_t sys_rt_exec_alarm;   // Global realtime executor bitflag variable for setting various alarms.
volatile uint8_t sys_rt_exec_motion_override; // Global realtime executor bitflag variable for motion-based overrides.
volatile uint8_t sys_rt_exec_accessory_override; // Global realtime executor bitflag variable for spindle/coolant overrides.
#ifdef DEBUG
volatile uint8_t sys_rt_exec_debug;
#endif


void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

  GPIOx->ODR = PortVal;
}

uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

  return ((uint16_t)GPIOx->ODR);
}

uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

  return ((uint16_t)GPIOx->IDR);
}

void LedBlink(void)
{
	uint8_t nOnFlag = GPIO_PIN_SET;
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, nOnFlag);
	nOnFlag = (nOnFlag == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
}

void LedOn(void)
{
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
}

void LedOff(void)
{
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}

void TIM_SetCompare4(TIM_TypeDef* TIMx, uint16_t Compare4)
{
  /* Check the parameters */
  assert_param(IS_TIM_LIST3_PERIPH(TIMx));
  /* Set the Capture Compare4 Register value */
  TIMx->CCR4 = Compare4;
}

uint32_t gerbil_loop(void)
{
	  settings_init(); // Load Grbl settings from EEPROM
	  stepper_init();  // Configure stepper pins and interrupt timers
	  system_init();   // Configure pinout pins and pin-change interrupt
	  spindle_init(0); // paul, added to set the spindle timers for pwm
	  memset(sys_position,0,sizeof(sys_position)); // Clear machine position.

	  // Initialize system state.


	  #ifdef FORCE_INITIALIZATION_ALARM
	    // Force Grbl into an ALARM state upon a power-cycle or hard reset.
	    sys.state = STATE_ALARM;
	  #else
	    sys.state = STATE_IDLE;
	  #endif

	  // Check for power-up and set system alarm if homing is enabled to force homing cycle
	  // by setting Grbl's alarm state. Alarm locks out all g-code commands, including the
	  // startup scripts, but allows access to settings and internal commands. Only a homing
	  // cycle '$H' or kill alarm locks '$X' will disable the alarm.
	  // NOTE: The startup script will run after successful completion of the homing cycle, but
	  // not after disabling the alarm locks. Prevents motion startup blocks from crashing into
	  // things uncontrollably. Very bad.
	  #ifdef HOMING_INIT_LOCK
	    if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { sys.state = STATE_ALARM; }
	  #endif

	  // Grbl initialization loop upon power-up or a system abort. For the latter, all processes
	  // will return to this loop to be cleanly re-initialized.
	  for(;;) {

	    // Reset system variables.
	//    delay_ms(2000);
	    uint8_t prior_state = sys.state;
	    memset(&sys, 0, sizeof(system_t)); // Clear system struct variable.
	    sys.state = prior_state;
	    sys.f_override = DEFAULT_FEED_OVERRIDE;  // Set to 100%
	    sys.r_override = DEFAULT_RAPID_OVERRIDE; // Set to 100%
	    sys.spindle_speed_ovr = DEFAULT_SPINDLE_SPEED_OVERRIDE; // Set to 100%
			memset(sys_probe_position,0,sizeof(sys_probe_position)); // Clear probe position.
	    sys_probe_state = 0;
	    /*
	     * author Paul ATC
	     */
	    sys_m6_state = 0;
	    sys_tool_state = 0;
	    /*
	     * ATC ends
	     */
	    sys_rt_exec_state = 0;
	    sys_rt_exec_alarm = 0;
	    sys_rt_exec_motion_override = 0;
	    sys_rt_exec_accessory_override = 0;

	    // Reset Grbl primary systems.
	    serial_reset_read_buffer(); // Clear serial read buffer
	    gc_init(); // Set g-code parser to default state

	    coolant_init();
	    system_init();
	    probe_init();
	    plan_reset(); // Clear block buffer and planner variables
	    st_reset(); // Clear stepper subsystem variables.
	    plan_sync_position();
	    gc_sync_position();
	    serial_enable();
		report_init_message();
	    LedBlink();

	    // Start Grbl main loop. Processes program inputs and executes them.
	    protocol_main_loop();
	  }

	  return 0;   /* Never reached */
	}

