#ifndef HCSR04_H_
#define HCSR04_H_

#include <stdint.h>

#define HCSR04_TRIG_PULSE_US 200
#define HCSR04_ECHO_TIMEOUT_US 38000

typedef void (*hcsr04_pin_write_cb_t)(uint8_t state);
typedef uint8_t (*hcsr04_pin_read_cb_t)(void);
typedef void    (*hcsr04_result_cb_t)(float distance_cm);

typedef enum {
	HCSR04_STATE_IDLE = 0,
	HCSR04_STATE_TRIGGER_HIGH,
	HCSR04_STATE_WAIT_ECHO_HIGH,
	HCSR04_STATE_MEASURING,
} HCSR04_State_t;

typedef struct {
	hcsr04_pin_write_cb_t set_trigger;
	hcsr04_pin_read_cb_t get_echo;
	hcsr04_result_cb_t on_result;
	HCSR04_State_t  state;
	uint32_t tick_ref_us;
	uint32_t echo_start_us;
} HCSR04_t;

#define HCSR04_US_TO_CM 0.01715f

void HCSR04_Init   (HCSR04_t *dev, hcsr04_pin_write_cb_t set_trig, hcsr04_pin_read_cb_t get_echo, hcsr04_result_cb_t on_result);
void HCSR04_Trigger(HCSR04_t *dev, uint32_t now_us);
void HCSR04_Tick   (HCSR04_t *dev, uint32_t now_us);
uint8_t HCSR04_IsBusy (const HCSR04_t *dev);

#endif /* HCSR04_H */
