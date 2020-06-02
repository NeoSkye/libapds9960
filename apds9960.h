#ifndef _APDS_9960_H_
#define _APDS_9960_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#if defined (__cplusplus)
extern "C" {
#endif

struct _apds9960_ctx;
typedef struct _apds9960_ctx apds9960_ctx;
typedef apds9960_ctx* apds9960_ctxp;

void apds9960_set_log_file(FILE* log_file);

int apds9960_new(apds9960_ctxp* new_ctx, int dev_id);
void apds9960_free(apds9960_ctxp ctx);

bool apds9960_set_chip_enabled(apds9960_ctxp ctx, bool enabled);
bool apds9960_get_chip_enabled(apds9960_ctxp ctx, bool* enabled);

bool apds9960_set_gesture_enabled(apds9960_ctxp ctx, bool enabled);
bool apds9960_get_gesture_enabled(apds9960_ctxp ctx, bool* enabled);

bool apds9960_set_proximity_enabled(apds9960_ctxp ctx, bool enabled);
bool apds9960_get_proximity_enabled(apds9960_ctxp ctx, bool* enabled);

bool apds9960_set_color_enabled(apds9960_ctxp ctx, bool enabled);
bool apds9960_get_color_enabled(apds9960_ctxp ctx, bool* enabled);

bool apds9960_set_proximity_interrupt_enabled(apds9960_ctxp ctx, bool enabled);
bool apds9960_get_proximity_interrupt_enabled(apds9960_ctxp ctx, bool* enabled);

bool apds9960_set_color_gain(apds9960_ctxp ctx, uint8_t gain);
bool apds9960_get_color_gain(apds9960_ctxp ctx, uint8_t* gain);

bool apds9960_set_proximity_gain(apds9960_ctxp ctx, uint8_t gain);
bool apds9960_get_proximity_gain(apds9960_ctxp ctx, uint8_t* gain);

bool apds9960_set_gesture_gain(apds9960_ctxp ctx, uint8_t gain);
bool apds9960_get_gesture_gain(apds9960_ctxp ctx, uint8_t* gain);

bool apds9960_set_gesture_fifo_threshold(apds9960_ctxp ctx, uint8_t threshold);
bool apds9960_get_gesture_fifo_threshold(apds9960_ctxp ctx, uint8_t* threshold);

bool apds9960_set_gesture_proximity_threshold(apds9960_ctxp ctx, uint8_t threshold);
bool apds9960_get_gesture_proximity_threshold(apds9960_ctxp ctx, uint8_t* threshold);

bool apds9960_set_gesture_exit_threshold(apds9960_ctxp ctx, uint8_t threshold);
bool apds9960_get_gesture_exit_threshold(apds9960_ctxp ctx, uint8_t* threshold);

bool apds9960_set_gesture_dimensions(apds9960_ctxp ctx, uint8_t dimensions);
bool apds9960_get_gesture_dimensions(apds9960_ctxp ctx, uint8_t* dimensions);

bool apds9960_set_gesture_offsets(apds9960_ctxp ctx, int8_t off_u, int8_t off_d, int8_t off_l, int8_t off_r);
bool apds9960_get_gesture_offsets(apds9960_ctxp ctx, int8_t* off_u, int8_t* off_d, int8_t* off_l, int8_t* off_r);

bool apds9960_set_adc_integration_time(apds9960_ctxp ctx, uint8_t value);
bool apds9960_get_adc_integration_time(apds9960_ctxp ctx, uint8_t* value);

//0-3, 0=100mA, 1=50mA, 2=25mA, 3=12.5mA
bool apds9960_set_proximity_led_strength(apds9960_ctxp ctx, uint8_t value);
bool apds9960_get_proximity_led_strength(apds9960_ctxp ctx, uint8_t* value);

bool apds9960_set_gesture_led_strength(apds9960_ctxp ctx, uint8_t value);
bool apds9960_get_gesture_led_strength(apds9960_ctxp ctx, uint8_t* value);

bool apds9960_set_led_boost(apds9960_ctxp ctx, uint8_t value);
bool apds9960_get_led_boost(apds9960_ctxp ctx, uint8_t* value);

bool apds9960_set_gmode(apds9960_ctxp ctx, bool enabled);
bool apds9960_get_gmode(apds9960_ctxp ctx, bool* enabled);

bool apds9960_clear_interrupts(apds9960_ctxp ctx);

bool apds9960_read_clear_light_level(apds9960_ctxp ctx, uint16_t* level);

bool apds9960_read_proximity(apds9960_ctxp ctx, uint8_t* proximity);

#define APDS9960_MAX_GESTURE_POINTS 32

typedef struct _apds9960_gesture_point
{
	uint8_t up;
	uint8_t down;
	uint8_t left;
	uint8_t right;
} apds9960_gesture_point;

bool apds9960_read_gesture_fifo(apds9960_ctxp ctx, apds9960_gesture_point* point_buffer, uint8_t point_buffer_size, uint8_t* num_points_recv);

typedef enum _apds9960_gesture
{
	kGestureNone,
	kGestureSwipeUp,
	kGestureSwipeDown,
	kGestureSwipeLeft,
	kGestureSwipeRight,
	kGestureClockwise,
	kGestureCounterClockwise
} apds9960_gesture;

apds9960_gesture apds9960_guess_gesture(apds9960_ctxp ctx);

#if defined (__cplusplus)
}
#endif

#define LIBAPDS9960_OK 0
#define LIBAPDS9960_ENOMEM -1
#define LIBAPDS9960_EOPEN -2
#define LIBAPDS9960_EINTERNAL -3
#define LIBAPDS9960_EINVALIDID -4
#define LIBAPDS9960_EINITCHIP -5

#endif //_APDS_9960_H_
