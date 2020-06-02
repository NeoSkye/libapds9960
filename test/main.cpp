#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <chrono>
#include <thread>

#include <apds9960.h>

using namespace std::chrono_literals;

void dump_gesture_info(apds9960_gesture_point* points, int num_points)
{
	int v[APDS9960_MAX_GESTURE_POINTS];
	int h[APDS9960_MAX_GESTURE_POINTS];
	printf("Got %d points\n", num_points);
	for (int i = 0; i < num_points; ++i)
	{
		v[i] = (int)points[i].up - points[i].down;
		h[i] = (int)points[i].left - points[i].right;
		printf("U: %u\tD: %u\tL: %u\tR: %u\t\tV: %d\tH: %d",
			points[i].up, points[i].down, points[i].left, points[i].right,
			v[i], h[i]);
		if (i > 0)
			printf("\tdV: %d\tdH: %d\n", v[i] - v[i - 1], h[i] - h[i - 1]);
		else
			printf("\n");
	}

}

int main(int argc, char** argv)
{
	apds9960_set_log_file(stdout);

	apds9960_ctxp apds9960;
	int ret = apds9960_new(&apds9960, 1);
	if (ret) return ret;

	/*if (!apds9960_set_color_enabled(apds9960, true))
		return -1;*/

	if (!apds9960_set_proximity_led_strength(apds9960, 0x0))
		return -1;

	if (!apds9960_set_proximity_gain(apds9960, 0x3))
		return -1;

	if (!apds9960_set_proximity_enabled(apds9960, true))
		return -1;

	//if (!apds9960_set_gesture_dimensions(apds9960, 0))
	//	return -1;

	if (!apds9960_set_gesture_proximity_threshold(apds9960, 20))
		return -1;

	if (!apds9960_set_gesture_exit_threshold(apds9960, 10))
		return -1;

	if (!apds9960_set_gesture_offsets(apds9960, 4, 0, 0, 0))
		return -1;

	if (!apds9960_set_gesture_gain(apds9960, 0x2))
		return -1;

	if (!apds9960_set_gesture_enabled(apds9960, true))
		return -1;

	if (!apds9960_set_gesture_led_strength(apds9960, 0x0))
		return -1;

	//uint8_t tmp;
	//if (apds9960_get_gesture_led_strength(apds9960, &tmp))
	//	printf("Gesture LED str: %u\n", tmp);

	if (!apds9960_set_led_boost(apds9960, 0x3))
		return -1;

	while (true)
	{
		std::this_thread::sleep_for(10ms);

		/*uint16_t level;
		apds9960_read_clear_light_level(apds9960, &level);
		printf("AL level: %u\n", level);*/

		apds9960_gesture gesture = apds9960_guess_gesture(apds9960);
		switch (gesture)
		{
		case kGestureSwipeUp:
			printf("Up\n");
			break;
		case kGestureSwipeDown:
			printf("Down\n");
			break;
		case kGestureSwipeLeft:
			printf("Left\n");
			break;
		case kGestureSwipeRight:
			printf("Right\n");
			break;
		default:
			break;
		}

		//apds9960_set_gmode(apds9960, false);

		//std::this_thread::sleep_for(1s);

		//bool pen;
		//uint8_t proximity;
		//if (apds9960_read_proximity(apds9960, &proximity))
		//{
		//	printf("Proximity: %u\n", proximity);
		//}
		//else if (apds9960_get_proximity_enabled(apds9960, &pen))
		//{
		//	printf("proximity enabled: %s\n", pen ? "enabled" : "disabled");
		//}
	}

	apds9960_free(apds9960);
}
