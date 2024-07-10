/********************* pwmled.h *********************/

#define PWMLED_MAX_BRIGHTNESS 1000

typedef enum {
  PWMLED_CMD_SET_BRIGHTNESS = 0x1,
  PWMLED_CMD_GET_BRIGHTNESS,
} pwmled_cmd_t;

