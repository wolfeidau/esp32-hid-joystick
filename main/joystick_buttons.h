#ifndef __JOYSTICK_BUTTONS_H__
#define __JOYSTICK_BUTTONS_H__

#ifdef __cplusplus
extern "C" {
#endif

// J1 connector
#define BUTTON_1      14
#define BUTTON_2      25
#define BUTTON_3      26
#define BUTTON_4      27

// J2 connector
#define BUTTON_5      4
#define BUTTON_6      5
#define BUTTON_7      16
#define BUTTON_8      17
#define BUTTON_9      18
#define BUTTON_10     19

QueueHandle_t * joystick_buttons_init(void);

typedef struct {
    uint8_t pin;
} button_t;

typedef struct {
    uint16_t state;
} joystick_buttons_event_t;

#ifdef __cplusplus
}
#endif

#endif /* __JOYSTICK_BUTTONS_H__ */