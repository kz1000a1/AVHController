#ifndef __SUBARU_LEVORG_VNX_H__
#define __SUBARU_LEVORG_VNX_H__

/* #define for DEBUG_MODE */
#define no_printf_(fmt, ...)                 \
({                                           \
	if (0)                               \
		printf_(fmt, ##__VA_ARGS__); \
	0;                                   \
})

#ifdef DEBUG_MODE
#define dprintf_(fmt, ...) \
	printf_(fmt, ##__VA_ARGS__)
#else
#define dprintf_(fmt, ...) \
	no_printf_(fmt, ##__VA_ARGS__)
#endif
// DebugMode==DEBUG?printf_(fmt, ##__VA_ARGS__):no_printf_(fmt, ##__VA_ARGS__)

// Receive Only Two CAN Ids
#define CAN_ID_AVH_CONTROL 0x6BB
#define CAN_ID_AVH_STATUS  0x32B
#define CAN_ID_SHIFT       0x048
#define CAN_ID_SPEED       0x139
#define CAN_ID_ACCEL       0x040
#define CAN_ID_BELT        0x390
#define CAN_ID_DOOR        0x3AC
#define CAN_ID_EYESIGHT    0x321

struct param{
    uint8_t AvhStatus;
    uint8_t AvhHold;
    uint8_t ParkBrake;
    uint8_t SeatBelt;
    uint8_t Door;
    uint8_t EyeSight;
    uint8_t Gear;
    float Speed;
    float Brake;
    float Accel;
};

// Brake Pressure to Enable AVH
#define BRAKE_HIGH 70

// Brake Pressure to Disable AVH
#define BRAKE_LOW  10

// AVH CONTROL STATUS
enum avh_control_status {
    ENGINE_STOP,
    WAIT,
    READY
};

// STATUS
enum status {
    PROCESSING,
    CANCELLED,
    FAILED,
    SUCCEEDED
};

// AVH STATUS
#define AVH_OFF    0
#define AVH_ON     1
#definr AVH_HOLD   2

// AVH HOLD STATUS
#define HOLD_OFF   0
#define HOLD_ON    1

// PARKING BRAKE
#define BRAKE_OFF  0
#define BRAKE_ON   1

// SAFETY_BELT
#define BELT_OFF   0
#define BELT_ON    1

// DOOR
#define DOOR_OPEN  0
#define DOOR_CLOSE 1

// LED for DEBUG
#define LED_OFF    0
#define LED_ON     1

// SHIFT
#define SHIFT_D 1
#define SHIFT_N 2
#define SHIFT_R 3
#define SHIFT_P 4

// for Calculate Check Sum
#define SUM_CHECK_ADDER (-0x3F)

#define MAX_RETRY 5

#endif /* __SUBARU_LEVORG_VNX_H_ */
