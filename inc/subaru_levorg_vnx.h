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
#define BRAKE_HIGH 60

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
#define AVH_OFF    0b00
#define AVH_ON     0b01
#define AVH_HOLD   0b11

// AVH HOLD STATUS
#define UNHOLD     0
#define HOLD       1

// DOOR/SEAT BELT
#define OPEN       1
#define CLOSE      0

// PARKING BRAKE/DEBUG LED/EYESIGHT HOLD
#define OFF        0
#define ON         1

// AVH HOLD Released by Brake
#define CLEAR      0
#define RELEASE    1
#define BLOCK      2

// SHIFT
#define SHIFT_D 1
#define SHIFT_N 2
#define SHIFT_R 3
#define SHIFT_P 4

// for Calculate Check Sum
#define SUM_CHECK_ADDER (-0x3F)

#define MAX_RETRY 5

#endif /* __SUBARU_LEVORG_VNX_H_ */
