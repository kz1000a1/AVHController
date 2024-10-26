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
	DebugMode==DEBUG?printf_(fmt, ##__VA_ARGS__):no_printf_(fmt, ##__VA_ARGS__)
#else
#define dprintf_(fmt, ...) \
	no_printf_(fmt, ##__VA_ARGS__)
#endif

// Receive Only Two CAN Ids
#define CAN_ID_AVH_CONTROL 0x6BB
#define CAN_ID_AVH_STATUS  0x32B
#define CAN_ID_SHIFT       0x048
#define CAN_ID_SPEED       0x139
#define CAN_ID_ACCEL       0x040

// Brake Pressure to Start AVH
#define BRAKE_HIGH 50

// AVH CONTROL STATUS
enum avh_control_status {
    ENGINE_STOP,
    NOT_READY,
    READY
};

// AVH STATUS
#define AVH_OFF false
#define AVH_ON  true

/*
enum avh_status {
    AVH_OFF,
    AVH_ON
};
*/

// STATUS
enum status {
    PROCESSING,
    CANCELLED,
    FAILED,
    SUCCEEDED
};

// MODE
enum debug_mode {
    NORMAL,
    DEBUG,
    CANDUMP
};

// SHIFT
enum shift {
    D = 1,
    N,
    R,
    P
};

extern enum debug_mode DebugMode;

// for Calculate Check Sum
#define SUM_CHECK_ADDER (-0x3F)


#define MAX_RETRY 2

#endif /* __SUBARU_LEVORG_VNX_H_ */
