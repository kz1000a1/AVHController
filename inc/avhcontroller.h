#ifndef _AVHCONTROLLER_H
#define _AVHCONTROLLER_H

int8_t avhcontroller_parse_str(uint8_t *buf, uint8_t len);

// maximum rx buffer len: command length from USB CDC port
#define AVHCONTROLLER_MTU 2 // (sizeof("M0"))

#endif // _AVHCONTROLLER_H
