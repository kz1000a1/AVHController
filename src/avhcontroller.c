//
// avhcontroller: Parse an incoming command from the USB CDC port and change function
//

#include "stm32f0xx_hal.h"
#include <string.h>
#include "can.h"
#include "error.h"
#include "printf.h"
#include "usbd_cdc_if.h"
#include "subaru_levorg_vnx.h"

// Parse an incoming command from the USB CDC port
int8_t avhcontroller_parse_str(uint8_t *buf, uint8_t len)
{

    // Convert from ASCII (2nd character to end)
    for (uint8_t i = 1; i < len; i++)
    {
        // Lowercase letters
        if(buf[i] >= 'a')
            buf[i] = buf[i] - 'a' + 10;
        // Uppercase letters
        else if(buf[i] >= 'A')
            buf[i] = buf[i] - 'A' + 10;
        // Numbers
        else
            buf[i] = buf[i] - '0';
    }

    // Process command
    switch(buf[0])
    {
		case 'v':
		case 'V':
		{
			// Report firmware version and remote
			printf_(GIT_VERSION " " GIT_REMOTE "\n");
			break;
		}

    		default:
    		// Error, unknown command
    		return -1;
    }

    return 0;
}

