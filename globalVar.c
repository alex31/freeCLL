#include "globalVar.h"

/*===========================================================================*/
/* USB related stuff.                                                        */
/*===========================================================================*/

/*
 * USB Driver structure.
 */

#if HAL_USE_SERIAL_USB
SerialUSBDriver SDU1;
BaseSequentialStream *chp = (BaseSequentialStream *) &SDU1;
#else
BaseSequentialStream *chp = (BaseSequentialStream *) &SD2;
#endif // HAL_USE_SERIAL_USB



const uint8_t *UniqProcessorId = (uint8_t *) 0x1FFF7590;
const uint8_t UniqProcessorIdLen = 12;

/*
exemple of uinq id

[1C] [0] [26] [0] [14] [47] [31] [31] [33] [35] [34] [31]
[38] [0] [3A] [0] [15] [47] [31] [31] [33] [35] [34] [31] 
[28] [0] [34] [0] [13] [47] [31] [31] [33] [35] [34] [31]
[29] [0] [33] [0] [17] [47] [31] [31] [33] [35] [34] [31] 
[1E] [0] [3A] [0] [0B] [47] [31] [33] [31] [33] [30] [37] 
 */

