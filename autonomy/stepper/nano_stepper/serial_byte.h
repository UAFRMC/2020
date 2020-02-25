/* This header defines the serial datatype we send to/from the Arduino 
*/
#ifndef __STEPPER_SERIAL_BYTE_H
#define __STEPPER_SERIAL_BYTE_H

// This is the datatype we exchange over serial.  
//   It's limited to 0-255 (0x00 - 0xFF)
typedef unsigned char serial_byte;

#define BYTE_TO_ANGLE 2 /* 0-180 byte to 0-360 angle */
#define BYTE_MAX_ANGLE 180 /* largest serial_byte value that represents an angle */
#define BYTE_HOME ((serial_byte)0xD0) /* re-home command, sent from PC side */
#define BYTE_ERROR ((serial_byte)0xE0) /* error, sent up from Arduino (plus up to 15 error types) */
#define BYTE_OK ((serial_byte)0xF0) /* "I'm alive" byte, sent out from Arduino */

inline long byte2deg(serial_byte b) { return b*BYTE_TO_ANGLE; }
inline serial_byte deg2byte(long angle_deg) { return (angle_deg+BYTE_TO_ANGLE/2)/BYTE_TO_ANGLE; }


#endif /* def(this header) */
