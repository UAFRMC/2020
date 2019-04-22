/*
 Timing used for communication and sensors.
 "milli_t" is an unsigned int, so that it wraps around
 every 65.535 seconds.  This is to force you to handle 
 overflow (rather than unsigned long which will last a few weeks then fail).
*/
#ifndef MILLI_H
#define MILLI_H

typedef unsigned int milli_t;
extern milli_t milli;

#endif
