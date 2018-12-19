/**
  Control a USB-connected 3D printer with direct gcode commands.
*/
#include <stdio.h>
#include "serial.h"

class printer_gcode {
public:
  SerialPort serial;
  printer_gcode() {
    serial.begin(115200);
    
  }
  // Check for responses from the printer, and print them.
  void poll(void) {
    while (serial.available()>0) {
      int c=serial.read();
      fputc(c,stdout);
    }
  }
  
  // Wait for this letter from printer
  void wait_letter(int target) {
    while (true) {
      while (serial.available()<=0) { /* wait */ }
      int c=serial.read();
      if (c>0) {
        fputc(c,stdout);
        if (c==target) return;
      }
    }
  }
  
  void wait(const char *str) {
    printf("GCODE WAIT FOR> %s",str); fflush(stdout);
    while (*str) wait_letter(*str++);
  }
  
  // Send this gcode or m-code to the printer
  void send(const char *string) {
    printf("GCODE OUT> %s",string); fflush(stdout);
    serial.write((char *)string);
  }
  void send(float value) {
    char buf[100];
    snprintf(buf,100,"%f",value);
    send(buf);
  }
};

