/**
 * Simple hack: forward port 2's serial traffic up to PC.
 * 
 * This lets an arduino act like a serial filter, eliminating 
 * electrical noise from USB.
 * 
 */
HardwareSerial &donor=Serial2;
HardwareSerial &uplink=Serial;
long baudrate=9600;

void setup() {
  donor.begin(baudrate);
  uplink.begin(baudrate);
}

void loop() {
  while (donor.available()) {
    int c=donor.read();
    if (c!=-1) uplink.write((unsigned char)c);
  }

  while (uplink.available()) {
    int c=uplink.read();
    if (c!=-1) donor.write((unsigned char)c);
  }
}


