/*
 Adapt raw A-packets to a bidirectional communication channel,
 with connection tracking.  Arduino only.
*/
#ifndef __AURORA_COMMS_H
#define __AURORA_COMMS_H

#include "milli.h" /* for timing */
#include "serial_packet.h" /* for A-packet serial comms */

/** This class manages communication via an A_packet_formatter,
 including timeouts. */
template <class HardwareSerial>
class CommunicationChannel {
public:
  HardwareSerial &backend;
  A_packet_formatter<HardwareSerial> pkt; // packet formatter
  bool is_connected; // 1 if we're recently connected; 0 if no response
  milli_t last_read; // the last time we got data back
  milli_t next_send; // the next time we should send off data

  CommunicationChannel(HardwareSerial &new_backend)
    :backend(new_backend), pkt(backend)
  {
    is_connected=0;
    last_read=milli;
    next_send=milli;
  }

  bool read_packet(A_packet &p) {
    p.valid=0;
    if (backend.available()) {
      while (-1==pkt.read_packet(p)) {}
      if (p.valid && p.length>0) {
        last_read=milli;
        next_send=milli;
        is_connected=true; // got valid packet
        return true;
      }
    }
    if (milli-next_send>500) { // read timeout
      next_send=milli;
      pkt.reset();
      pkt.write_packet(0,0,0); // send heartbeat ping packet
      is_connected=false;
    }
    return false;
  }
};



#endif



