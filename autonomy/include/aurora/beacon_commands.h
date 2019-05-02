/*
 TCP socket for relaying commands from the backend
 to the beacon, such as:
  - Scan for obstacles
  - Point in a direction
  - Power off.
*/
#ifndef __AURORA_BEACON_CMD_H
#define __AURORA_BEACON_CMD_H

#include <zmq.hpp>

// Command letters:
enum {
  aurora_beacon_command_scan='S', // scan for obstacles
  aurora_beacon_command_point='P', // point sensor this direction
  aurora_beacon_command_off='O', // power off the beacon
};
typedef signed short aurora_beacon_command_angle_t;

struct aurora_beacon_command {
  // Command letter
  char letter;

  // Angle, used by scan and point commands.
  //   Angle is in degrees up from the +X axis.
  aurora_beacon_command_angle_t angle;
};


// Sends beacon commands to the beacon.  
//   Blocks until any data has been returned.
template <class T>
void send_aurora_beacon_command(char letter,
  std::vector<T> &returnData,
  aurora_beacon_command_angle_t angle=0)
{
  printf("Sending beacon command %c (angle %d)\n", letter,(int)angle);
  fflush(stdout);
  zmq::context_t context(1);
  zmq::socket_t socket(context,ZMQ_REQ); // request port
  std::string server="tcp://10.10.10.100";
  const char *beacon_str=getenv("BEACON");
  if (beacon_str) {
    server="tcp://";
    server+=beacon_str;
  }
#define aurora_beacon_command_port "1111"
  server+=":" aurora_beacon_command_port;
  socket.connect(server.c_str());
  
  zmq::message_t cmdbuf(sizeof(aurora_beacon_command));
  aurora_beacon_command c;
  c.letter=letter; c.angle=angle;
  memcpy(cmdbuf.data(),&c,sizeof(c));
  socket.send(cmdbuf);
  
  zmq::message_t reply;
  socket.recv(&reply);
  
  size_t nreturn=reply.size()/sizeof(T);
  returnData.resize(nreturn);
  memcpy(&returnData[0],reply.data(),nreturn*sizeof(T));
  
  printf("Response from beacon: %d data items of size %d bytes each\n",
    (int)nreturn,(int)sizeof(T));
}

// Receives beacon commands (on the beacon)
class aurora_beacon_command_server {
  zmq::context_t context;
  zmq::socket_t socket;
public:
  aurora_beacon_command_server() 
    :context(1), socket(context,ZMQ_REP)
  {
    socket.bind("tcp://*:" aurora_beacon_command_port);
  }
  
  // Front half of request: receive a command and return true,
  //  or else return false.
  // If this returns true, you MUST send a response, even if it's empty.
  bool request(aurora_beacon_command &c) {
    zmq::message_t cmdbuf;
    int err=socket.recv(&cmdbuf,ZMQ_NOBLOCK);
    if (err==0) { // we got one!
      if (sizeof(c)==cmdbuf.size())
        memcpy(&c,cmdbuf.data(),sizeof(c));
      else 
        printf("WARNING: Size mismatch, expected %d, got %d bytes\n",
          (int)sizeof(c), (int)cmdbuf.size());
      printf("Incoming beacon command %c (angle %d)\n",
        c.letter, (int)c.angle);
      return true;
    }
    else return false;
  }
  
  // Back half of request: send any response data
  //   (and let the caller continue)
  void response(const void *data=NULL,size_t ndata=0) {
    zmq::message_t responsebuf(ndata);
    memcpy(responsebuf.data(),data,ndata);
    socket.send(responsebuf);
  }
};

#endif



