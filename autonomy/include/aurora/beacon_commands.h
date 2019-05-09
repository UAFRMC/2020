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
#include <unistd.h> // for sleep
#include <thread>
#include <vector>

// Command letters:
enum {
  aurora_beacon_command_scan='T', // take a scan for obstacles
  aurora_beacon_command_point='P', // point sensor this direction
  aurora_beacon_command_home='H', // point sensor this direction
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

struct aurora_detected_obstacle {
  signed short x,y; // location on field (cm, field coords)
  signed short height; // height above/below neighbors
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
  if(letter=='P')
    socket.setsockopt(ZMQ_RCVTIMEO,1*1000); //1 sec timeout for Point commands
  zmq::message_t reply;
  socket.recv(&reply);
  
  size_t nreturn=reply.size()/sizeof(T);
  returnData.resize(nreturn);
  memcpy(&returnData[0],reply.data(),nreturn*sizeof(T));
  
  printf("Response from beacon: %d data items of size %d bytes each\n",
    (int)nreturn,(int)sizeof(T));
}

// template<class T>
// inline void zmq_thread_waiton_reply(std:vector<)
// {
//     while(1)
//     {
//         std::vector<T> beacon_reply;
//         if(cmd.letter!=0)
//         {
//             send_aurora_beacon_command(cmd.letter,beacon_reply,cmd.angle);
//         }
//         while (cmd.letter!=0)
//         {
//             sleep(0);
//         }
//     }

// }

/* ZMQ doesn't seem to have a decent 'check if client connected' interface,
   so use the blocking interface from a thread.  Silly! */
inline void zmq_thread_poll_on_socket(zmq::socket_t &socket,
      volatile aurora_beacon_command &cmd) 
{
  while (1) {
    zmq::message_t cmdbuf;
    socket.recv(&cmdbuf);
    if (cmdbuf.size()>0) {
      if (sizeof(cmd)==cmdbuf.size())
      {
        memcpy((void *)&cmd,cmdbuf.data(),sizeof(cmd));
        while (cmd.letter!=0) {
           sleep(0); // yield CPU
        }
      }
      else 
        printf("WARNING: Size mismatch, expected %d, got %d bytes\n",
          (int)sizeof(cmd), (int)cmdbuf.size());
    }
  }
}

// Receives beacon commands (on the beacon)
class aurora_beacon_command_server {
  zmq::context_t context;
  zmq::socket_t socket;
  aurora_beacon_command cmd;
public:

  aurora_beacon_command_server() 
    :context(1), socket(context,ZMQ_REP)
  {
    socket.bind("tcp://*:" aurora_beacon_command_port);
    cmd.letter=0;
    new std::thread(zmq_thread_poll_on_socket,std::ref(socket),std::ref(cmd));
  }
  
  // Front half of request: receive a command and return true,
  //  or else return false.
  // If this returns true, you MUST send a response, even if it's empty.
  bool request(aurora_beacon_command &c) {
    if (cmd.letter==0) return false; // nothing there yet
    if (cmd.letter=='b') return false; // still blocked on last request
    printf("Incoming beacon command %c (angle %d)\n",
       cmd.letter, (int)cmd.angle);
    c=cmd;
    cmd.letter='b'; // blocked on last request
    return true;
  }
  
  // Back half of request: send any response data
  //   (and let the caller continue)
  void response(const void *data=NULL,size_t ndata=0) {
    zmq::message_t responsebuf(ndata);
    memcpy(responsebuf.data(),data,ndata);
    socket.send(responsebuf);
    cmd.letter=0; // signal thread the socket is theirs again
  }
};

#endif



