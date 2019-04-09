/*
  Communicates pose data across the network.
*/
#ifndef __AURORA_pose_NETWORK_H
#define __AURORA_pose_NETWORK_H

#include "pose.h"
#include <zmq.hpp>

#define ZMQ_POSE_PORT "10010"

/**
  Publishes pose data
*/
class pose_publisher {
public:
  zmq::context_t context;
  zmq::socket_t publisher;

  pose_publisher() 
    :context(1),
     publisher(context, ZMQ_PUB)
  {
    publisher.bind("tcp://*:" ZMQ_POSE_PORT);
  }
  
  template<class all_markers>
  void publish(const all_markers &m) {
    zmq::message_t message(sizeof(m));
    memcpy(message.data(),&m,sizeof(m));
    publisher.send(message);
  }
};

/**
 Subscribes to pose data updates
*/
class pose_subscriber {
public:
  zmq::context_t context;
  zmq::socket_t subscriber;
  
  pose_subscriber(const char *serverID) 
    :context(1),
     subscriber(context,ZMQ_SUB)
  {
    subscriber.connect(serverID);
    subscriber.setsockopt(ZMQ_SUBSCRIBE,"",0);
  }
  
  template<class all_markers>
  bool update(all_markers &m) {
    zmq::message_t msg;
    try {
      subscriber.recv(&msg,ZMQ_NOBLOCK);
    } catch (...) { // Maybe: (zmq::error_t &e) {
      return false;
    }
    if (msg.size()<=0) return false;
    if (msg.size()!=sizeof(m)) {
      fprintf(stderr,"Size mismatch in marker data: expected %d, got %d!\n",
        (int)sizeof(m), (int)msg.size());
    }
    memcpy(&m,msg.data(),sizeof(m));
    return true;
  }
};




#endif

