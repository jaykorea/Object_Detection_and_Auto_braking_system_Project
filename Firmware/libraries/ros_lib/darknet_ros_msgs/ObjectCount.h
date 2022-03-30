#ifndef _ROS_darknet_ros_msgs_ObjectCount_h
#define _ROS_darknet_ros_msgs_ObjectCount_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace darknet_ros_msgs
{

  class ObjectCount : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int8_t _count_type;
      _count_type count;

    ObjectCount():
      header(),
      count(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_count;
      u_count.real = this->count;
      *(outbuffer + offset + 0) = (u_count.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->count);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_count;
      u_count.base = 0;
      u_count.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->count = u_count.real;
      offset += sizeof(this->count);
     return offset;
    }

    const char * getType(){ return "darknet_ros_msgs/ObjectCount"; };
    const char * getMD5(){ return "123675bf4420958597b5a78cb3800a1f"; };

  };

}
#endif
