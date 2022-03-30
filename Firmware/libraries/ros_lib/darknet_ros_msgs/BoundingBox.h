#ifndef _ROS_darknet_ros_msgs_BoundingBox_h
#define _ROS_darknet_ros_msgs_BoundingBox_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace darknet_ros_msgs
{

  class BoundingBox : public ros::Msg
  {
    public:
      typedef float _probability_type;
      _probability_type probability;
      typedef int64_t _xmin_type;
      _xmin_type xmin;
      typedef int64_t _ymin_type;
      _ymin_type ymin;
      typedef int64_t _xmax_type;
      _xmax_type xmax;
      typedef int64_t _ymax_type;
      _ymax_type ymax;
      typedef int16_t _id_type;
      _id_type id;
      typedef const char* _Class_type;
      _Class_type Class;

    BoundingBox():
      probability(0),
      xmin(0),
      ymin(0),
      xmax(0),
      ymax(0),
      id(0),
      Class("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->probability);
      union {
        int64_t real;
        uint64_t base;
      } u_xmin;
      u_xmin.real = this->xmin;
      *(outbuffer + offset + 0) = (u_xmin.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xmin.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xmin.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xmin.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_xmin.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_xmin.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_xmin.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_xmin.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->xmin);
      union {
        int64_t real;
        uint64_t base;
      } u_ymin;
      u_ymin.real = this->ymin;
      *(outbuffer + offset + 0) = (u_ymin.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ymin.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ymin.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ymin.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_ymin.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_ymin.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_ymin.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_ymin.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->ymin);
      union {
        int64_t real;
        uint64_t base;
      } u_xmax;
      u_xmax.real = this->xmax;
      *(outbuffer + offset + 0) = (u_xmax.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xmax.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xmax.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xmax.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_xmax.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_xmax.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_xmax.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_xmax.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->xmax);
      union {
        int64_t real;
        uint64_t base;
      } u_ymax;
      u_ymax.real = this->ymax;
      *(outbuffer + offset + 0) = (u_ymax.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ymax.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ymax.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ymax.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_ymax.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_ymax.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_ymax.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_ymax.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->ymax);
      union {
        int16_t real;
        uint16_t base;
      } u_id;
      u_id.real = this->id;
      *(outbuffer + offset + 0) = (u_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_id.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->id);
      uint32_t length_Class = strlen(this->Class);
      varToArr(outbuffer + offset, length_Class);
      offset += 4;
      memcpy(outbuffer + offset, this->Class, length_Class);
      offset += length_Class;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->probability));
      union {
        int64_t real;
        uint64_t base;
      } u_xmin;
      u_xmin.base = 0;
      u_xmin.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_xmin.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_xmin.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_xmin.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_xmin.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_xmin.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_xmin.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_xmin.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->xmin = u_xmin.real;
      offset += sizeof(this->xmin);
      union {
        int64_t real;
        uint64_t base;
      } u_ymin;
      u_ymin.base = 0;
      u_ymin.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ymin.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ymin.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ymin.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_ymin.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_ymin.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_ymin.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_ymin.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->ymin = u_ymin.real;
      offset += sizeof(this->ymin);
      union {
        int64_t real;
        uint64_t base;
      } u_xmax;
      u_xmax.base = 0;
      u_xmax.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_xmax.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_xmax.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_xmax.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_xmax.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_xmax.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_xmax.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_xmax.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->xmax = u_xmax.real;
      offset += sizeof(this->xmax);
      union {
        int64_t real;
        uint64_t base;
      } u_ymax;
      u_ymax.base = 0;
      u_ymax.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ymax.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ymax.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ymax.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_ymax.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_ymax.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_ymax.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_ymax.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->ymax = u_ymax.real;
      offset += sizeof(this->ymax);
      union {
        int16_t real;
        uint16_t base;
      } u_id;
      u_id.base = 0;
      u_id.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_id.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->id = u_id.real;
      offset += sizeof(this->id);
      uint32_t length_Class;
      arrToVar(length_Class, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_Class; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_Class-1]=0;
      this->Class = (char *)(inbuffer + offset-1);
      offset += length_Class;
     return offset;
    }

    const char * getType(){ return "darknet_ros_msgs/BoundingBox"; };
    const char * getMD5(){ return "6b8f03f076d75c692c40b501a56253e0"; };

  };

}
#endif
