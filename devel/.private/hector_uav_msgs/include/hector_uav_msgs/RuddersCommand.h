// Generated by gencpp from file hector_uav_msgs/RuddersCommand.msg
// DO NOT EDIT!


#ifndef HECTOR_UAV_MSGS_MESSAGE_RUDDERSCOMMAND_H
#define HECTOR_UAV_MSGS_MESSAGE_RUDDERSCOMMAND_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace hector_uav_msgs
{
template <class ContainerAllocator>
struct RuddersCommand_
{
  typedef RuddersCommand_<ContainerAllocator> Type;

  RuddersCommand_()
    : header()
    , aileron(0.0)
    , elevator(0.0)
    , rudder(0.0)  {
    }
  RuddersCommand_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , aileron(0.0)
    , elevator(0.0)
    , rudder(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _aileron_type;
  _aileron_type aileron;

   typedef float _elevator_type;
  _elevator_type elevator;

   typedef float _rudder_type;
  _rudder_type rudder;





  typedef boost::shared_ptr< ::hector_uav_msgs::RuddersCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hector_uav_msgs::RuddersCommand_<ContainerAllocator> const> ConstPtr;

}; // struct RuddersCommand_

typedef ::hector_uav_msgs::RuddersCommand_<std::allocator<void> > RuddersCommand;

typedef boost::shared_ptr< ::hector_uav_msgs::RuddersCommand > RuddersCommandPtr;
typedef boost::shared_ptr< ::hector_uav_msgs::RuddersCommand const> RuddersCommandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hector_uav_msgs::RuddersCommand_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hector_uav_msgs::RuddersCommand_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hector_uav_msgs::RuddersCommand_<ContainerAllocator1> & lhs, const ::hector_uav_msgs::RuddersCommand_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.aileron == rhs.aileron &&
    lhs.elevator == rhs.elevator &&
    lhs.rudder == rhs.rudder;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hector_uav_msgs::RuddersCommand_<ContainerAllocator1> & lhs, const ::hector_uav_msgs::RuddersCommand_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hector_uav_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::hector_uav_msgs::RuddersCommand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hector_uav_msgs::RuddersCommand_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hector_uav_msgs::RuddersCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hector_uav_msgs::RuddersCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hector_uav_msgs::RuddersCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hector_uav_msgs::RuddersCommand_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hector_uav_msgs::RuddersCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2e136cb8cfffc2233e404b320c27bca6";
  }

  static const char* value(const ::hector_uav_msgs::RuddersCommand_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2e136cb8cfffc223ULL;
  static const uint64_t static_value2 = 0x3e404b320c27bca6ULL;
};

template<class ContainerAllocator>
struct DataType< ::hector_uav_msgs::RuddersCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hector_uav_msgs/RuddersCommand";
  }

  static const char* value(const ::hector_uav_msgs::RuddersCommand_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hector_uav_msgs::RuddersCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"float32 aileron\n"
"float32 elevator\n"
"float32 rudder\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::hector_uav_msgs::RuddersCommand_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hector_uav_msgs::RuddersCommand_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.aileron);
      stream.next(m.elevator);
      stream.next(m.rudder);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RuddersCommand_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hector_uav_msgs::RuddersCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hector_uav_msgs::RuddersCommand_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "aileron: ";
    Printer<float>::stream(s, indent + "  ", v.aileron);
    s << indent << "elevator: ";
    Printer<float>::stream(s, indent + "  ", v.elevator);
    s << indent << "rudder: ";
    Printer<float>::stream(s, indent + "  ", v.rudder);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HECTOR_UAV_MSGS_MESSAGE_RUDDERSCOMMAND_H