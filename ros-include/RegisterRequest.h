// Generated by gencpp from file smarthome/RegisterRequest.msg
// DO NOT EDIT!


#ifndef SMARTHOME_MESSAGE_REGISTERREQUEST_H
#define SMARTHOME_MESSAGE_REGISTERREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <smarthome/Device.h>

namespace smarthome
{
template <class ContainerAllocator>
struct RegisterRequest_
{
  typedef RegisterRequest_<ContainerAllocator> Type;

  RegisterRequest_()
    : device()  {
    }
  RegisterRequest_(const ContainerAllocator& _alloc)
    : device(_alloc)  {
  (void)_alloc;
    }



   typedef  ::smarthome::Device_<ContainerAllocator>  _device_type;
  _device_type device;





  typedef boost::shared_ptr< ::smarthome::RegisterRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::smarthome::RegisterRequest_<ContainerAllocator> const> ConstPtr;

}; // struct RegisterRequest_

typedef ::smarthome::RegisterRequest_<std::allocator<void> > RegisterRequest;

typedef boost::shared_ptr< ::smarthome::RegisterRequest > RegisterRequestPtr;
typedef boost::shared_ptr< ::smarthome::RegisterRequest const> RegisterRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::smarthome::RegisterRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::smarthome::RegisterRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::smarthome::RegisterRequest_<ContainerAllocator1> & lhs, const ::smarthome::RegisterRequest_<ContainerAllocator2> & rhs)
{
  return lhs.device == rhs.device;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::smarthome::RegisterRequest_<ContainerAllocator1> & lhs, const ::smarthome::RegisterRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace smarthome

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::smarthome::RegisterRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::smarthome::RegisterRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::smarthome::RegisterRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::smarthome::RegisterRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::smarthome::RegisterRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::smarthome::RegisterRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::smarthome::RegisterRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a32d225bd19c7358fc08ec887dcdf6b9";
  }

  static const char* value(const ::smarthome::RegisterRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa32d225bd19c7358ULL;
  static const uint64_t static_value2 = 0xfc08ec887dcdf6b9ULL;
};

template<class ContainerAllocator>
struct DataType< ::smarthome::RegisterRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "smarthome/RegisterRequest";
  }

  static const char* value(const ::smarthome::RegisterRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::smarthome::RegisterRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Device device\n"
"\n"
"================================================================================\n"
"MSG: smarthome/Device\n"
"# See https://developers.google.com/assistant/smarthome/reference/intent/sync\n"
"#   for details of all fields\n"
"\n"
"# Device ID and capabilities\n"
"string id\n"
"string type\n"
"string[] traits\n"
"\n"
"# Device names\n"
"string name\n"
"string[] nicknames\n"
"string[] default_names\n"
"\n"
"# Status reporting info\n"
"bool will_report_state\n"
"string room_hint\n"
"\n"
"# Make and manufacture\n"
"string manufacturer\n"
"string model\n"
"string sw_version\n"
"string hw_version\n"
"\n"
"# Alternate, local device ID\n"
"string device_id\n"
"\n"
;
  }

  static const char* value(const ::smarthome::RegisterRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::smarthome::RegisterRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.device);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RegisterRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::smarthome::RegisterRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::smarthome::RegisterRequest_<ContainerAllocator>& v)
  {
    s << indent << "device: ";
    s << std::endl;
    Printer< ::smarthome::Device_<ContainerAllocator> >::stream(s, indent + "  ", v.device);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SMARTHOME_MESSAGE_REGISTERREQUEST_H
