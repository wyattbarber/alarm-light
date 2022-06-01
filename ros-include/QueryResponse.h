// Generated by gencpp from file smarthome/QueryResponse.msg
// DO NOT EDIT!


#ifndef SMARTHOME_MESSAGE_QUERYRESPONSE_H
#define SMARTHOME_MESSAGE_QUERYRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace smarthome
{
template <class ContainerAllocator>
struct QueryResponse_
{
  typedef QueryResponse_<ContainerAllocator> Type;

  QueryResponse_()
    : param_names()
    , param_values()
    , error_code()  {
    }
  QueryResponse_(const ContainerAllocator& _alloc)
    : param_names(_alloc)
    , param_values(_alloc)
    , error_code(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _param_names_type;
  _param_names_type param_names;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _param_values_type;
  _param_values_type param_values;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _error_code_type;
  _error_code_type error_code;





  typedef boost::shared_ptr< ::smarthome::QueryResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::smarthome::QueryResponse_<ContainerAllocator> const> ConstPtr;

}; // struct QueryResponse_

typedef ::smarthome::QueryResponse_<std::allocator<void> > QueryResponse;

typedef boost::shared_ptr< ::smarthome::QueryResponse > QueryResponsePtr;
typedef boost::shared_ptr< ::smarthome::QueryResponse const> QueryResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::smarthome::QueryResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::smarthome::QueryResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::smarthome::QueryResponse_<ContainerAllocator1> & lhs, const ::smarthome::QueryResponse_<ContainerAllocator2> & rhs)
{
  return lhs.param_names == rhs.param_names &&
    lhs.param_values == rhs.param_values &&
    lhs.error_code == rhs.error_code;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::smarthome::QueryResponse_<ContainerAllocator1> & lhs, const ::smarthome::QueryResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace smarthome

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::smarthome::QueryResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::smarthome::QueryResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::smarthome::QueryResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::smarthome::QueryResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::smarthome::QueryResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::smarthome::QueryResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::smarthome::QueryResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "273866a0060dc9b65fcdaa39ff337d53";
  }

  static const char* value(const ::smarthome::QueryResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x273866a0060dc9b6ULL;
  static const uint64_t static_value2 = 0x5fcdaa39ff337d53ULL;
};

template<class ContainerAllocator>
struct DataType< ::smarthome::QueryResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "smarthome/QueryResponse";
  }

  static const char* value(const ::smarthome::QueryResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::smarthome::QueryResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string[] param_names\n"
"string[] param_values\n"
"string error_code\n"
;
  }

  static const char* value(const ::smarthome::QueryResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::smarthome::QueryResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.param_names);
      stream.next(m.param_values);
      stream.next(m.error_code);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct QueryResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::smarthome::QueryResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::smarthome::QueryResponse_<ContainerAllocator>& v)
  {
    s << indent << "param_names[]" << std::endl;
    for (size_t i = 0; i < v.param_names.size(); ++i)
    {
      s << indent << "  param_names[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.param_names[i]);
    }
    s << indent << "param_values[]" << std::endl;
    for (size_t i = 0; i < v.param_values.size(); ++i)
    {
      s << indent << "  param_values[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.param_values[i]);
    }
    s << indent << "error_code: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.error_code);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SMARTHOME_MESSAGE_QUERYRESPONSE_H
