// Generated by gencpp from file franka_control/ErrorRecoveryFeedback.msg
// DO NOT EDIT!


#ifndef FRANKA_CONTROL_MESSAGE_ERRORRECOVERYFEEDBACK_H
#define FRANKA_CONTROL_MESSAGE_ERRORRECOVERYFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace franka_control
{
template <class ContainerAllocator>
struct ErrorRecoveryFeedback_
{
  typedef ErrorRecoveryFeedback_<ContainerAllocator> Type;

  ErrorRecoveryFeedback_()
    {
    }
  ErrorRecoveryFeedback_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::franka_control::ErrorRecoveryFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::franka_control::ErrorRecoveryFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct ErrorRecoveryFeedback_

typedef ::franka_control::ErrorRecoveryFeedback_<std::allocator<void> > ErrorRecoveryFeedback;

typedef boost::shared_ptr< ::franka_control::ErrorRecoveryFeedback > ErrorRecoveryFeedbackPtr;
typedef boost::shared_ptr< ::franka_control::ErrorRecoveryFeedback const> ErrorRecoveryFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::franka_control::ErrorRecoveryFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::franka_control::ErrorRecoveryFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace franka_control

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'franka_control': ['/home/crslab/catkin_main/devel/share/franka_control/msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::franka_control::ErrorRecoveryFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::franka_control::ErrorRecoveryFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::franka_control::ErrorRecoveryFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::franka_control::ErrorRecoveryFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::franka_control::ErrorRecoveryFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::franka_control::ErrorRecoveryFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::franka_control::ErrorRecoveryFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::franka_control::ErrorRecoveryFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::franka_control::ErrorRecoveryFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "franka_control/ErrorRecoveryFeedback";
  }

  static const char* value(const ::franka_control::ErrorRecoveryFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::franka_control::ErrorRecoveryFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
";
  }

  static const char* value(const ::franka_control::ErrorRecoveryFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::franka_control::ErrorRecoveryFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ErrorRecoveryFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::franka_control::ErrorRecoveryFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::franka_control::ErrorRecoveryFeedback_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // FRANKA_CONTROL_MESSAGE_ERRORRECOVERYFEEDBACK_H
