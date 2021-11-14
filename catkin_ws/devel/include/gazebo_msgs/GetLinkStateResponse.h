// Generated by gencpp from file gazebo_msgs/GetLinkStateResponse.msg
// DO NOT EDIT!


#ifndef GAZEBO_MSGS_MESSAGE_GETLINKSTATERESPONSE_H
#define GAZEBO_MSGS_MESSAGE_GETLINKSTATERESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <gazebo_msgs/LinkState.h>

namespace gazebo_msgs
{
template <class ContainerAllocator>
struct GetLinkStateResponse_
{
  typedef GetLinkStateResponse_<ContainerAllocator> Type;

  GetLinkStateResponse_()
    : link_state()
    , success(false)
    , status_message()  {
    }
  GetLinkStateResponse_(const ContainerAllocator& _alloc)
    : link_state(_alloc)
    , success(false)
    , status_message(_alloc)  {
  (void)_alloc;
    }



   typedef  ::gazebo_msgs::LinkState_<ContainerAllocator>  _link_state_type;
  _link_state_type link_state;

   typedef uint8_t _success_type;
  _success_type success;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _status_message_type;
  _status_message_type status_message;





  typedef boost::shared_ptr< ::gazebo_msgs::GetLinkStateResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::gazebo_msgs::GetLinkStateResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetLinkStateResponse_

typedef ::gazebo_msgs::GetLinkStateResponse_<std::allocator<void> > GetLinkStateResponse;

typedef boost::shared_ptr< ::gazebo_msgs::GetLinkStateResponse > GetLinkStateResponsePtr;
typedef boost::shared_ptr< ::gazebo_msgs::GetLinkStateResponse const> GetLinkStateResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::gazebo_msgs::GetLinkStateResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::gazebo_msgs::GetLinkStateResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace gazebo_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'trajectory_msgs': ['/opt/ros/kinetic/share/trajectory_msgs/cmake/../msg'], 'gazebo_msgs': ['/home/ur3/catkin_ws/src/lab2andDriver/drivers/gazebo_ros_pkgs/gazebo_msgs/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::gazebo_msgs::GetLinkStateResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::gazebo_msgs::GetLinkStateResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::gazebo_msgs::GetLinkStateResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::gazebo_msgs::GetLinkStateResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gazebo_msgs::GetLinkStateResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gazebo_msgs::GetLinkStateResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::gazebo_msgs::GetLinkStateResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8ba55ad34f9c072e75c0de57b089753b";
  }

  static const char* value(const ::gazebo_msgs::GetLinkStateResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8ba55ad34f9c072eULL;
  static const uint64_t static_value2 = 0x75c0de57b089753bULL;
};

template<class ContainerAllocator>
struct DataType< ::gazebo_msgs::GetLinkStateResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "gazebo_msgs/GetLinkStateResponse";
  }

  static const char* value(const ::gazebo_msgs::GetLinkStateResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::gazebo_msgs::GetLinkStateResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "gazebo_msgs/LinkState link_state\n\
bool success\n\
string status_message\n\
\n\
\n\
================================================================================\n\
MSG: gazebo_msgs/LinkState\n\
# @todo: FIXME: sets pose and twist of a link.  All children link poses/twists of the URDF tree are not updated accordingly, but should be.\n\
string link_name            # link name, link_names are in gazebo scoped name notation, [model_name::body_name]\n\
geometry_msgs/Pose pose     # desired pose in reference frame\n\
geometry_msgs/Twist twist   # desired twist in reference frame\n\
string reference_frame      # set pose/twist relative to the frame of this link/body\n\
                            # leave empty or \"world\" or \"map\" defaults to world-frame\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of position and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Twist\n\
# This expresses velocity in free space broken into its linear and angular parts.\n\
Vector3  linear\n\
Vector3  angular\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
# It is only meant to represent a direction. Therefore, it does not\n\
# make sense to apply a translation to it (e.g., when applying a \n\
# generic rigid transformation to a Vector3, tf2 will only apply the\n\
# rotation). If you want your data to be translatable too, use the\n\
# geometry_msgs/Point message instead.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::gazebo_msgs::GetLinkStateResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::gazebo_msgs::GetLinkStateResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.link_state);
      stream.next(m.success);
      stream.next(m.status_message);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetLinkStateResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::gazebo_msgs::GetLinkStateResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::gazebo_msgs::GetLinkStateResponse_<ContainerAllocator>& v)
  {
    s << indent << "link_state: ";
    s << std::endl;
    Printer< ::gazebo_msgs::LinkState_<ContainerAllocator> >::stream(s, indent + "  ", v.link_state);
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
    s << indent << "status_message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.status_message);
  }
};

} // namespace message_operations
} // namespace ros

#endif // GAZEBO_MSGS_MESSAGE_GETLINKSTATERESPONSE_H
