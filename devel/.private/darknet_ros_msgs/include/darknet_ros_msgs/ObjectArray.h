// Generated by gencpp from file darknet_ros_msgs/ObjectArray.msg
// DO NOT EDIT!


#ifndef DARKNET_ROS_MSGS_MESSAGE_OBJECTARRAY_H
#define DARKNET_ROS_MSGS_MESSAGE_OBJECTARRAY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <darknet_ros_msgs/ObjectPoint.h>

namespace darknet_ros_msgs
{
template <class ContainerAllocator>
struct ObjectArray_
{
  typedef ObjectArray_<ContainerAllocator> Type;

  ObjectArray_()
    : objects()  {
    }
  ObjectArray_(const ContainerAllocator& _alloc)
    : objects(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::darknet_ros_msgs::ObjectPoint_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::darknet_ros_msgs::ObjectPoint_<ContainerAllocator> >::other >  _objects_type;
  _objects_type objects;





  typedef boost::shared_ptr< ::darknet_ros_msgs::ObjectArray_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::darknet_ros_msgs::ObjectArray_<ContainerAllocator> const> ConstPtr;

}; // struct ObjectArray_

typedef ::darknet_ros_msgs::ObjectArray_<std::allocator<void> > ObjectArray;

typedef boost::shared_ptr< ::darknet_ros_msgs::ObjectArray > ObjectArrayPtr;
typedef boost::shared_ptr< ::darknet_ros_msgs::ObjectArray const> ObjectArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::darknet_ros_msgs::ObjectArray_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::darknet_ros_msgs::ObjectArray_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::darknet_ros_msgs::ObjectArray_<ContainerAllocator1> & lhs, const ::darknet_ros_msgs::ObjectArray_<ContainerAllocator2> & rhs)
{
  return lhs.objects == rhs.objects;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::darknet_ros_msgs::ObjectArray_<ContainerAllocator1> & lhs, const ::darknet_ros_msgs::ObjectArray_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace darknet_ros_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::darknet_ros_msgs::ObjectArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::darknet_ros_msgs::ObjectArray_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::darknet_ros_msgs::ObjectArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::darknet_ros_msgs::ObjectArray_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::darknet_ros_msgs::ObjectArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::darknet_ros_msgs::ObjectArray_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::darknet_ros_msgs::ObjectArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7193c987d88114e069280a4723fa5977";
  }

  static const char* value(const ::darknet_ros_msgs::ObjectArray_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7193c987d88114e0ULL;
  static const uint64_t static_value2 = 0x69280a4723fa5977ULL;
};

template<class ContainerAllocator>
struct DataType< ::darknet_ros_msgs::ObjectArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "darknet_ros_msgs/ObjectArray";
  }

  static const char* value(const ::darknet_ros_msgs::ObjectArray_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::darknet_ros_msgs::ObjectArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "darknet_ros_msgs/ObjectPoint[] objects\n"
"\n"
"================================================================================\n"
"MSG: darknet_ros_msgs/ObjectPoint\n"
"string Class\n"
"float64 probability\n"
"int8 width\n"
"int8 height\n"
;
  }

  static const char* value(const ::darknet_ros_msgs::ObjectArray_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::darknet_ros_msgs::ObjectArray_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.objects);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ObjectArray_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::darknet_ros_msgs::ObjectArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::darknet_ros_msgs::ObjectArray_<ContainerAllocator>& v)
  {
    s << indent << "objects[]" << std::endl;
    for (size_t i = 0; i < v.objects.size(); ++i)
    {
      s << indent << "  objects[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::darknet_ros_msgs::ObjectPoint_<ContainerAllocator> >::stream(s, indent + "    ", v.objects[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // DARKNET_ROS_MSGS_MESSAGE_OBJECTARRAY_H