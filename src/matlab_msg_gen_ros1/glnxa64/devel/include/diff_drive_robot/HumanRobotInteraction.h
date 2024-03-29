// Generated by gencpp from file diff_drive_robot/HumanRobotInteraction.msg
// DO NOT EDIT!


#ifndef DIFF_DRIVE_ROBOT_MESSAGE_HUMANROBOTINTERACTION_H
#define DIFF_DRIVE_ROBOT_MESSAGE_HUMANROBOTINTERACTION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace diff_drive_robot
{
template <class ContainerAllocator>
struct HumanRobotInteraction_
{
  typedef HumanRobotInteraction_<ContainerAllocator> Type;

  HumanRobotInteraction_()
    : HumanID(0)
    , RobotVelocity()
    , WaitingTime()
    , StartFilling()
    , FinishFilling()
    , StartServing()
    , FinishServing()
    , Confirm(0)  {
    }
  HumanRobotInteraction_(const ContainerAllocator& _alloc)
    : HumanID(0)
    , RobotVelocity(_alloc)
    , WaitingTime(_alloc)
    , StartFilling(_alloc)
    , FinishFilling(_alloc)
    , StartServing(_alloc)
    , FinishServing(_alloc)
    , Confirm(0)  {
  (void)_alloc;
    }



   typedef int32_t _HumanID_type;
  _HumanID_type HumanID;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _RobotVelocity_type;
  _RobotVelocity_type RobotVelocity;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _WaitingTime_type;
  _WaitingTime_type WaitingTime;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _StartFilling_type;
  _StartFilling_type StartFilling;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _FinishFilling_type;
  _FinishFilling_type FinishFilling;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _StartServing_type;
  _StartServing_type StartServing;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _FinishServing_type;
  _FinishServing_type FinishServing;

   typedef int32_t _Confirm_type;
  _Confirm_type Confirm;





  typedef boost::shared_ptr< ::diff_drive_robot::HumanRobotInteraction_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::diff_drive_robot::HumanRobotInteraction_<ContainerAllocator> const> ConstPtr;

}; // struct HumanRobotInteraction_

typedef ::diff_drive_robot::HumanRobotInteraction_<std::allocator<void> > HumanRobotInteraction;

typedef boost::shared_ptr< ::diff_drive_robot::HumanRobotInteraction > HumanRobotInteractionPtr;
typedef boost::shared_ptr< ::diff_drive_robot::HumanRobotInteraction const> HumanRobotInteractionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::diff_drive_robot::HumanRobotInteraction_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::diff_drive_robot::HumanRobotInteraction_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::diff_drive_robot::HumanRobotInteraction_<ContainerAllocator1> & lhs, const ::diff_drive_robot::HumanRobotInteraction_<ContainerAllocator2> & rhs)
{
  return lhs.HumanID == rhs.HumanID &&
    lhs.RobotVelocity == rhs.RobotVelocity &&
    lhs.WaitingTime == rhs.WaitingTime &&
    lhs.StartFilling == rhs.StartFilling &&
    lhs.FinishFilling == rhs.FinishFilling &&
    lhs.StartServing == rhs.StartServing &&
    lhs.FinishServing == rhs.FinishServing &&
    lhs.Confirm == rhs.Confirm;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::diff_drive_robot::HumanRobotInteraction_<ContainerAllocator1> & lhs, const ::diff_drive_robot::HumanRobotInteraction_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace diff_drive_robot

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::diff_drive_robot::HumanRobotInteraction_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::diff_drive_robot::HumanRobotInteraction_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::diff_drive_robot::HumanRobotInteraction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::diff_drive_robot::HumanRobotInteraction_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::diff_drive_robot::HumanRobotInteraction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::diff_drive_robot::HumanRobotInteraction_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::diff_drive_robot::HumanRobotInteraction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a3487fb54248fa58bf6d4fa71ad8c145";
  }

  static const char* value(const ::diff_drive_robot::HumanRobotInteraction_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa3487fb54248fa58ULL;
  static const uint64_t static_value2 = 0xbf6d4fa71ad8c145ULL;
};

template<class ContainerAllocator>
struct DataType< ::diff_drive_robot::HumanRobotInteraction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "diff_drive_robot/HumanRobotInteraction";
  }

  static const char* value(const ::diff_drive_robot::HumanRobotInteraction_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::diff_drive_robot::HumanRobotInteraction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 HumanID\n"
"float64[] RobotVelocity\n"
"float64[] WaitingTime\n"
"float64[] StartFilling\n"
"float64[] FinishFilling\n"
"float64[] StartServing\n"
"float64[] FinishServing\n"
"int32 Confirm\n"
;
  }

  static const char* value(const ::diff_drive_robot::HumanRobotInteraction_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::diff_drive_robot::HumanRobotInteraction_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.HumanID);
      stream.next(m.RobotVelocity);
      stream.next(m.WaitingTime);
      stream.next(m.StartFilling);
      stream.next(m.FinishFilling);
      stream.next(m.StartServing);
      stream.next(m.FinishServing);
      stream.next(m.Confirm);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct HumanRobotInteraction_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::diff_drive_robot::HumanRobotInteraction_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::diff_drive_robot::HumanRobotInteraction_<ContainerAllocator>& v)
  {
    s << indent << "HumanID: ";
    Printer<int32_t>::stream(s, indent + "  ", v.HumanID);
    s << indent << "RobotVelocity[]" << std::endl;
    for (size_t i = 0; i < v.RobotVelocity.size(); ++i)
    {
      s << indent << "  RobotVelocity[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.RobotVelocity[i]);
    }
    s << indent << "WaitingTime[]" << std::endl;
    for (size_t i = 0; i < v.WaitingTime.size(); ++i)
    {
      s << indent << "  WaitingTime[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.WaitingTime[i]);
    }
    s << indent << "StartFilling[]" << std::endl;
    for (size_t i = 0; i < v.StartFilling.size(); ++i)
    {
      s << indent << "  StartFilling[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.StartFilling[i]);
    }
    s << indent << "FinishFilling[]" << std::endl;
    for (size_t i = 0; i < v.FinishFilling.size(); ++i)
    {
      s << indent << "  FinishFilling[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.FinishFilling[i]);
    }
    s << indent << "StartServing[]" << std::endl;
    for (size_t i = 0; i < v.StartServing.size(); ++i)
    {
      s << indent << "  StartServing[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.StartServing[i]);
    }
    s << indent << "FinishServing[]" << std::endl;
    for (size_t i = 0; i < v.FinishServing.size(); ++i)
    {
      s << indent << "  FinishServing[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.FinishServing[i]);
    }
    s << indent << "Confirm: ";
    Printer<int32_t>::stream(s, indent + "  ", v.Confirm);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DIFF_DRIVE_ROBOT_MESSAGE_HUMANROBOTINTERACTION_H
