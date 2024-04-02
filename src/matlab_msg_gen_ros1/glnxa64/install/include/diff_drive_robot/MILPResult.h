// Generated by gencpp from file diff_drive_robot/MILPResult.msg
// DO NOT EDIT!


#ifndef DIFF_DRIVE_ROBOT_MESSAGE_MILPRESULT_H
#define DIFF_DRIVE_ROBOT_MESSAGE_MILPRESULT_H


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
struct MILPResult_
{
  typedef MILPResult_<ContainerAllocator> Type;

  MILPResult_()
    : RobotID(0)
    , Humans()
    , GoingStart()
    , WaitingStart()
    , ServingStart()
    , DepotStart()
    , GoingFinish()
    , WaitingFinish()
    , ServingFinish()
    , DepotFinish()  {
    }
  MILPResult_(const ContainerAllocator& _alloc)
    : RobotID(0)
    , Humans(_alloc)
    , GoingStart(_alloc)
    , WaitingStart(_alloc)
    , ServingStart(_alloc)
    , DepotStart(_alloc)
    , GoingFinish(_alloc)
    , WaitingFinish(_alloc)
    , ServingFinish(_alloc)
    , DepotFinish(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _RobotID_type;
  _RobotID_type RobotID;

   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _Humans_type;
  _Humans_type Humans;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _GoingStart_type;
  _GoingStart_type GoingStart;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _WaitingStart_type;
  _WaitingStart_type WaitingStart;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _ServingStart_type;
  _ServingStart_type ServingStart;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _DepotStart_type;
  _DepotStart_type DepotStart;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _GoingFinish_type;
  _GoingFinish_type GoingFinish;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _WaitingFinish_type;
  _WaitingFinish_type WaitingFinish;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _ServingFinish_type;
  _ServingFinish_type ServingFinish;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _DepotFinish_type;
  _DepotFinish_type DepotFinish;





  typedef boost::shared_ptr< ::diff_drive_robot::MILPResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::diff_drive_robot::MILPResult_<ContainerAllocator> const> ConstPtr;

}; // struct MILPResult_

typedef ::diff_drive_robot::MILPResult_<std::allocator<void> > MILPResult;

typedef boost::shared_ptr< ::diff_drive_robot::MILPResult > MILPResultPtr;
typedef boost::shared_ptr< ::diff_drive_robot::MILPResult const> MILPResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::diff_drive_robot::MILPResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::diff_drive_robot::MILPResult_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::diff_drive_robot::MILPResult_<ContainerAllocator1> & lhs, const ::diff_drive_robot::MILPResult_<ContainerAllocator2> & rhs)
{
  return lhs.RobotID == rhs.RobotID &&
    lhs.Humans == rhs.Humans &&
    lhs.GoingStart == rhs.GoingStart &&
    lhs.WaitingStart == rhs.WaitingStart &&
    lhs.ServingStart == rhs.ServingStart &&
    lhs.DepotStart == rhs.DepotStart &&
    lhs.GoingFinish == rhs.GoingFinish &&
    lhs.WaitingFinish == rhs.WaitingFinish &&
    lhs.ServingFinish == rhs.ServingFinish &&
    lhs.DepotFinish == rhs.DepotFinish;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::diff_drive_robot::MILPResult_<ContainerAllocator1> & lhs, const ::diff_drive_robot::MILPResult_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace diff_drive_robot

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::diff_drive_robot::MILPResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::diff_drive_robot::MILPResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::diff_drive_robot::MILPResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::diff_drive_robot::MILPResult_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::diff_drive_robot::MILPResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::diff_drive_robot::MILPResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::diff_drive_robot::MILPResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2d08fbd0bb0c654ece09998565b41c04";
  }

  static const char* value(const ::diff_drive_robot::MILPResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2d08fbd0bb0c654eULL;
  static const uint64_t static_value2 = 0xce09998565b41c04ULL;
};

template<class ContainerAllocator>
struct DataType< ::diff_drive_robot::MILPResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "diff_drive_robot/MILPResult";
  }

  static const char* value(const ::diff_drive_robot::MILPResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::diff_drive_robot::MILPResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 RobotID\n"
"int32[] Humans\n"
"float64[] GoingStart\n"
"float64[] WaitingStart \n"
"float64[] ServingStart\n"
"float64[] DepotStart\n"
"float64[] GoingFinish\n"
"float64[] WaitingFinish \n"
"float64[] ServingFinish\n"
"float64[] DepotFinish\n"
;
  }

  static const char* value(const ::diff_drive_robot::MILPResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::diff_drive_robot::MILPResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.RobotID);
      stream.next(m.Humans);
      stream.next(m.GoingStart);
      stream.next(m.WaitingStart);
      stream.next(m.ServingStart);
      stream.next(m.DepotStart);
      stream.next(m.GoingFinish);
      stream.next(m.WaitingFinish);
      stream.next(m.ServingFinish);
      stream.next(m.DepotFinish);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MILPResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::diff_drive_robot::MILPResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::diff_drive_robot::MILPResult_<ContainerAllocator>& v)
  {
    s << indent << "RobotID: ";
    Printer<int32_t>::stream(s, indent + "  ", v.RobotID);
    s << indent << "Humans[]" << std::endl;
    for (size_t i = 0; i < v.Humans.size(); ++i)
    {
      s << indent << "  Humans[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.Humans[i]);
    }
    s << indent << "GoingStart[]" << std::endl;
    for (size_t i = 0; i < v.GoingStart.size(); ++i)
    {
      s << indent << "  GoingStart[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.GoingStart[i]);
    }
    s << indent << "WaitingStart[]" << std::endl;
    for (size_t i = 0; i < v.WaitingStart.size(); ++i)
    {
      s << indent << "  WaitingStart[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.WaitingStart[i]);
    }
    s << indent << "ServingStart[]" << std::endl;
    for (size_t i = 0; i < v.ServingStart.size(); ++i)
    {
      s << indent << "  ServingStart[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.ServingStart[i]);
    }
    s << indent << "DepotStart[]" << std::endl;
    for (size_t i = 0; i < v.DepotStart.size(); ++i)
    {
      s << indent << "  DepotStart[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.DepotStart[i]);
    }
    s << indent << "GoingFinish[]" << std::endl;
    for (size_t i = 0; i < v.GoingFinish.size(); ++i)
    {
      s << indent << "  GoingFinish[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.GoingFinish[i]);
    }
    s << indent << "WaitingFinish[]" << std::endl;
    for (size_t i = 0; i < v.WaitingFinish.size(); ++i)
    {
      s << indent << "  WaitingFinish[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.WaitingFinish[i]);
    }
    s << indent << "ServingFinish[]" << std::endl;
    for (size_t i = 0; i < v.ServingFinish.size(); ++i)
    {
      s << indent << "  ServingFinish[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.ServingFinish[i]);
    }
    s << indent << "DepotFinish[]" << std::endl;
    for (size_t i = 0; i < v.DepotFinish.size(); ++i)
    {
      s << indent << "  DepotFinish[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.DepotFinish[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // DIFF_DRIVE_ROBOT_MESSAGE_MILPRESULT_H
