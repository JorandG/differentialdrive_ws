// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for diff_drive_robot/HumanRobotInteraction
#include "boost/date_time.hpp"
#include "boost/shared_array.hpp"
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4244)
#pragma warning(disable : 4265)
#pragma warning(disable : 4458)
#pragma warning(disable : 4100)
#pragma warning(disable : 4127)
#pragma warning(disable : 4267)
#pragma warning(disable : 4068)
#pragma warning(disable : 4245)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif //_MSC_VER
#include "ros/ros.h"
#include "diff_drive_robot/HumanRobotInteraction.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class DIFF_DRIVE_ROBOT_EXPORT diff_drive_robot_msg_HumanRobotInteraction_common : public MATLABROSMsgInterface<diff_drive_robot::HumanRobotInteraction> {
  public:
    virtual ~diff_drive_robot_msg_HumanRobotInteraction_common(){}
    virtual void copy_from_struct(diff_drive_robot::HumanRobotInteraction* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const diff_drive_robot::HumanRobotInteraction* msg, MultiLibLoader loader, size_t size = 1);
};
  void diff_drive_robot_msg_HumanRobotInteraction_common::copy_from_struct(diff_drive_robot::HumanRobotInteraction* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //HumanID
        const matlab::data::TypedArray<int32_t> HumanID_arr = arr["HumanID"];
        msg->HumanID = HumanID_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'HumanID' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'HumanID' is wrong type; expected a int32.");
    }
    try {
        //Robots
        const matlab::data::TypedArray<double> Robots_arr = arr["Robots"];
        size_t nelem = Robots_arr.getNumberOfElements();
        	msg->Robots.resize(nelem);
        	std::copy(Robots_arr.begin(), Robots_arr.begin()+nelem, msg->Robots.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Robots' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Robots' is wrong type; expected a double.");
    }
    try {
        //RobotWaitingDistance
        const matlab::data::TypedArray<double> RobotWaitingDistance_arr = arr["RobotWaitingDistance"];
        size_t nelem = RobotWaitingDistance_arr.getNumberOfElements();
        	msg->RobotWaitingDistance.resize(nelem);
        	std::copy(RobotWaitingDistance_arr.begin(), RobotWaitingDistance_arr.begin()+nelem, msg->RobotWaitingDistance.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'RobotWaitingDistance' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'RobotWaitingDistance' is wrong type; expected a double.");
    }
    try {
        //RobotVelocityProximity
        const matlab::data::TypedArray<double> RobotVelocityProximity_arr = arr["RobotVelocityProximity"];
        size_t nelem = RobotVelocityProximity_arr.getNumberOfElements();
        	msg->RobotVelocityProximity.resize(nelem);
        	std::copy(RobotVelocityProximity_arr.begin(), RobotVelocityProximity_arr.begin()+nelem, msg->RobotVelocityProximity.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'RobotVelocityProximity' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'RobotVelocityProximity' is wrong type; expected a double.");
    }
    try {
        //RobotMinVelocityProximity
        const matlab::data::TypedArray<double> RobotMinVelocityProximity_arr = arr["RobotMinVelocityProximity"];
        size_t nelem = RobotMinVelocityProximity_arr.getNumberOfElements();
        	msg->RobotMinVelocityProximity.resize(nelem);
        	std::copy(RobotMinVelocityProximity_arr.begin(), RobotMinVelocityProximity_arr.begin()+nelem, msg->RobotMinVelocityProximity.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'RobotMinVelocityProximity' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'RobotMinVelocityProximity' is wrong type; expected a double.");
    }
    try {
        //RobotMaxVelocityProximity
        const matlab::data::TypedArray<double> RobotMaxVelocityProximity_arr = arr["RobotMaxVelocityProximity"];
        size_t nelem = RobotMaxVelocityProximity_arr.getNumberOfElements();
        	msg->RobotMaxVelocityProximity.resize(nelem);
        	std::copy(RobotMaxVelocityProximity_arr.begin(), RobotMaxVelocityProximity_arr.begin()+nelem, msg->RobotMaxVelocityProximity.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'RobotMaxVelocityProximity' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'RobotMaxVelocityProximity' is wrong type; expected a double.");
    }
    try {
        //RobotVelocityProximityWeight
        const matlab::data::TypedArray<double> RobotVelocityProximityWeight_arr = arr["RobotVelocityProximityWeight"];
        size_t nelem = RobotVelocityProximityWeight_arr.getNumberOfElements();
        	msg->RobotVelocityProximityWeight.resize(nelem);
        	std::copy(RobotVelocityProximityWeight_arr.begin(), RobotVelocityProximityWeight_arr.begin()+nelem, msg->RobotVelocityProximityWeight.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'RobotVelocityProximityWeight' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'RobotVelocityProximityWeight' is wrong type; expected a double.");
    }
    try {
        //WaitingTime
        const matlab::data::TypedArray<double> WaitingTime_arr = arr["WaitingTime"];
        size_t nelem = WaitingTime_arr.getNumberOfElements();
        	msg->WaitingTime.resize(nelem);
        	std::copy(WaitingTime_arr.begin(), WaitingTime_arr.begin()+nelem, msg->WaitingTime.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'WaitingTime' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'WaitingTime' is wrong type; expected a double.");
    }
    try {
        //WaitingTimeWeight
        const matlab::data::TypedArray<double> WaitingTimeWeight_arr = arr["WaitingTimeWeight"];
        size_t nelem = WaitingTimeWeight_arr.getNumberOfElements();
        	msg->WaitingTimeWeight.resize(nelem);
        	std::copy(WaitingTimeWeight_arr.begin(), WaitingTimeWeight_arr.begin()+nelem, msg->WaitingTimeWeight.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'WaitingTimeWeight' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'WaitingTimeWeight' is wrong type; expected a double.");
    }
    try {
        //StartFilling
        const matlab::data::TypedArray<double> StartFilling_arr = arr["StartFilling"];
        size_t nelem = StartFilling_arr.getNumberOfElements();
        	msg->StartFilling.resize(nelem);
        	std::copy(StartFilling_arr.begin(), StartFilling_arr.begin()+nelem, msg->StartFilling.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'StartFilling' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'StartFilling' is wrong type; expected a double.");
    }
    try {
        //FinishFilling
        const matlab::data::TypedArray<double> FinishFilling_arr = arr["FinishFilling"];
        size_t nelem = FinishFilling_arr.getNumberOfElements();
        	msg->FinishFilling.resize(nelem);
        	std::copy(FinishFilling_arr.begin(), FinishFilling_arr.begin()+nelem, msg->FinishFilling.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'FinishFilling' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'FinishFilling' is wrong type; expected a double.");
    }
    try {
        //StartServing
        const matlab::data::TypedArray<double> StartServing_arr = arr["StartServing"];
        size_t nelem = StartServing_arr.getNumberOfElements();
        	msg->StartServing.resize(nelem);
        	std::copy(StartServing_arr.begin(), StartServing_arr.begin()+nelem, msg->StartServing.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'StartServing' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'StartServing' is wrong type; expected a double.");
    }
    try {
        //FinishServing
        const matlab::data::TypedArray<double> FinishServing_arr = arr["FinishServing"];
        size_t nelem = FinishServing_arr.getNumberOfElements();
        	msg->FinishServing.resize(nelem);
        	std::copy(FinishServing_arr.begin(), FinishServing_arr.begin()+nelem, msg->FinishServing.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'FinishServing' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'FinishServing' is wrong type; expected a double.");
    }
    try {
        //TimeFilling
        const matlab::data::TypedArray<double> TimeFilling_arr = arr["TimeFilling"];
        size_t nelem = TimeFilling_arr.getNumberOfElements();
        	msg->TimeFilling.resize(nelem);
        	std::copy(TimeFilling_arr.begin(), TimeFilling_arr.begin()+nelem, msg->TimeFilling.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'TimeFilling' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'TimeFilling' is wrong type; expected a double.");
    }
    try {
        //TimeServing
        const matlab::data::TypedArray<double> TimeServing_arr = arr["TimeServing"];
        size_t nelem = TimeServing_arr.getNumberOfElements();
        	msg->TimeServing.resize(nelem);
        	std::copy(TimeServing_arr.begin(), TimeServing_arr.begin()+nelem, msg->TimeServing.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'TimeServing' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'TimeServing' is wrong type; expected a double.");
    }
    try {
        //ConfirmServing
        const matlab::data::TypedArray<int32_t> ConfirmServing_arr = arr["ConfirmServing"];
        size_t nelem = ConfirmServing_arr.getNumberOfElements();
        	msg->ConfirmServing.resize(nelem);
        	std::copy(ConfirmServing_arr.begin(), ConfirmServing_arr.begin()+nelem, msg->ConfirmServing.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'ConfirmServing' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'ConfirmServing' is wrong type; expected a int32.");
    }
    try {
        //ConfirmFilling
        const matlab::data::TypedArray<int32_t> ConfirmFilling_arr = arr["ConfirmFilling"];
        size_t nelem = ConfirmFilling_arr.getNumberOfElements();
        	msg->ConfirmFilling.resize(nelem);
        	std::copy(ConfirmFilling_arr.begin(), ConfirmFilling_arr.begin()+nelem, msg->ConfirmFilling.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'ConfirmFilling' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'ConfirmFilling' is wrong type; expected a int32.");
    }
    try {
        //Task
        const matlab::data::TypedArray<int32_t> Task_arr = arr["Task"];
        msg->Task = Task_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Task' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Task' is wrong type; expected a int32.");
    }
    try {
        //TaskFilling
        const matlab::data::TypedArray<int32_t> TaskFilling_arr = arr["TaskFilling"];
        msg->TaskFilling = TaskFilling_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'TaskFilling' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'TaskFilling' is wrong type; expected a int32.");
    }
    try {
        //Happiness
        const matlab::data::TypedArray<double> Happiness_arr = arr["Happiness"];
        size_t nelem = Happiness_arr.getNumberOfElements();
        	msg->Happiness.resize(nelem);
        	std::copy(Happiness_arr.begin(), Happiness_arr.begin()+nelem, msg->Happiness.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Happiness' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Happiness' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T diff_drive_robot_msg_HumanRobotInteraction_common::get_arr(MDFactory_T& factory, const diff_drive_robot::HumanRobotInteraction* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","HumanID","Robots","RobotWaitingDistance","RobotVelocityProximity","RobotMinVelocityProximity","RobotMaxVelocityProximity","RobotVelocityProximityWeight","WaitingTime","WaitingTimeWeight","StartFilling","FinishFilling","StartServing","FinishServing","TimeFilling","TimeServing","ConfirmServing","ConfirmFilling","Task","TaskFilling","Happiness"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("diff_drive_robot/HumanRobotInteraction");
    // HumanID
    auto currentElement_HumanID = (msg + ctr)->HumanID;
    outArray[ctr]["HumanID"] = factory.createScalar(currentElement_HumanID);
    // Robots
    auto currentElement_Robots = (msg + ctr)->Robots;
    outArray[ctr]["Robots"] = factory.createArray<diff_drive_robot::HumanRobotInteraction::_Robots_type::const_iterator, double>({currentElement_Robots.size(),1}, currentElement_Robots.begin(), currentElement_Robots.end());
    // RobotWaitingDistance
    auto currentElement_RobotWaitingDistance = (msg + ctr)->RobotWaitingDistance;
    outArray[ctr]["RobotWaitingDistance"] = factory.createArray<diff_drive_robot::HumanRobotInteraction::_RobotWaitingDistance_type::const_iterator, double>({currentElement_RobotWaitingDistance.size(),1}, currentElement_RobotWaitingDistance.begin(), currentElement_RobotWaitingDistance.end());
    // RobotVelocityProximity
    auto currentElement_RobotVelocityProximity = (msg + ctr)->RobotVelocityProximity;
    outArray[ctr]["RobotVelocityProximity"] = factory.createArray<diff_drive_robot::HumanRobotInteraction::_RobotVelocityProximity_type::const_iterator, double>({currentElement_RobotVelocityProximity.size(),1}, currentElement_RobotVelocityProximity.begin(), currentElement_RobotVelocityProximity.end());
    // RobotMinVelocityProximity
    auto currentElement_RobotMinVelocityProximity = (msg + ctr)->RobotMinVelocityProximity;
    outArray[ctr]["RobotMinVelocityProximity"] = factory.createArray<diff_drive_robot::HumanRobotInteraction::_RobotMinVelocityProximity_type::const_iterator, double>({currentElement_RobotMinVelocityProximity.size(),1}, currentElement_RobotMinVelocityProximity.begin(), currentElement_RobotMinVelocityProximity.end());
    // RobotMaxVelocityProximity
    auto currentElement_RobotMaxVelocityProximity = (msg + ctr)->RobotMaxVelocityProximity;
    outArray[ctr]["RobotMaxVelocityProximity"] = factory.createArray<diff_drive_robot::HumanRobotInteraction::_RobotMaxVelocityProximity_type::const_iterator, double>({currentElement_RobotMaxVelocityProximity.size(),1}, currentElement_RobotMaxVelocityProximity.begin(), currentElement_RobotMaxVelocityProximity.end());
    // RobotVelocityProximityWeight
    auto currentElement_RobotVelocityProximityWeight = (msg + ctr)->RobotVelocityProximityWeight;
    outArray[ctr]["RobotVelocityProximityWeight"] = factory.createArray<diff_drive_robot::HumanRobotInteraction::_RobotVelocityProximityWeight_type::const_iterator, double>({currentElement_RobotVelocityProximityWeight.size(),1}, currentElement_RobotVelocityProximityWeight.begin(), currentElement_RobotVelocityProximityWeight.end());
    // WaitingTime
    auto currentElement_WaitingTime = (msg + ctr)->WaitingTime;
    outArray[ctr]["WaitingTime"] = factory.createArray<diff_drive_robot::HumanRobotInteraction::_WaitingTime_type::const_iterator, double>({currentElement_WaitingTime.size(),1}, currentElement_WaitingTime.begin(), currentElement_WaitingTime.end());
    // WaitingTimeWeight
    auto currentElement_WaitingTimeWeight = (msg + ctr)->WaitingTimeWeight;
    outArray[ctr]["WaitingTimeWeight"] = factory.createArray<diff_drive_robot::HumanRobotInteraction::_WaitingTimeWeight_type::const_iterator, double>({currentElement_WaitingTimeWeight.size(),1}, currentElement_WaitingTimeWeight.begin(), currentElement_WaitingTimeWeight.end());
    // StartFilling
    auto currentElement_StartFilling = (msg + ctr)->StartFilling;
    outArray[ctr]["StartFilling"] = factory.createArray<diff_drive_robot::HumanRobotInteraction::_StartFilling_type::const_iterator, double>({currentElement_StartFilling.size(),1}, currentElement_StartFilling.begin(), currentElement_StartFilling.end());
    // FinishFilling
    auto currentElement_FinishFilling = (msg + ctr)->FinishFilling;
    outArray[ctr]["FinishFilling"] = factory.createArray<diff_drive_robot::HumanRobotInteraction::_FinishFilling_type::const_iterator, double>({currentElement_FinishFilling.size(),1}, currentElement_FinishFilling.begin(), currentElement_FinishFilling.end());
    // StartServing
    auto currentElement_StartServing = (msg + ctr)->StartServing;
    outArray[ctr]["StartServing"] = factory.createArray<diff_drive_robot::HumanRobotInteraction::_StartServing_type::const_iterator, double>({currentElement_StartServing.size(),1}, currentElement_StartServing.begin(), currentElement_StartServing.end());
    // FinishServing
    auto currentElement_FinishServing = (msg + ctr)->FinishServing;
    outArray[ctr]["FinishServing"] = factory.createArray<diff_drive_robot::HumanRobotInteraction::_FinishServing_type::const_iterator, double>({currentElement_FinishServing.size(),1}, currentElement_FinishServing.begin(), currentElement_FinishServing.end());
    // TimeFilling
    auto currentElement_TimeFilling = (msg + ctr)->TimeFilling;
    outArray[ctr]["TimeFilling"] = factory.createArray<diff_drive_robot::HumanRobotInteraction::_TimeFilling_type::const_iterator, double>({currentElement_TimeFilling.size(),1}, currentElement_TimeFilling.begin(), currentElement_TimeFilling.end());
    // TimeServing
    auto currentElement_TimeServing = (msg + ctr)->TimeServing;
    outArray[ctr]["TimeServing"] = factory.createArray<diff_drive_robot::HumanRobotInteraction::_TimeServing_type::const_iterator, double>({currentElement_TimeServing.size(),1}, currentElement_TimeServing.begin(), currentElement_TimeServing.end());
    // ConfirmServing
    auto currentElement_ConfirmServing = (msg + ctr)->ConfirmServing;
    outArray[ctr]["ConfirmServing"] = factory.createArray<diff_drive_robot::HumanRobotInteraction::_ConfirmServing_type::const_iterator, int32_t>({currentElement_ConfirmServing.size(),1}, currentElement_ConfirmServing.begin(), currentElement_ConfirmServing.end());
    // ConfirmFilling
    auto currentElement_ConfirmFilling = (msg + ctr)->ConfirmFilling;
    outArray[ctr]["ConfirmFilling"] = factory.createArray<diff_drive_robot::HumanRobotInteraction::_ConfirmFilling_type::const_iterator, int32_t>({currentElement_ConfirmFilling.size(),1}, currentElement_ConfirmFilling.begin(), currentElement_ConfirmFilling.end());
    // Task
    auto currentElement_Task = (msg + ctr)->Task;
    outArray[ctr]["Task"] = factory.createScalar(currentElement_Task);
    // TaskFilling
    auto currentElement_TaskFilling = (msg + ctr)->TaskFilling;
    outArray[ctr]["TaskFilling"] = factory.createScalar(currentElement_TaskFilling);
    // Happiness
    auto currentElement_Happiness = (msg + ctr)->Happiness;
    outArray[ctr]["Happiness"] = factory.createArray<diff_drive_robot::HumanRobotInteraction::_Happiness_type::const_iterator, double>({currentElement_Happiness.size(),1}, currentElement_Happiness.begin(), currentElement_Happiness.end());
    }
    return std::move(outArray);
  } 
class DIFF_DRIVE_ROBOT_EXPORT diff_drive_robot_HumanRobotInteraction_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~diff_drive_robot_HumanRobotInteraction_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          diff_drive_robot_HumanRobotInteraction_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<diff_drive_robot::HumanRobotInteraction,diff_drive_robot_msg_HumanRobotInteraction_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         diff_drive_robot_HumanRobotInteraction_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<diff_drive_robot::HumanRobotInteraction,diff_drive_robot::HumanRobotInteraction::ConstPtr,diff_drive_robot_msg_HumanRobotInteraction_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         diff_drive_robot_HumanRobotInteraction_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<diff_drive_robot::HumanRobotInteraction,diff_drive_robot_msg_HumanRobotInteraction_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(diff_drive_robot_msg_HumanRobotInteraction_common, MATLABROSMsgInterface<diff_drive_robot::HumanRobotInteraction>)
CLASS_LOADER_REGISTER_CLASS(diff_drive_robot_HumanRobotInteraction_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1