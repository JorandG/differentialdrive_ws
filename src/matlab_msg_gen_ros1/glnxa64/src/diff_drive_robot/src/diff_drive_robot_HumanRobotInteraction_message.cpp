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
        //RobotVelocity
        const matlab::data::TypedArray<double> RobotVelocity_arr = arr["RobotVelocity"];
        size_t nelem = RobotVelocity_arr.getNumberOfElements();
        	msg->RobotVelocity.resize(nelem);
        	std::copy(RobotVelocity_arr.begin(), RobotVelocity_arr.begin()+nelem, msg->RobotVelocity.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'RobotVelocity' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'RobotVelocity' is wrong type; expected a double.");
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
        msg->ConfirmServing = ConfirmServing_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'ConfirmServing' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'ConfirmServing' is wrong type; expected a int32.");
    }
    try {
        //ConfirmFilling
        const matlab::data::TypedArray<int32_t> ConfirmFilling_arr = arr["ConfirmFilling"];
        msg->ConfirmFilling = ConfirmFilling_arr[0];
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
  }
  //----------------------------------------------------------------------------
  MDArray_T diff_drive_robot_msg_HumanRobotInteraction_common::get_arr(MDFactory_T& factory, const diff_drive_robot::HumanRobotInteraction* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","HumanID","RobotVelocity","WaitingTime","StartFilling","FinishFilling","StartServing","FinishServing","TimeFilling","TimeServing","ConfirmServing","ConfirmFilling","Task"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("diff_drive_robot/HumanRobotInteraction");
    // HumanID
    auto currentElement_HumanID = (msg + ctr)->HumanID;
    outArray[ctr]["HumanID"] = factory.createScalar(currentElement_HumanID);
    // RobotVelocity
    auto currentElement_RobotVelocity = (msg + ctr)->RobotVelocity;
    outArray[ctr]["RobotVelocity"] = factory.createArray<diff_drive_robot::HumanRobotInteraction::_RobotVelocity_type::const_iterator, double>({currentElement_RobotVelocity.size(),1}, currentElement_RobotVelocity.begin(), currentElement_RobotVelocity.end());
    // WaitingTime
    auto currentElement_WaitingTime = (msg + ctr)->WaitingTime;
    outArray[ctr]["WaitingTime"] = factory.createArray<diff_drive_robot::HumanRobotInteraction::_WaitingTime_type::const_iterator, double>({currentElement_WaitingTime.size(),1}, currentElement_WaitingTime.begin(), currentElement_WaitingTime.end());
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
    outArray[ctr]["ConfirmServing"] = factory.createScalar(currentElement_ConfirmServing);
    // ConfirmFilling
    auto currentElement_ConfirmFilling = (msg + ctr)->ConfirmFilling;
    outArray[ctr]["ConfirmFilling"] = factory.createScalar(currentElement_ConfirmFilling);
    // Task
    auto currentElement_Task = (msg + ctr)->Task;
    outArray[ctr]["Task"] = factory.createScalar(currentElement_Task);
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