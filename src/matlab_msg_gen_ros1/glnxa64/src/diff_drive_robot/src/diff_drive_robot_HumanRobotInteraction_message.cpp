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
        msg->RobotVelocity = RobotVelocity_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'RobotVelocity' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'RobotVelocity' is wrong type; expected a double.");
    }
    try {
        //WaitingTime
        const matlab::data::TypedArray<double> WaitingTime_arr = arr["WaitingTime"];
        msg->WaitingTime = WaitingTime_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'WaitingTime' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'WaitingTime' is wrong type; expected a double.");
    }
    try {
        //StartFilling
        const matlab::data::TypedArray<double> StartFilling_arr = arr["StartFilling"];
        msg->StartFilling = StartFilling_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'StartFilling' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'StartFilling' is wrong type; expected a double.");
    }
    try {
        //FinishFilling
        const matlab::data::TypedArray<double> FinishFilling_arr = arr["FinishFilling"];
        msg->FinishFilling = FinishFilling_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'FinishFilling' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'FinishFilling' is wrong type; expected a double.");
    }
    try {
        //StartServing
        const matlab::data::TypedArray<double> StartServing_arr = arr["StartServing"];
        msg->StartServing = StartServing_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'StartServing' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'StartServing' is wrong type; expected a double.");
    }
    try {
        //FinishServing
        const matlab::data::TypedArray<double> FinishServing_arr = arr["FinishServing"];
        msg->FinishServing = FinishServing_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'FinishServing' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'FinishServing' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T diff_drive_robot_msg_HumanRobotInteraction_common::get_arr(MDFactory_T& factory, const diff_drive_robot::HumanRobotInteraction* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","HumanID","RobotVelocity","WaitingTime","StartFilling","FinishFilling","StartServing","FinishServing"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("diff_drive_robot/HumanRobotInteraction");
    // HumanID
    auto currentElement_HumanID = (msg + ctr)->HumanID;
    outArray[ctr]["HumanID"] = factory.createScalar(currentElement_HumanID);
    // RobotVelocity
    auto currentElement_RobotVelocity = (msg + ctr)->RobotVelocity;
    outArray[ctr]["RobotVelocity"] = factory.createScalar(currentElement_RobotVelocity);
    // WaitingTime
    auto currentElement_WaitingTime = (msg + ctr)->WaitingTime;
    outArray[ctr]["WaitingTime"] = factory.createScalar(currentElement_WaitingTime);
    // StartFilling
    auto currentElement_StartFilling = (msg + ctr)->StartFilling;
    outArray[ctr]["StartFilling"] = factory.createScalar(currentElement_StartFilling);
    // FinishFilling
    auto currentElement_FinishFilling = (msg + ctr)->FinishFilling;
    outArray[ctr]["FinishFilling"] = factory.createScalar(currentElement_FinishFilling);
    // StartServing
    auto currentElement_StartServing = (msg + ctr)->StartServing;
    outArray[ctr]["StartServing"] = factory.createScalar(currentElement_StartServing);
    // FinishServing
    auto currentElement_FinishServing = (msg + ctr)->FinishServing;
    outArray[ctr]["FinishServing"] = factory.createScalar(currentElement_FinishServing);
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