// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for diff_drive_robot/TimeMessage
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
#include "diff_drive_robot/TimeMessage.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class DIFF_DRIVE_ROBOT_EXPORT diff_drive_robot_msg_TimeMessage_common : public MATLABROSMsgInterface<diff_drive_robot::TimeMessage> {
  public:
    virtual ~diff_drive_robot_msg_TimeMessage_common(){}
    virtual void copy_from_struct(diff_drive_robot::TimeMessage* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const diff_drive_robot::TimeMessage* msg, MultiLibLoader loader, size_t size = 1);
};
  void diff_drive_robot_msg_TimeMessage_common::copy_from_struct(diff_drive_robot::TimeMessage* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //time
        const matlab::data::CharArray time_arr = arr["Time"];
        msg->time = time_arr.toAscii();
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Time' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Time' is wrong type; expected a string.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T diff_drive_robot_msg_TimeMessage_common::get_arr(MDFactory_T& factory, const diff_drive_robot::TimeMessage* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Time"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("diff_drive_robot/TimeMessage");
    // time
    auto currentElement_time = (msg + ctr)->time;
    outArray[ctr]["Time"] = factory.createCharArray(currentElement_time);
    }
    return std::move(outArray);
  } 
class DIFF_DRIVE_ROBOT_EXPORT diff_drive_robot_TimeMessage_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~diff_drive_robot_TimeMessage_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          diff_drive_robot_TimeMessage_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<diff_drive_robot::TimeMessage,diff_drive_robot_msg_TimeMessage_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         diff_drive_robot_TimeMessage_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<diff_drive_robot::TimeMessage,diff_drive_robot::TimeMessage::ConstPtr,diff_drive_robot_msg_TimeMessage_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         diff_drive_robot_TimeMessage_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<diff_drive_robot::TimeMessage,diff_drive_robot_msg_TimeMessage_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(diff_drive_robot_msg_TimeMessage_common, MATLABROSMsgInterface<diff_drive_robot::TimeMessage>)
CLASS_LOADER_REGISTER_CLASS(diff_drive_robot_TimeMessage_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1