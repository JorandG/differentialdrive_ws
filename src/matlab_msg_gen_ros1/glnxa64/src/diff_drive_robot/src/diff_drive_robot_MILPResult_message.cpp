// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for diff_drive_robot/MILPResult
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
#include "diff_drive_robot/MILPResult.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class DIFF_DRIVE_ROBOT_EXPORT diff_drive_robot_msg_MILPResult_common : public MATLABROSMsgInterface<diff_drive_robot::MILPResult> {
  public:
    virtual ~diff_drive_robot_msg_MILPResult_common(){}
    virtual void copy_from_struct(diff_drive_robot::MILPResult* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const diff_drive_robot::MILPResult* msg, MultiLibLoader loader, size_t size = 1);
};
  void diff_drive_robot_msg_MILPResult_common::copy_from_struct(diff_drive_robot::MILPResult* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //RobotID
        const matlab::data::TypedArray<int32_t> RobotID_arr = arr["RobotID"];
        msg->RobotID = RobotID_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'RobotID' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'RobotID' is wrong type; expected a int32.");
    }
    try {
        //Humans
        const matlab::data::TypedArray<int32_t> Humans_arr = arr["Humans"];
        size_t nelem = Humans_arr.getNumberOfElements();
        	msg->Humans.resize(nelem);
        	std::copy(Humans_arr.begin(), Humans_arr.begin()+nelem, msg->Humans.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Humans' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Humans' is wrong type; expected a int32.");
    }
    try {
        //GoingStart
        const matlab::data::TypedArray<double> GoingStart_arr = arr["GoingStart"];
        size_t nelem = GoingStart_arr.getNumberOfElements();
        	msg->GoingStart.resize(nelem);
        	std::copy(GoingStart_arr.begin(), GoingStart_arr.begin()+nelem, msg->GoingStart.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'GoingStart' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'GoingStart' is wrong type; expected a double.");
    }
    try {
        //ApproachingStart
        const matlab::data::TypedArray<double> ApproachingStart_arr = arr["ApproachingStart"];
        size_t nelem = ApproachingStart_arr.getNumberOfElements();
        	msg->ApproachingStart.resize(nelem);
        	std::copy(ApproachingStart_arr.begin(), ApproachingStart_arr.begin()+nelem, msg->ApproachingStart.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'ApproachingStart' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'ApproachingStart' is wrong type; expected a double.");
    }
    try {
        //WaitingStart
        const matlab::data::TypedArray<double> WaitingStart_arr = arr["WaitingStart"];
        size_t nelem = WaitingStart_arr.getNumberOfElements();
        	msg->WaitingStart.resize(nelem);
        	std::copy(WaitingStart_arr.begin(), WaitingStart_arr.begin()+nelem, msg->WaitingStart.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'WaitingStart' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'WaitingStart' is wrong type; expected a double.");
    }
    try {
        //ServingStart
        const matlab::data::TypedArray<double> ServingStart_arr = arr["ServingStart"];
        size_t nelem = ServingStart_arr.getNumberOfElements();
        	msg->ServingStart.resize(nelem);
        	std::copy(ServingStart_arr.begin(), ServingStart_arr.begin()+nelem, msg->ServingStart.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'ServingStart' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'ServingStart' is wrong type; expected a double.");
    }
    try {
        //DepotStart
        const matlab::data::TypedArray<double> DepotStart_arr = arr["DepotStart"];
        size_t nelem = DepotStart_arr.getNumberOfElements();
        	msg->DepotStart.resize(nelem);
        	std::copy(DepotStart_arr.begin(), DepotStart_arr.begin()+nelem, msg->DepotStart.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'DepotStart' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'DepotStart' is wrong type; expected a double.");
    }
    try {
        //GoingFinish
        const matlab::data::TypedArray<double> GoingFinish_arr = arr["GoingFinish"];
        size_t nelem = GoingFinish_arr.getNumberOfElements();
        	msg->GoingFinish.resize(nelem);
        	std::copy(GoingFinish_arr.begin(), GoingFinish_arr.begin()+nelem, msg->GoingFinish.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'GoingFinish' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'GoingFinish' is wrong type; expected a double.");
    }
    try {
        //ApproachingFinish
        const matlab::data::TypedArray<double> ApproachingFinish_arr = arr["ApproachingFinish"];
        size_t nelem = ApproachingFinish_arr.getNumberOfElements();
        	msg->ApproachingFinish.resize(nelem);
        	std::copy(ApproachingFinish_arr.begin(), ApproachingFinish_arr.begin()+nelem, msg->ApproachingFinish.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'ApproachingFinish' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'ApproachingFinish' is wrong type; expected a double.");
    }
    try {
        //WaitingFinish
        const matlab::data::TypedArray<double> WaitingFinish_arr = arr["WaitingFinish"];
        size_t nelem = WaitingFinish_arr.getNumberOfElements();
        	msg->WaitingFinish.resize(nelem);
        	std::copy(WaitingFinish_arr.begin(), WaitingFinish_arr.begin()+nelem, msg->WaitingFinish.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'WaitingFinish' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'WaitingFinish' is wrong type; expected a double.");
    }
    try {
        //ServingFinish
        const matlab::data::TypedArray<double> ServingFinish_arr = arr["ServingFinish"];
        size_t nelem = ServingFinish_arr.getNumberOfElements();
        	msg->ServingFinish.resize(nelem);
        	std::copy(ServingFinish_arr.begin(), ServingFinish_arr.begin()+nelem, msg->ServingFinish.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'ServingFinish' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'ServingFinish' is wrong type; expected a double.");
    }
    try {
        //DepotFinish
        const matlab::data::TypedArray<double> DepotFinish_arr = arr["DepotFinish"];
        size_t nelem = DepotFinish_arr.getNumberOfElements();
        	msg->DepotFinish.resize(nelem);
        	std::copy(DepotFinish_arr.begin(), DepotFinish_arr.begin()+nelem, msg->DepotFinish.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'DepotFinish' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'DepotFinish' is wrong type; expected a double.");
    }
    try {
        //FinishedFilling
        const matlab::data::TypedArray<int32_t> FinishedFilling_arr = arr["FinishedFilling"];
        size_t nelem = FinishedFilling_arr.getNumberOfElements();
        	msg->FinishedFilling.resize(nelem);
        	std::copy(FinishedFilling_arr.begin(), FinishedFilling_arr.begin()+nelem, msg->FinishedFilling.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'FinishedFilling' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'FinishedFilling' is wrong type; expected a int32.");
    }
    try {
        //FinishedService
        const matlab::data::TypedArray<int32_t> FinishedService_arr = arr["FinishedService"];
        size_t nelem = FinishedService_arr.getNumberOfElements();
        	msg->FinishedService.resize(nelem);
        	std::copy(FinishedService_arr.begin(), FinishedService_arr.begin()+nelem, msg->FinishedService.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'FinishedService' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'FinishedService' is wrong type; expected a int32.");
    }
    try {
        //DistanceWaiting
        const matlab::data::TypedArray<double> DistanceWaiting_arr = arr["DistanceWaiting"];
        size_t nelem = DistanceWaiting_arr.getNumberOfElements();
        	msg->DistanceWaiting.resize(nelem);
        	std::copy(DistanceWaiting_arr.begin(), DistanceWaiting_arr.begin()+nelem, msg->DistanceWaiting.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'DistanceWaiting' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'DistanceWaiting' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T diff_drive_robot_msg_MILPResult_common::get_arr(MDFactory_T& factory, const diff_drive_robot::MILPResult* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","RobotID","Humans","GoingStart","ApproachingStart","WaitingStart","ServingStart","DepotStart","GoingFinish","ApproachingFinish","WaitingFinish","ServingFinish","DepotFinish","FinishedFilling","FinishedService","DistanceWaiting"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("diff_drive_robot/MILPResult");
    // RobotID
    auto currentElement_RobotID = (msg + ctr)->RobotID;
    outArray[ctr]["RobotID"] = factory.createScalar(currentElement_RobotID);
    // Humans
    auto currentElement_Humans = (msg + ctr)->Humans;
    outArray[ctr]["Humans"] = factory.createArray<diff_drive_robot::MILPResult::_Humans_type::const_iterator, int32_t>({currentElement_Humans.size(),1}, currentElement_Humans.begin(), currentElement_Humans.end());
    // GoingStart
    auto currentElement_GoingStart = (msg + ctr)->GoingStart;
    outArray[ctr]["GoingStart"] = factory.createArray<diff_drive_robot::MILPResult::_GoingStart_type::const_iterator, double>({currentElement_GoingStart.size(),1}, currentElement_GoingStart.begin(), currentElement_GoingStart.end());
    // ApproachingStart
    auto currentElement_ApproachingStart = (msg + ctr)->ApproachingStart;
    outArray[ctr]["ApproachingStart"] = factory.createArray<diff_drive_robot::MILPResult::_ApproachingStart_type::const_iterator, double>({currentElement_ApproachingStart.size(),1}, currentElement_ApproachingStart.begin(), currentElement_ApproachingStart.end());
    // WaitingStart
    auto currentElement_WaitingStart = (msg + ctr)->WaitingStart;
    outArray[ctr]["WaitingStart"] = factory.createArray<diff_drive_robot::MILPResult::_WaitingStart_type::const_iterator, double>({currentElement_WaitingStart.size(),1}, currentElement_WaitingStart.begin(), currentElement_WaitingStart.end());
    // ServingStart
    auto currentElement_ServingStart = (msg + ctr)->ServingStart;
    outArray[ctr]["ServingStart"] = factory.createArray<diff_drive_robot::MILPResult::_ServingStart_type::const_iterator, double>({currentElement_ServingStart.size(),1}, currentElement_ServingStart.begin(), currentElement_ServingStart.end());
    // DepotStart
    auto currentElement_DepotStart = (msg + ctr)->DepotStart;
    outArray[ctr]["DepotStart"] = factory.createArray<diff_drive_robot::MILPResult::_DepotStart_type::const_iterator, double>({currentElement_DepotStart.size(),1}, currentElement_DepotStart.begin(), currentElement_DepotStart.end());
    // GoingFinish
    auto currentElement_GoingFinish = (msg + ctr)->GoingFinish;
    outArray[ctr]["GoingFinish"] = factory.createArray<diff_drive_robot::MILPResult::_GoingFinish_type::const_iterator, double>({currentElement_GoingFinish.size(),1}, currentElement_GoingFinish.begin(), currentElement_GoingFinish.end());
    // ApproachingFinish
    auto currentElement_ApproachingFinish = (msg + ctr)->ApproachingFinish;
    outArray[ctr]["ApproachingFinish"] = factory.createArray<diff_drive_robot::MILPResult::_ApproachingFinish_type::const_iterator, double>({currentElement_ApproachingFinish.size(),1}, currentElement_ApproachingFinish.begin(), currentElement_ApproachingFinish.end());
    // WaitingFinish
    auto currentElement_WaitingFinish = (msg + ctr)->WaitingFinish;
    outArray[ctr]["WaitingFinish"] = factory.createArray<diff_drive_robot::MILPResult::_WaitingFinish_type::const_iterator, double>({currentElement_WaitingFinish.size(),1}, currentElement_WaitingFinish.begin(), currentElement_WaitingFinish.end());
    // ServingFinish
    auto currentElement_ServingFinish = (msg + ctr)->ServingFinish;
    outArray[ctr]["ServingFinish"] = factory.createArray<diff_drive_robot::MILPResult::_ServingFinish_type::const_iterator, double>({currentElement_ServingFinish.size(),1}, currentElement_ServingFinish.begin(), currentElement_ServingFinish.end());
    // DepotFinish
    auto currentElement_DepotFinish = (msg + ctr)->DepotFinish;
    outArray[ctr]["DepotFinish"] = factory.createArray<diff_drive_robot::MILPResult::_DepotFinish_type::const_iterator, double>({currentElement_DepotFinish.size(),1}, currentElement_DepotFinish.begin(), currentElement_DepotFinish.end());
    // FinishedFilling
    auto currentElement_FinishedFilling = (msg + ctr)->FinishedFilling;
    outArray[ctr]["FinishedFilling"] = factory.createArray<diff_drive_robot::MILPResult::_FinishedFilling_type::const_iterator, int32_t>({currentElement_FinishedFilling.size(),1}, currentElement_FinishedFilling.begin(), currentElement_FinishedFilling.end());
    // FinishedService
    auto currentElement_FinishedService = (msg + ctr)->FinishedService;
    outArray[ctr]["FinishedService"] = factory.createArray<diff_drive_robot::MILPResult::_FinishedService_type::const_iterator, int32_t>({currentElement_FinishedService.size(),1}, currentElement_FinishedService.begin(), currentElement_FinishedService.end());
    // DistanceWaiting
    auto currentElement_DistanceWaiting = (msg + ctr)->DistanceWaiting;
    outArray[ctr]["DistanceWaiting"] = factory.createArray<diff_drive_robot::MILPResult::_DistanceWaiting_type::const_iterator, double>({currentElement_DistanceWaiting.size(),1}, currentElement_DistanceWaiting.begin(), currentElement_DistanceWaiting.end());
    }
    return std::move(outArray);
  } 
class DIFF_DRIVE_ROBOT_EXPORT diff_drive_robot_MILPResult_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~diff_drive_robot_MILPResult_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          diff_drive_robot_MILPResult_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<diff_drive_robot::MILPResult,diff_drive_robot_msg_MILPResult_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         diff_drive_robot_MILPResult_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<diff_drive_robot::MILPResult,diff_drive_robot::MILPResult::ConstPtr,diff_drive_robot_msg_MILPResult_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         diff_drive_robot_MILPResult_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<diff_drive_robot::MILPResult,diff_drive_robot_msg_MILPResult_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(diff_drive_robot_msg_MILPResult_common, MATLABROSMsgInterface<diff_drive_robot::MILPResult>)
CLASS_LOADER_REGISTER_CLASS(diff_drive_robot_MILPResult_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1