// Copyright 2019-2020 The MathWorks, Inc.
// Common copy functions for manipulation_msgs/ListOfObjectsResponse
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
#endif //_MSC_VER
#include "ros/ros.h"
#include "manipulation_msgs/ListOfObjectsResponse.h"
#include "visibility_control.h"
#ifndef FOUNDATION_MATLABDATA_API
#include "MDArray.hpp"
#include "StructArray.hpp"
#include "TypedArrayRef.hpp"
#include "Struct.hpp"
#include "ArrayFactory.hpp"
#include "StructRef.hpp"
#include "Reference.hpp"
#endif
#ifndef FOUNDATION_MATLABDATA_API
typedef matlab::data::Array MDArray_T;
typedef matlab::data::ArrayFactory MDFactory_T;
#else
typedef foundation::matlabdata::Array MDArray_T;
typedef foundation::matlabdata::standalone::ClientArrayFactory MDFactory_T;
#endif
namespace manipulation_msgs {
namespace matlabhelper {
namespace ListOfObjectsResponse {
  //----------------------------------------------------------------------------
  MANIPULATION_MSGS_EXPORT void copy_from_arr(boost::shared_ptr<manipulation_msgs::ListOfObjectsResponse>& msg, const matlab::data::StructArray arr) {
    try {
        //type
        const matlab::data::CellArray type_cellarr = arr[0]["type"];
        size_t nelem = type_cellarr.getNumberOfElements();
        for (size_t idx=0; idx < nelem; ++idx){
        	const matlab::data::CharArray type_arr = type_cellarr[idx];
        	msg->type.push_back(type_arr.toAscii());
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'type' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'type' is wrong type; expected a string.");
    }
    try {
        //id
        const matlab::data::CellArray id_cellarr = arr[0]["id"];
        size_t nelem = id_cellarr.getNumberOfElements();
        for (size_t idx=0; idx < nelem; ++idx){
        	const matlab::data::CharArray id_arr = id_cellarr[idx];
        	msg->id.push_back(id_arr.toAscii());
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'id' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'id' is wrong type; expected a string.");
    }
    try {
        //box
        const matlab::data::CellArray box_cellarr = arr[0]["box"];
        size_t nelem = box_cellarr.getNumberOfElements();
        for (size_t idx=0; idx < nelem; ++idx){
        	const matlab::data::CharArray box_arr = box_cellarr[idx];
        	msg->box.push_back(box_arr.toAscii());
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'box' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'box' is wrong type; expected a string.");
    }
  }
  //----------------------------------------------------------------------------
  MANIPULATION_MSGS_EXPORT MDArray_T get_arr(MDFactory_T& factory, const boost::shared_ptr<const manipulation_msgs::ListOfObjectsResponse>& msg) {
    auto outArray = factory.createStructArray({1,1},{"type","id","box"});
    // type
    auto typeoutCell = factory.createCellArray({1,msg->type.size()});
    for(size_t idxin = 0; idxin < msg->type.size(); ++ idxin){
    	typeoutCell[idxin] = factory.createCharArray(msg->type[idxin]);
    }
    outArray[0]["type"] = typeoutCell;
    // id
    auto idoutCell = factory.createCellArray({1,msg->id.size()});
    for(size_t idxin = 0; idxin < msg->id.size(); ++ idxin){
    	idoutCell[idxin] = factory.createCharArray(msg->id[idxin]);
    }
    outArray[0]["id"] = idoutCell;
    // box
    auto boxoutCell = factory.createCellArray({1,msg->box.size()});
    for(size_t idxin = 0; idxin < msg->box.size(); ++ idxin){
    	boxoutCell[idxin] = factory.createCharArray(msg->box[idxin]);
    }
    outArray[0]["box"] = boxoutCell;
    return std::move(outArray);
  }
} //namespace ListOfObjectsResponse
} //namespace matlabhelper
} //namespace manipulation_msgs
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1