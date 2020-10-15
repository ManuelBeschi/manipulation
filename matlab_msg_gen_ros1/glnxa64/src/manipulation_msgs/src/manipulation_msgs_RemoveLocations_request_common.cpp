// Copyright 2020 The MathWorks, Inc.
// Common copy functions for manipulation_msgs/RemoveLocationsRequest
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
#include "manipulation_msgs/RemoveLocations.h"
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
namespace RemoveLocations_Request {
  //----------------------------------------------------------------------------
  MANIPULATION_MSGS_EXPORT void copy_from_arr(manipulation_msgs::RemoveLocations::Request& msg, const matlab::data::StructArray arr) {
    try {
        //location_names
        const matlab::data::CellArray location_names_cellarr = arr[0]["location_names"];
        size_t nelem = location_names_cellarr.getNumberOfElements();
        for (size_t idx=0; idx < nelem; ++idx){
        	const matlab::data::CharArray location_names_arr = location_names_cellarr[idx];
        	msg.location_names.push_back(location_names_arr.toAscii());
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'location_names' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'location_names' is wrong type; expected a string.");
    }
  }
  MANIPULATION_MSGS_EXPORT int64_t get_requestId_from_arr(const matlab::data::StructArray arr) {
    // Get the request ID
    int64_t requestId = 0;
    try {
        // data
        const matlab::data::TypedArray<int64_t> data_arr = arr[0]["requestId"];
        requestId = data_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'requestId' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'requestId' is wrong type; expected a int64.");
    }
    return requestId;
  }
  //----------------------------------------------------------------------------
  MANIPULATION_MSGS_EXPORT MDArray_T get_arr(MDFactory_T& factory, const manipulation_msgs::RemoveLocations::Request& msg) {
    auto outArray = factory.createStructArray({1,1},{"location_names"});
    // location_names
    auto location_namesoutCell = factory.createCellArray({1,msg.location_names.size()});
    for(size_t idxin = 0; idxin < msg.location_names.size(); ++ idxin){
    	location_namesoutCell[idxin] = factory.createCharArray(msg.location_names[idxin]);
    }
    outArray[0]["location_names"] = location_namesoutCell;
    return std::move(outArray);
  }
} //namespace RemoveLocations_Request
} //namespace matlabhelper
} //namespace manipulation_msgs
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1