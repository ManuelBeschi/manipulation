// Copyright 2019-2020 The MathWorks, Inc.
// Common copy functions for manipulation_msgs/AddLocationsRequest
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
#include "manipulation_msgs/AddLocationsRequest.h"
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
namespace AddLocationsRequest {
  void copy_from_arr_manipulation_msgs_Location_var_arr(manipulation_msgs::Location& val, const matlab::data::Struct& arr);
  MDArray_T get_arr_manipulation_msgs_Location_var_arr(MDFactory_T& factory, const manipulation_msgs::AddLocationsRequest::_locations_type& val);
  void copy_from_arr_geometry_msgs_Pose(geometry_msgs::Pose& val, const matlab::data::StructArray& arr);
  MDArray_T get_arr_geometry_msgs_Pose(MDFactory_T& factory, const manipulation_msgs::Location::_pose_type& val);
  void copy_from_arr_geometry_msgs_Point(geometry_msgs::Point& val, const matlab::data::StructArray& arr);
  MDArray_T get_arr_geometry_msgs_Point(MDFactory_T& factory, const geometry_msgs::Pose::_position_type& val);
  void copy_from_arr_geometry_msgs_Quaternion(geometry_msgs::Quaternion& val, const matlab::data::StructArray& arr);
  MDArray_T get_arr_geometry_msgs_Quaternion(MDFactory_T& factory, const geometry_msgs::Pose::_orientation_type& val);
  //----------------------------------------------------------------------------
  void copy_from_arr_manipulation_msgs_Location_var_arr(manipulation_msgs::Location& val, const matlab::data::Struct& arr) {
    // _manipulation_msgs_Location_var_arr.id
    try {
        const matlab::data::CharArray _locationsid_arr = arr["id"];
        val.id = _locationsid_arr.toAscii();
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'id' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'id' is wrong type; expected a string.");
    }
    // _manipulation_msgs_Location_var_arr.frame
    try {
        const matlab::data::CharArray _locationsframe_arr = arr["frame"];
        val.frame = _locationsframe_arr.toAscii();
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'frame' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'frame' is wrong type; expected a string.");
    }
    // _manipulation_msgs_Location_var_arr.pose
    try {
        const matlab::data::StructArray _locationspose_arr = arr["pose"];
        copy_from_arr_geometry_msgs_Pose(val.pose,_locationspose_arr);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'pose' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'pose' is wrong type; expected a struct.");
    }
    // _manipulation_msgs_Location_var_arr.approach_relative_pose
    try {
        const matlab::data::StructArray _locationsapproach_relative_pose_arr = arr["approach_relative_pose"];
        copy_from_arr_geometry_msgs_Pose(val.approach_relative_pose,_locationsapproach_relative_pose_arr);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'approach_relative_pose' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'approach_relative_pose' is wrong type; expected a struct.");
    }
    // _manipulation_msgs_Location_var_arr.return_relative_pose
    try {
        const matlab::data::StructArray _locationsreturn_relative_pose_arr = arr["return_relative_pose"];
        copy_from_arr_geometry_msgs_Pose(val.return_relative_pose,_locationsreturn_relative_pose_arr);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'return_relative_pose' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'return_relative_pose' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T get_arr_manipulation_msgs_Location_var_arr(MDFactory_T& factory, const manipulation_msgs::AddLocationsRequest::_locations_type& val) {
    auto _locationsoutArray = factory.createStructArray({1,val.size()},{"id","frame","pose","approach_relative_pose","return_relative_pose"});
    for (size_t idx = 0; idx < val.size(); ++idx){
    // _manipulation_msgs_Location_var_arr.id
    	_locationsoutArray[idx]["id"] = factory.createCharArray(val[idx].id);
    // _manipulation_msgs_Location_var_arr.frame
    	_locationsoutArray[idx]["frame"] = factory.createCharArray(val[idx].frame);
    // _manipulation_msgs_Location_var_arr.pose
    _locationsoutArray[idx]["pose"] = get_arr_geometry_msgs_Pose(factory, val[idx].pose);
    // _manipulation_msgs_Location_var_arr.approach_relative_pose
    _locationsoutArray[idx]["approach_relative_pose"] = get_arr_geometry_msgs_Pose(factory, val[idx].approach_relative_pose);
    // _manipulation_msgs_Location_var_arr.return_relative_pose
    _locationsoutArray[idx]["return_relative_pose"] = get_arr_geometry_msgs_Pose(factory, val[idx].return_relative_pose);
    }
    return std::move(_locationsoutArray);
  }
  //----------------------------------------------------------------------------
  void copy_from_arr_geometry_msgs_Pose(geometry_msgs::Pose& val, const matlab::data::StructArray& arr) {
    // _geometry_msgs_Pose.position
    try {
        const matlab::data::StructArray _locations_poseposition_arr = arr[0]["position"];
        copy_from_arr_geometry_msgs_Point(val.position,_locations_poseposition_arr);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'position' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'position' is wrong type; expected a struct.");
    }
    // _geometry_msgs_Pose.orientation
    try {
        const matlab::data::StructArray _locations_poseorientation_arr = arr[0]["orientation"];
        copy_from_arr_geometry_msgs_Quaternion(val.orientation,_locations_poseorientation_arr);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'orientation' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'orientation' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T get_arr_geometry_msgs_Pose(MDFactory_T& factory, const manipulation_msgs::Location::_pose_type& val) {
    auto _locations_poseoutArray = factory.createStructArray({1,1},{"position","orientation"});
    // _geometry_msgs_Pose.position
    _locations_poseoutArray[0]["position"] = get_arr_geometry_msgs_Point(factory, val.position);
    // _geometry_msgs_Pose.orientation
    _locations_poseoutArray[0]["orientation"] = get_arr_geometry_msgs_Quaternion(factory, val.orientation);
    return std::move(_locations_poseoutArray);
  }
  //----------------------------------------------------------------------------
  void copy_from_arr_geometry_msgs_Point(geometry_msgs::Point& val, const matlab::data::StructArray& arr) {
    // _geometry_msgs_Point.x
    try {
        const matlab::data::TypedArray<double> _locations_pose_positionx_arr = arr[0]["x"];
        val.x = _locations_pose_positionx_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'x' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'x' is wrong type; expected a double.");
    }
    // _geometry_msgs_Point.y
    try {
        const matlab::data::TypedArray<double> _locations_pose_positiony_arr = arr[0]["y"];
        val.y = _locations_pose_positiony_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'y' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'y' is wrong type; expected a double.");
    }
    // _geometry_msgs_Point.z
    try {
        const matlab::data::TypedArray<double> _locations_pose_positionz_arr = arr[0]["z"];
        val.z = _locations_pose_positionz_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'z' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'z' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T get_arr_geometry_msgs_Point(MDFactory_T& factory, const geometry_msgs::Pose::_position_type& val) {
    auto _locations_pose_positionoutArray = factory.createStructArray({1,1},{"x","y","z"});
    // _geometry_msgs_Point.x
    _locations_pose_positionoutArray[0]["x"] = factory.createScalar(val.x);
    // _geometry_msgs_Point.y
    _locations_pose_positionoutArray[0]["y"] = factory.createScalar(val.y);
    // _geometry_msgs_Point.z
    _locations_pose_positionoutArray[0]["z"] = factory.createScalar(val.z);
    return std::move(_locations_pose_positionoutArray);
  }
  //----------------------------------------------------------------------------
  void copy_from_arr_geometry_msgs_Quaternion(geometry_msgs::Quaternion& val, const matlab::data::StructArray& arr) {
    // _geometry_msgs_Quaternion.x
    try {
        const matlab::data::TypedArray<double> _locations_pose_orientationx_arr = arr[0]["x"];
        val.x = _locations_pose_orientationx_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'x' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'x' is wrong type; expected a double.");
    }
    // _geometry_msgs_Quaternion.y
    try {
        const matlab::data::TypedArray<double> _locations_pose_orientationy_arr = arr[0]["y"];
        val.y = _locations_pose_orientationy_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'y' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'y' is wrong type; expected a double.");
    }
    // _geometry_msgs_Quaternion.z
    try {
        const matlab::data::TypedArray<double> _locations_pose_orientationz_arr = arr[0]["z"];
        val.z = _locations_pose_orientationz_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'z' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'z' is wrong type; expected a double.");
    }
    // _geometry_msgs_Quaternion.w
    try {
        const matlab::data::TypedArray<double> _locations_pose_orientationw_arr = arr[0]["w"];
        val.w = _locations_pose_orientationw_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'w' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'w' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T get_arr_geometry_msgs_Quaternion(MDFactory_T& factory, const geometry_msgs::Pose::_orientation_type& val) {
    auto _locations_pose_orientationoutArray = factory.createStructArray({1,1},{"x","y","z","w"});
    // _geometry_msgs_Quaternion.x
    _locations_pose_orientationoutArray[0]["x"] = factory.createScalar(val.x);
    // _geometry_msgs_Quaternion.y
    _locations_pose_orientationoutArray[0]["y"] = factory.createScalar(val.y);
    // _geometry_msgs_Quaternion.z
    _locations_pose_orientationoutArray[0]["z"] = factory.createScalar(val.z);
    // _geometry_msgs_Quaternion.w
    _locations_pose_orientationoutArray[0]["w"] = factory.createScalar(val.w);
    return std::move(_locations_pose_orientationoutArray);
  }
  //----------------------------------------------------------------------------
  MANIPULATION_MSGS_EXPORT void copy_from_arr(boost::shared_ptr<manipulation_msgs::AddLocationsRequest>& msg, const matlab::data::StructArray arr) {
    try {
        //locations
        const matlab::data::StructArray locations_arr = arr[0]["locations"];
        for (auto _locationsarr : locations_arr) {
        	manipulation_msgs::Location _val;
        	copy_from_arr_manipulation_msgs_Location_var_arr(_val,_locationsarr);
        	msg->locations.push_back(_val);
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'locations' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'locations' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MANIPULATION_MSGS_EXPORT MDArray_T get_arr(MDFactory_T& factory, const boost::shared_ptr<const manipulation_msgs::AddLocationsRequest>& msg) {
    auto outArray = factory.createStructArray({1,1},{"locations"});
    // locations
    outArray[0]["locations"] = get_arr_manipulation_msgs_Location_var_arr(factory, msg->locations);
    return std::move(outArray);
  }
} //namespace AddLocationsRequest
} //namespace matlabhelper
} //namespace manipulation_msgs
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1