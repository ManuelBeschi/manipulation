// Copyright 2019-2020 The MathWorks, Inc.
// Common copy functions for pickplace_msgs/ListOfObjectsResponse
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
#include "pickplace_msgs/ListOfObjectsResponse.h"
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
namespace pickplace_msgs {
namespace matlabhelper {
namespace ListOfObjectsResponse {
  void copy_from_arr_pickplace_msgs_PickBox_var_arr(pickplace_msgs::PickBox& val, const matlab::data::Struct& arr);
  MDArray_T get_arr_pickplace_msgs_PickBox_var_arr(MDFactory_T& factory, const pickplace_msgs::ListOfObjectsResponse::_inbound_boxes_type& val);
  void copy_from_arr_pickplace_msgs_Object_var_arr(pickplace_msgs::Object& val, const matlab::data::Struct& arr);
  MDArray_T get_arr_pickplace_msgs_Object_var_arr(MDFactory_T& factory, const pickplace_msgs::PickBox::_objects_type& val);
  void copy_from_arr_pickplace_msgs_Grasp_var_arr(pickplace_msgs::Grasp& val, const matlab::data::Struct& arr);
  MDArray_T get_arr_pickplace_msgs_Grasp_var_arr(MDFactory_T& factory, const pickplace_msgs::Object::_grasping_poses_type& val);
  void copy_from_arr_geometry_msgs_Pose(geometry_msgs::Pose& val, const matlab::data::StructArray& arr);
  MDArray_T get_arr_geometry_msgs_Pose(MDFactory_T& factory, const pickplace_msgs::Grasp::_pose_type& val);
  void copy_from_arr_geometry_msgs_Point(geometry_msgs::Point& val, const matlab::data::StructArray& arr);
  MDArray_T get_arr_geometry_msgs_Point(MDFactory_T& factory, const geometry_msgs::Pose::_position_type& val);
  void copy_from_arr_geometry_msgs_Quaternion(geometry_msgs::Quaternion& val, const matlab::data::StructArray& arr);
  MDArray_T get_arr_geometry_msgs_Quaternion(MDFactory_T& factory, const geometry_msgs::Pose::_orientation_type& val);
  //----------------------------------------------------------------------------
  void copy_from_arr_pickplace_msgs_PickBox_var_arr(pickplace_msgs::PickBox& val, const matlab::data::Struct& arr) {
    // _pickplace_msgs_PickBox_var_arr.objects
    try {
        const matlab::data::StructArray _inbound_boxesobjects_arr = arr["objects"];
        for (auto _objectsarr : _inbound_boxesobjects_arr) {
        	pickplace_msgs::Object _val;
        	copy_from_arr_pickplace_msgs_Object_var_arr(_val,_objectsarr);
        	val.objects.push_back(_val);
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'objects' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'objects' is wrong type; expected a struct.");
    }
    // _pickplace_msgs_PickBox_var_arr.name
    try {
        const matlab::data::CharArray _inbound_boxesname_arr = arr["name"];
        val.name = _inbound_boxesname_arr.toAscii();
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'name' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'name' is wrong type; expected a string.");
    }
    // _pickplace_msgs_PickBox_var_arr.approach_pose
    try {
        const matlab::data::StructArray _inbound_boxesapproach_pose_arr = arr["approach_pose"];
        copy_from_arr_geometry_msgs_Pose(val.approach_pose,_inbound_boxesapproach_pose_arr);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'approach_pose' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'approach_pose' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T get_arr_pickplace_msgs_PickBox_var_arr(MDFactory_T& factory, const pickplace_msgs::ListOfObjectsResponse::_inbound_boxes_type& val) {
    auto _inbound_boxesoutArray = factory.createStructArray({1,val.size()},{"objects","name","approach_pose"});
    for (size_t idx = 0; idx < val.size(); ++idx){
    // _pickplace_msgs_PickBox_var_arr.objects
    _inbound_boxesoutArray[idx]["objects"] = get_arr_pickplace_msgs_Object_var_arr(factory, val[idx].objects);
    // _pickplace_msgs_PickBox_var_arr.name
    	_inbound_boxesoutArray[idx]["name"] = factory.createCharArray(val[idx].name);
    // _pickplace_msgs_PickBox_var_arr.approach_pose
    _inbound_boxesoutArray[idx]["approach_pose"] = get_arr_geometry_msgs_Pose(factory, val[idx].approach_pose);
    }
    return std::move(_inbound_boxesoutArray);
  }
  //----------------------------------------------------------------------------
  void copy_from_arr_pickplace_msgs_Object_var_arr(pickplace_msgs::Object& val, const matlab::data::Struct& arr) {
    // _pickplace_msgs_Object_var_arr.grasping_poses
    try {
        const matlab::data::StructArray _inbound_boxes_objectsgrasping_poses_arr = arr["grasping_poses"];
        for (auto _grasping_posesarr : _inbound_boxes_objectsgrasping_poses_arr) {
        	pickplace_msgs::Grasp _val;
        	copy_from_arr_pickplace_msgs_Grasp_var_arr(_val,_grasping_posesarr);
        	val.grasping_poses.push_back(_val);
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'grasping_poses' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'grasping_poses' is wrong type; expected a struct.");
    }
    // _pickplace_msgs_Object_var_arr.type
    try {
        const matlab::data::CharArray _inbound_boxes_objectstype_arr = arr["type"];
        val.type = _inbound_boxes_objectstype_arr.toAscii();
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'type' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'type' is wrong type; expected a string.");
    }
    // _pickplace_msgs_Object_var_arr.id
    try {
        const matlab::data::CharArray _inbound_boxes_objectsid_arr = arr["id"];
        val.id = _inbound_boxes_objectsid_arr.toAscii();
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'id' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'id' is wrong type; expected a string.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T get_arr_pickplace_msgs_Object_var_arr(MDFactory_T& factory, const pickplace_msgs::PickBox::_objects_type& val) {
    auto _inbound_boxes_objectsoutArray = factory.createStructArray({1,val.size()},{"grasping_poses","type","id"});
    for (size_t idx = 0; idx < val.size(); ++idx){
    // _pickplace_msgs_Object_var_arr.grasping_poses
    _inbound_boxes_objectsoutArray[idx]["grasping_poses"] = get_arr_pickplace_msgs_Grasp_var_arr(factory, val[idx].grasping_poses);
    // _pickplace_msgs_Object_var_arr.type
    	_inbound_boxes_objectsoutArray[idx]["type"] = factory.createCharArray(val[idx].type);
    // _pickplace_msgs_Object_var_arr.id
    	_inbound_boxes_objectsoutArray[idx]["id"] = factory.createCharArray(val[idx].id);
    }
    return std::move(_inbound_boxes_objectsoutArray);
  }
  //----------------------------------------------------------------------------
  void copy_from_arr_pickplace_msgs_Grasp_var_arr(pickplace_msgs::Grasp& val, const matlab::data::Struct& arr) {
    // _pickplace_msgs_Grasp_var_arr.pose
    try {
        const matlab::data::StructArray _inbound_boxes_objects_grasping_posespose_arr = arr["pose"];
        copy_from_arr_geometry_msgs_Pose(val.pose,_inbound_boxes_objects_grasping_posespose_arr);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'pose' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'pose' is wrong type; expected a struct.");
    }
    // _pickplace_msgs_Grasp_var_arr.tool_name
    try {
        const matlab::data::CharArray _inbound_boxes_objects_grasping_posestool_name_arr = arr["tool_name"];
        val.tool_name = _inbound_boxes_objects_grasping_posestool_name_arr.toAscii();
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'tool_name' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'tool_name' is wrong type; expected a string.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T get_arr_pickplace_msgs_Grasp_var_arr(MDFactory_T& factory, const pickplace_msgs::Object::_grasping_poses_type& val) {
    auto _inbound_boxes_objects_grasping_posesoutArray = factory.createStructArray({1,val.size()},{"pose","tool_name"});
    for (size_t idx = 0; idx < val.size(); ++idx){
    // _pickplace_msgs_Grasp_var_arr.pose
    _inbound_boxes_objects_grasping_posesoutArray[idx]["pose"] = get_arr_geometry_msgs_Pose(factory, val[idx].pose);
    // _pickplace_msgs_Grasp_var_arr.tool_name
    	_inbound_boxes_objects_grasping_posesoutArray[idx]["tool_name"] = factory.createCharArray(val[idx].tool_name);
    }
    return std::move(_inbound_boxes_objects_grasping_posesoutArray);
  }
  //----------------------------------------------------------------------------
  void copy_from_arr_geometry_msgs_Pose(geometry_msgs::Pose& val, const matlab::data::StructArray& arr) {
    // _geometry_msgs_Pose.position
    try {
        const matlab::data::StructArray _inbound_boxes_objects_grasping_poses_poseposition_arr = arr[0]["position"];
        copy_from_arr_geometry_msgs_Point(val.position,_inbound_boxes_objects_grasping_poses_poseposition_arr);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'position' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'position' is wrong type; expected a struct.");
    }
    // _geometry_msgs_Pose.orientation
    try {
        const matlab::data::StructArray _inbound_boxes_objects_grasping_poses_poseorientation_arr = arr[0]["orientation"];
        copy_from_arr_geometry_msgs_Quaternion(val.orientation,_inbound_boxes_objects_grasping_poses_poseorientation_arr);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'orientation' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'orientation' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T get_arr_geometry_msgs_Pose(MDFactory_T& factory, const pickplace_msgs::Grasp::_pose_type& val) {
    auto _inbound_boxes_objects_grasping_poses_poseoutArray = factory.createStructArray({1,1},{"position","orientation"});
    // _geometry_msgs_Pose.position
    _inbound_boxes_objects_grasping_poses_poseoutArray[0]["position"] = get_arr_geometry_msgs_Point(factory, val.position);
    // _geometry_msgs_Pose.orientation
    _inbound_boxes_objects_grasping_poses_poseoutArray[0]["orientation"] = get_arr_geometry_msgs_Quaternion(factory, val.orientation);
    return std::move(_inbound_boxes_objects_grasping_poses_poseoutArray);
  }
  //----------------------------------------------------------------------------
  void copy_from_arr_geometry_msgs_Point(geometry_msgs::Point& val, const matlab::data::StructArray& arr) {
    // _geometry_msgs_Point.x
    try {
        const matlab::data::TypedArray<double> _inbound_boxes_objects_grasping_poses_pose_positionx_arr = arr[0]["x"];
        val.x = _inbound_boxes_objects_grasping_poses_pose_positionx_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'x' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'x' is wrong type; expected a double.");
    }
    // _geometry_msgs_Point.y
    try {
        const matlab::data::TypedArray<double> _inbound_boxes_objects_grasping_poses_pose_positiony_arr = arr[0]["y"];
        val.y = _inbound_boxes_objects_grasping_poses_pose_positiony_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'y' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'y' is wrong type; expected a double.");
    }
    // _geometry_msgs_Point.z
    try {
        const matlab::data::TypedArray<double> _inbound_boxes_objects_grasping_poses_pose_positionz_arr = arr[0]["z"];
        val.z = _inbound_boxes_objects_grasping_poses_pose_positionz_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'z' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'z' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T get_arr_geometry_msgs_Point(MDFactory_T& factory, const geometry_msgs::Pose::_position_type& val) {
    auto _inbound_boxes_objects_grasping_poses_pose_positionoutArray = factory.createStructArray({1,1},{"x","y","z"});
    // _geometry_msgs_Point.x
    _inbound_boxes_objects_grasping_poses_pose_positionoutArray[0]["x"] = factory.createScalar(val.x);
    // _geometry_msgs_Point.y
    _inbound_boxes_objects_grasping_poses_pose_positionoutArray[0]["y"] = factory.createScalar(val.y);
    // _geometry_msgs_Point.z
    _inbound_boxes_objects_grasping_poses_pose_positionoutArray[0]["z"] = factory.createScalar(val.z);
    return std::move(_inbound_boxes_objects_grasping_poses_pose_positionoutArray);
  }
  //----------------------------------------------------------------------------
  void copy_from_arr_geometry_msgs_Quaternion(geometry_msgs::Quaternion& val, const matlab::data::StructArray& arr) {
    // _geometry_msgs_Quaternion.x
    try {
        const matlab::data::TypedArray<double> _inbound_boxes_objects_grasping_poses_pose_orientationx_arr = arr[0]["x"];
        val.x = _inbound_boxes_objects_grasping_poses_pose_orientationx_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'x' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'x' is wrong type; expected a double.");
    }
    // _geometry_msgs_Quaternion.y
    try {
        const matlab::data::TypedArray<double> _inbound_boxes_objects_grasping_poses_pose_orientationy_arr = arr[0]["y"];
        val.y = _inbound_boxes_objects_grasping_poses_pose_orientationy_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'y' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'y' is wrong type; expected a double.");
    }
    // _geometry_msgs_Quaternion.z
    try {
        const matlab::data::TypedArray<double> _inbound_boxes_objects_grasping_poses_pose_orientationz_arr = arr[0]["z"];
        val.z = _inbound_boxes_objects_grasping_poses_pose_orientationz_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'z' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'z' is wrong type; expected a double.");
    }
    // _geometry_msgs_Quaternion.w
    try {
        const matlab::data::TypedArray<double> _inbound_boxes_objects_grasping_poses_pose_orientationw_arr = arr[0]["w"];
        val.w = _inbound_boxes_objects_grasping_poses_pose_orientationw_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'w' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'w' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T get_arr_geometry_msgs_Quaternion(MDFactory_T& factory, const geometry_msgs::Pose::_orientation_type& val) {
    auto _inbound_boxes_objects_grasping_poses_pose_orientationoutArray = factory.createStructArray({1,1},{"x","y","z","w"});
    // _geometry_msgs_Quaternion.x
    _inbound_boxes_objects_grasping_poses_pose_orientationoutArray[0]["x"] = factory.createScalar(val.x);
    // _geometry_msgs_Quaternion.y
    _inbound_boxes_objects_grasping_poses_pose_orientationoutArray[0]["y"] = factory.createScalar(val.y);
    // _geometry_msgs_Quaternion.z
    _inbound_boxes_objects_grasping_poses_pose_orientationoutArray[0]["z"] = factory.createScalar(val.z);
    // _geometry_msgs_Quaternion.w
    _inbound_boxes_objects_grasping_poses_pose_orientationoutArray[0]["w"] = factory.createScalar(val.w);
    return std::move(_inbound_boxes_objects_grasping_poses_pose_orientationoutArray);
  }
  //----------------------------------------------------------------------------
  PICKPLACE_MSGS_EXPORT void copy_from_arr(boost::shared_ptr<pickplace_msgs::ListOfObjectsResponse>& msg, const matlab::data::StructArray arr) {
    try {
        //inbound_boxes
        const matlab::data::StructArray inbound_boxes_arr = arr[0]["inbound_boxes"];
        for (auto _inbound_boxesarr : inbound_boxes_arr) {
        	pickplace_msgs::PickBox _val;
        	copy_from_arr_pickplace_msgs_PickBox_var_arr(_val,_inbound_boxesarr);
        	msg->inbound_boxes.push_back(_val);
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'inbound_boxes' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'inbound_boxes' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  PICKPLACE_MSGS_EXPORT MDArray_T get_arr(MDFactory_T& factory, const boost::shared_ptr<const pickplace_msgs::ListOfObjectsResponse>& msg) {
    auto outArray = factory.createStructArray({1,1},{"inbound_boxes"});
    // inbound_boxes
    outArray[0]["inbound_boxes"] = get_arr_pickplace_msgs_PickBox_var_arr(factory, msg->inbound_boxes);
    return std::move(outArray);
  }
} //namespace ListOfObjectsResponse
} //namespace matlabhelper
} //namespace pickplace_msgs
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1