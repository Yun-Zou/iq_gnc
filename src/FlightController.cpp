#include "FlightController.hpp"
#include "geometry_msgs/Quaternion.h"

bool FlightController::command_request(
    monash_main::RequestAction::Request &req,
    monash_main::RequestAction::Response &res) {

  if (accepting_commands) {

    // If flight command, move to position requested
    // param1 = x
    // param2 = y
    // param3 = z
    // param4 = psi
    if (req.command == (int)Flight) {
      absolute_move_WP(req.param1, req.param2, req.param3, req.param4);
    } 

    // If circle command, do circle command
    // param1 = x centre
    // param2 = y centre
    // param3 = radius
    // param4 = facing_direction as int
    else if (req.command == (int)Circle) {
      circular_WP(req.param1, req.param2, altitude, req.param3, (facing_direction)req.param4);
    }

    // Set flight mode
    state commandRequested = (state)req.command;
    set_flight_mode(commandRequested);
    res.success = true;

  } else {
    res.success = false;
    return false;
  }

  return true;
}

bool FlightController::request_apriltag_detection(bool request) {
  mavros_msgs::CommandBool request_srv;
  request_srv.request.value = request;

  // If not already requested/not requested
  if (request != requesting_apriltags) {
    requesting_apriltags = request;
    if (set_mode_client.call(request_srv)) {
      if (request) {
        ROS_INFO("Apriltags detection on");
      } else {
        ROS_INFO("Apriltags detection off");
      }
      return true;
    } else {
      ROS_ERROR("Failed Apriltags request");
      return false;
    }
  }
}

void FlightController::target_callback( const geometry_msgs::PoseStamped::ConstPtr &msg) {
  target_pose = *msg;
  // ROS_INFO("target_pose: %f %f %f", target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z);
  target_valid = true;
};

void FlightController::search_last_known(facing_direction direction) {

  circle_params params = {search_circle_radius, altitude, 0, 2 * M_PI, direction};

  geometry_msgs::Point search_origin;
  search_origin.x = target_pose.pose.position.x;
  search_origin.y = target_pose.pose.position.y;
  search_origin.z = altitude;

  flight_algorithm.set_circular_waypoint(waypointList, search_origin, params);
};

bool FlightController::set_flight_mode(state mode) {
  current_mission = mode;
  ROS_INFO("Setting flight mode: %s", state_string[mode].c_str());

  if (mode == Grounded) {
    clear_waypoints(counter);
    request_apriltag_detection(false);
    arm(false);

  } else if (mode == Hover) {
    clear_waypoints(counter);
    set_mode("Brake");
    request_apriltag_detection(false);

  } else if (mode == Flight) {
    set_mode("Guided");
    request_apriltag_detection(false);

  } else if (mode == Circle) {
    set_mode("Guided");
    clear_waypoints(counter);
    request_apriltag_detection(false);
    geometry_msgs::Point current_location = get_current_location();
    circle_params params = {circle_radius, altitude,0,2*M_PI,Forward};

    flight_algorithm.set_circular_waypoint(waypointList, current_location, params);


  } else if (mode == Search) {
    request_apriltag_detection(true);
    clear_waypoints(counter);
    grid_params params = {grid_length_x, grid_length_y, grid_spacing,
                          max_waypoint_dist};
    gnc_api_waypoint search_origin;
    search_origin.x = grid_initial_x;
    search_origin.y = grid_initial_y;
    search_origin.z = altitude;
    search_origin.psi = 0;

    flight_algorithm.set_search_grid(waypointList, search_origin, params);

  } else if (mode == Follow) {
    request_apriltag_detection(true);
    if (check_target_validity()) {
      clear_waypoints(counter);
      follow_target();

    } else {
      set_flight_mode(Flight);
      ROS_INFO("Can't set flight mode Follow. Setting flight mode to Flight");
      return false;
    }

  } else if (mode == RTL) {
    request_apriltag_detection(false);
    clear_waypoints(counter);
    flight_algorithm.return_home(waypointList, altitude);

  } else if (mode == Land) {
    request_apriltag_detection(false);
    clear_waypoints(counter);
    gnc_api_waypoint point;
    point.x = current_pos_local.x;
    point.y = current_pos_local.y;
    point.z = current_pos_local.z;
    point.psi = 0.0;
    waypointList.push_back(point);
    land();
    counter++;

  } else if (mode == TakeOff) {
    start_timer();
    clear_waypoints(counter);
    gnc_api_waypoint origin;
    origin.x = current_pos_local.x;
    origin.y = current_pos_local.y;
    origin.z = altitude;
    origin.psi = 0.0;
    waypointList.push_back(origin);
    takeoff(altitude);
    counter++;

  } else {
    ROS_INFO("Request didn't match a mode");
    return false;
  }

  return true;
};

void FlightController::clear_waypoints(int counter = 0) {

  // Clear waypoint from counter index to end
  waypointList.erase(std::next(waypointList.begin(), counter),
                     std::next(waypointList.begin(), waypointList.size()));

};

void FlightController::absolute_move(gnc_api_waypoint &absolute_position) {

  // Append waypoint of absolute position to list
  gnc_api_waypoint nextWayPoint = absolute_position;
  waypointList.push_back(nextWayPoint);

};

void FlightController::relative_move(gnc_api_waypoint &relative_position) {

  // Append waypoint for relative position compared to current location
  geometry_msgs::Point current_location = get_current_location();
  float current_heading = get_current_heading();

  gnc_api_waypoint nextWayPoint;
  nextWayPoint.x = current_location.x + relative_position.x;
  nextWayPoint.y = current_location.y + relative_position.y;
  nextWayPoint.z = current_location.z + relative_position.z;
  nextWayPoint.psi = current_heading + relative_position.psi;

  waypointList.push_back(nextWayPoint);
};

void FlightController::relative_move_WP(float x, float y, float z, float psi) {

  gnc_api_waypoint relative_position;
  relative_position.x = x;
  relative_position.y = y;
  relative_position.z = z;
  relative_position.psi = psi;

  relative_move(relative_position);
};

void FlightController::absolute_move_WP(float x, float y, float z, float psi) {

  gnc_api_waypoint absolute_position;
  absolute_position.x = x;
  absolute_position.y = y;
  absolute_position.z = z;
  absolute_position.psi = psi;

  absolute_move(absolute_position);
};

void FlightController::circular_WP(float x, float y, float z, float radius, facing_direction direction = Forward) {
  geometry_msgs::Point centre;
  centre.x = x;
  centre.y = y;
  centre.z = z;

  // Set circle movement parameters according to ones defined in config file
  // Do a full rotation starting at 0 degrees in circle arc
  circle_params params = {circle_radius, altitude, 0.0, 2 * M_PI, direction};

  flight_algorithm.set_circular_waypoint(waypointList, centre, params);

};

void FlightController::follow_target() {
  // Follow target according to parameters set in config
  follow_params params = {
      altitude,    follow_maintain_inner, follow_maintain_outer, follow_far,
      follow_time, max_waypoint_dist,     rapid_speed,           normal_speed};
  flight_algorithm.set_follow(waypointList, target_pose, params);

};

bool FlightController::check_target_validity() {
  
  // Find time difference between now and last stamp
  ros::Duration time_diff = ros::Time::now() - target_pose.header.stamp;

  // If time difference is less than search time set
  if (time_diff <= ros::Duration(search_lost_time)) {
    return true;
  } else {
    target_valid = false;
    return false;
  }
}

void FlightController::check_safety_conditions() {

  geometry_msgs::Point current_position = get_current_location();

  // Check flight time hasn't exceeded operation time set
  bool time_check = (ros::Time::now() - begin_time) > ros::Duration(operation_time);

  // Check position hasn't exceeded maximum radius set
  bool radius_check =
      pow(pow(current_position.x, 2) + pow(current_position.y, 2), 1 / 2) >
      max_radius;
  
  // Check if altitude hasn't exceeded maximum altitude set
  bool altitude_check = current_position.z > max_altitude;

  // If any conditions met, go home
  if (time_check || radius_check || altitude_check) {
    set_flight_mode(RTL);
  }

};

void FlightController::publish_waypoints() {

  visualization_msgs::Marker marker;
  geometry_msgs::Point point;

  // Set the frame ID and timestamp.  See the TF tutorials for information on
  // these.
  marker.header.frame_id = "/drone_local_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "waypoints";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the
  // frame/time specified in the header
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Append all waypoints in list
  for (int i = 0; i < counter; i++) {
    point.x = waypointList[i].x;
    point.y = waypointList[i].y;
    point.z = waypointList[i].z;

    marker.points.push_back(point);
  }

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = waypoint_radius;
  marker.scale.y = waypoint_radius;
  marker.scale.z = waypoint_radius;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.4f;
  marker.color.b = 0.0f;
  marker.color.a = 0.5f;

  // Only show for 2 seconds
  marker.lifetime = ros::Duration(2);

  waypoint_pub.publish(marker);
};

void FlightController::broadcast_local_frame() {

  // Set drone_local_frame relating to camera_odom_frame as the offset calculated in gnc_functions when calling initialize_local_frame
  transform_local.setOrigin(tf::Vector3(local_offset_pose_g.x, local_offset_pose_g.y, local_offset_pose_g.z));
  transform_camera_odom.setOrigin(tf::Vector3(0,0,0));
  
  tf::Quaternion q;
  // Set rotation of drone_local_frame as offset when calling initialize_local_frame
  q.setRPY(0, 0, (local_offset_g-90)/180 * M_PI);
  transform_local.setRotation(q);

  transform_camera_odom.setRotation(tf::Quaternion(0, 0, 0, 1));

  tf_br_camera.sendTransform(tf::StampedTransform(transform_camera_odom, ros::Time::now(), "map","camera_odom_frame"));
  tf_br_local.sendTransform(tf::StampedTransform(transform_local, ros::Time::now(), "camera_odom_frame","drone_local_frame"));
}

void FlightController::publish_pose() {
  visualization_msgs::Marker marker;
  geometry_msgs::Point point = get_current_location();
  float heading = (get_current_heading() * M_PI / 180) - 90;
  geometry_msgs::Quaternion quat_msg;

  // Set the frame ID and timestamp.  See the TF tutorials for information on
  // these.
  marker.header.frame_id = "/drone_local_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "pose";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the
  // frame/time specified in the header
  marker.pose.position = point;

  tf::Quaternion quat_tf;
  quat_tf.setRPY(0, 0, heading );

  quaternionTFToMsg(quat_tf, quat_msg);
  marker.pose.orientation = quat_msg;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.35;
  marker.scale.y = 0.3;
  marker.scale.z = 0.1;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  marker.lifetime = ros::Duration(2);

  // Publish drone position as cylinder
  geometry_msgs::PoseStamped poseStamped;
  poseStamped.header.frame_id = "/drone_local_frame";
  poseStamped.header.stamp = ros::Time::now();
  poseStamped.pose.position = point;
  poseStamped.pose.orientation = quat_msg;

  // Publish path to represent where drone has been
  pose_path.poses.push_back(poseStamped);
  pose_path.header.frame_id = "/drone_local_frame";
  pose_path.header.stamp = ros::Time::now();

  drone_pose_pub.publish(marker);
  drone_path_pub.publish(pose_path);

};

void FlightController::publish_flight_mode() {
  state mode = get_flight_mode();
  std_msgs::String msg;

  // Get state as string
  msg.data = state_string[mode];
  flight_mode_pub.publish(msg);
};

void FlightController::publish_topics() {
  publish_waypoints();
  publish_pose();
  publish_flight_mode();
  broadcast_local_frame();
}

void FlightController::init_params(ros::NodeHandle nh) {

  // General Flight Parameters
  nh.getParam("rapid_speed", rapid_speed);
  nh.getParam("normal_speed", normal_speed);
  nh.getParam("circle_radius", circle_radius);
  nh.getParam("altitude", altitude);
  nh.getParam("operation_time", operation_time);

  // Waypoint Parameters
  nh.getParam("max_waypoint_dist", max_waypoint_dist);
  nh.getParam("waypoint_radius", waypoint_radius);

  // Search Mode Parameters
  nh.getParam("grid_initial_x", grid_initial_x);
  nh.getParam("grid_initial_y", grid_initial_y);
  nh.getParam("grid_length_x", grid_length_x);
  nh.getParam("grid_length_y", grid_length_y);
  nh.getParam("grid_spacing", grid_spacing);
  nh.getParam("search_circle_radius", search_circle_radius);
  nh.getParam("search_lost_time", search_lost_time);

  // Follow Mode Parameters
  nh.getParam("follow_alt", follow_alt);
  nh.getParam("follow_maintain_inner", follow_maintain_inner);
  nh.getParam("follow_maintain_outer", follow_maintain_outer);
  nh.getParam("follow_far", follow_far);
  nh.getParam("follow_time", follow_time);

  // Safe Conditions
  nh.getParam("max_radius", max_radius);
  nh.getParam("max_altitude", max_altitude);
}

void FlightController::init_publisher_subscriber(ros::NodeHandle nh) {

  std::string ros_namespace;
  if (!nh.hasParam("namespace")) {
    ROS_INFO("using default namespace");
  } else {
    nh.getParam("namespace", ros_namespace);
    ROS_INFO("using namespace %s", ros_namespace.c_str());
  }

  waypoint_pub        = nh.advertise<visualization_msgs::Marker>((ros_namespace + "/monash_motion/waypoints").c_str(), 1);
  flight_mode_pub     = nh.advertise<std_msgs::String>(ros_namespace + "/monash_motion/flight_mode", 1);
  
  drone_pose_pub      = nh.advertise<visualization_msgs::Marker>((ros_namespace + "/monash_motion/pose").c_str(), 1);
  drone_path_pub      = nh.advertise<nav_msgs::Path>((ros_namespace + "/monash_motion/path").c_str(), 1);
  target_sub          = nh.subscribe<geometry_msgs::PoseStamped>((ros_namespace + "/monash_perception/target").c_str(), 5, &FlightController::target_callback, this);

  command_server      = nh.advertiseService((ros_namespace + "/monash_motion/request_command").c_str(), &FlightController::command_request, this);
  apriltag_client     = nh.serviceClient<mavros_msgs::CommandBool>((ros_namespace + "/monash_perception/detection_request"));
};
