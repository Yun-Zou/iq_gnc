#include <ros/ros.h>

#include "FlightAlgorithm.cpp"

#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/CommandBool.h"
#include "monash_main/RequestAction.h"
#include "nav_msgs/Path.h"
#include "ros/duration.h"
#include "ros/node_handle.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <string>
#include <utility>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

enum state { Grounded, Hover, Flight, Circle, Search, Follow, RTL, Land, TakeOff };
std::vector<std::string> state_string = {"Grounded", "Hover", "Flight", "Circle", "Search", "Follow", "RTL", "Land", "TakeOff"};

class FlightController {
protected:

  // General Flight Parameters
  double rapid_speed = 1.0;
  double normal_speed = 0.5;
  double altitude = 2.0;
  double circle_radius = 2.0;
  double operation_time = 120;

  // Waypoint Parameters
  double max_waypoint_dist = 1.0;
  double waypoint_radius = 0.3;

  // Search Mode Parameters
  double grid_initial_x = 2;
  double grid_initial_y = 2;
  double grid_length_x = 3;
  double grid_length_y = 3;
  double grid_spacing = 1;
  double search_circle_radius = 2.5;
  double search_lost_time = 5;

  // Follow Mode Parameters
  double follow_alt = 2;
  double follow_maintain_inner = 0.5;
  double follow_maintain_outer = 1.5;
  double follow_far = 5;
  double follow_time = 30;

  // Safe Conditions
  double max_radius = 10;
  double max_altitude = 10;

  ros::Subscriber target_sub;
  ros::Publisher waypoint_pub;
  ros::Publisher flight_mode_pub;
  ros::Publisher drone_path_pub;
  ros::Publisher drone_pose_pub;
  ros::ServiceServer command_server;
  ros::ServiceClient apriltag_client;

  nav_msgs::Path pose_path;

  geometry_msgs::PoseStamped target_pose;
  bool target_valid = false;
  ros::Duration followed_time = ros::Duration(0);

  ros::Time begin_time;

  state current_mission = Grounded;
  bool accepting_commands = false;
  bool requesting_apriltags = false;

  std::vector<gnc_api_waypoint> waypointList;

  tf::TransformBroadcaster tf_br_local;
  tf::TransformBroadcaster tf_br_camera;
  tf::Transform transform_local;
  tf::Transform transform_camera_odom;

  /** @brief FlightAlgorithm instance */
  FlightAlgorithm flight_algorithm;

public:
  FlightController(ros::NodeHandle nh) { init(nh); };

  int counter = 0;

  bool command_request(monash_main::RequestAction::Request &req,
                       monash_main::RequestAction::Response &res);

  bool request_apriltag_detection(bool request);

  void target_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);

  void search_last_known(facing_direction direction);

  bool set_flight_mode(state mode);

  state get_flight_mode() { return current_mission; };

  void set_accepting_commands(bool state) { accepting_commands = state; };

  std::vector<gnc_api_waypoint> get_waypoints() { return waypointList; };

  void clear_waypoints(int counter);

  void absolute_move(gnc_api_waypoint &absolute_position);

  void relative_move(gnc_api_waypoint &relative_position);

  void relative_move_WP(float x, float y, float z, float psi);

  void absolute_move_WP(float x, float y, float z, float psi);

  void circular_WP(float x, float y, float z, float radius, facing_direction direction);

  void follow_target();

  void search_circle(facing_direction direction);

  double get_rapid_speed() { return rapid_speed; }

  double get_normal_speed() { return normal_speed; }

  double get_normal_altitude() { return altitude; }

  double get_waypoint_radius() { return waypoint_radius; }

  void start_timer() { begin_time = ros::Time::now(); };

  bool check_target_validity();

  void check_safety_conditions();
  void publish_waypoints();

  void broadcast_local_frame();

  void publish_pose();

  void publish_flight_mode();

  void publish_topics();

  void init(ros::NodeHandle nh);

  void init_params(ros::NodeHandle nh);

  void init_publisher_subscriber(ros::NodeHandle nh);
};
