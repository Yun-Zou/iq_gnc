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

enum state { Grounded, Hover, Flight, Circle, Search, Follow, RTL, Land, TakeOff };
std::vector<std::string> state_string = {"Grounded", "Hover", "Flight", "Circle", "Search", "Follow", "RTL", "Land", "TakeOff"};

class FlightController {
protected:

  // General Flight Parameters
  double rapid_speed;
  double normal_speed;
  double altitude;
  double circle_radius;
  double operation_time;

  // Waypoint Parameters
  double max_waypoint_dist;
  double waypoint_radius;

  // Search Mode Parameters
  double grid_initial_x;
  double grid_initial_y;
  double grid_length_x;
  double grid_length_y;
  double grid_spacing;
  double search_circle_radius;
  double search_lost_time;

  // Follow Mode Parameters
  double follow_alt;
  double follow_maintain_inner;
  double follow_maintain_outer;
  double follow_far;
  double follow_time;

  // Safe Conditions
  double max_radius;
  double max_altitude;

  ros::Subscriber target_sub;
  ros::Publisher waypoint_pub;
  ros::Publisher flight_mode_pub;
  ros::ServiceServer command_server;
  ros::ServiceClient apriltag_client;

  geometry_msgs::PoseStamped target_pose;
  bool target_valid = false;
  ros::Duration followed_time = ros::Duration(0);

  ros::Time begin_time;

  state current_mission = Grounded;
  bool accepting_commands = false;
  bool requesting_apriltags = false;

  std::vector<gnc_api_waypoint> waypointList;

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

  void publish_flight_mode();

  void publish_topics();

  void init(ros::NodeHandle nh);

  void init_params(ros::NodeHandle nh);

  void init_publisher_subscriber(ros::NodeHandle nh);
};
