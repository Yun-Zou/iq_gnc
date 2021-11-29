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

  // Default flight parameters. When using launch file, read the params from cfg file.

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

  // Subscribers, publishers and services
  ros::Subscriber target_sub;
  ros::Publisher waypoint_pub;
  ros::Publisher flight_mode_pub;
  ros::Publisher drone_path_pub;
  ros::Publisher drone_pose_pub;
  ros::ServiceServer command_server;
  ros::ServiceClient apriltag_client;

  // Array poses the drone has been had during flight
  nav_msgs::Path pose_path;

  // Target Properties
  geometry_msgs::PoseStamped target_pose;
  bool target_valid = false;
  ros::Duration followed_time = ros::Duration(0);

  // Mission start time
  ros::Time begin_time;

  // Mission Properties
  state current_mission = Grounded; // Current flight mmode
  bool accepting_commands = false; // Accepting external commands?
  bool requesting_apriltags = false; // Is apriltags detection on?

  // Vector of waypoints in gnc_api_waypoint (gnc_functions.hpp) form which is read in flight plans
  std::vector<gnc_api_waypoint> waypointList;

  // Transforms 
  tf::TransformBroadcaster tf_br_local;
  tf::TransformBroadcaster tf_br_camera;
  tf::Transform transform_local;
  tf::Transform transform_camera_odom;

  /** @brief FlightAlgorithm instance */
  FlightAlgorithm flight_algorithm;

public:

  /**
   * @brief Construct a new Perception Controller object. Initialise params and
   * publishers/subscribers
   *
   * @param nh ROS NodeHandle
   */
  FlightController(ros::NodeHandle nh) { 
    init_params(nh);
    init_publisher_subscriber(nh);
  };

  // Waypoint counter. Which waypoint are we currently heading to.
  int counter = 0;

  /**
   * @brief Service Request server. Accepts commands sets flight mode if possible
   * 
   * @param req Accepts index of enum state corresponding to flight mode. Params used for coordinates and other settings
   * @param res Result success or failure
   * @return true 
   * @return false 
   */
  bool command_request(monash_main::RequestAction::Request &req,
                       monash_main::RequestAction::Response &res);

  /**
   * @brief Make a request apriltag detection service call to monash_perception
   * 
   * @param request true or false
   * @return true Success
   * @return false Failure
   */
  bool request_apriltag_detection(bool request);

  /**
   * @brief Get target pose published from monash_perception
   * 
   * @param msg Pose of target
   */
  void target_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);

  /**
   * @brief Perform a circle manuveur above last seen position of target
   * 
   * @param direction direction to face based on enum facing_direction from FlightAlgorithm.cpp
   */
  void search_last_known(facing_direction direction);

  /**
   * @brief Set the current flight mode state if possible
   * 
   * @param mode 
   * @return true 
   * @return false 
   */
  bool set_flight_mode(state mode);

  /**
   * @brief Get the flight_mode state
   * 
   * @return state 
   */
  state get_flight_mode() { return current_mission; };

  /**
   * @brief Set whether drone is accepting external commands
   * 
   * @param state 
   */
  void set_accepting_commands(bool state) { accepting_commands = state; };

  /**
   * @brief Get the vector of waypoints
   * 
   * @return std::vector<gnc_api_waypoint>
   */
  std::vector<gnc_api_waypoint> get_waypoints() { return waypointList; };

  /**
   * @brief Clear the waypoints list from specified index onwards
   * 
   * @param counter Index of vector to clear from
   */
  void clear_waypoints(int counter);

  /**
   * @brief Append a waypoint to list to travel to absolute position in local frame.
   * 
   * @param absolute_position 
   */
  void absolute_move(gnc_api_waypoint &absolute_position);

  /**
   * @brief Alternative absolute move function. Accepts x,y,z and heading (psi) floats as params
   * 
   * @param x (m) in local frame
   * @param y (m) in local frame
   * @param z (m)
   * @param psi degrees
   */
  void absolute_move_WP(float x, float y, float z, float psi);

  /**
   * @brief Append a waypoint to list which is relative to current position. Note: It would be current position, not position of final waypoint
   *        Note: Not very reliable. Still need to be improved
   * @param relative_position Waypoint with relative coordinates (x,y,z and heading (degrees))
   */
  void relative_move(gnc_api_waypoint &relative_position);

  /**
   * @brief Alternative relative move function. Accepts x,y,z and heading (psi) floats as params 
   *        Note: Not very reliable. Still need to be improved
   * @param x (m) in local frame
   * @param y (m) in local frame
   * @param z (m)
   * @param psi degrees
   */
  void relative_move_WP(float x, float y, float z, float psi);

  /**
   * @brief Fly in a circle
   *
   * @param x coordinates of centre point in local frame
   * @param y coordinates of centre point in local frame
   * @param z altitude to fly at
   * @param radius (m) of circle
   * @param direction the drone faces during circular move: Forwards, Inwards, Outwards
   */
  void circular_WP(float x, float y, float z, float radius, facing_direction direction);

  /**
   * @brief Append a waypoint which keeps the drone following the target published by monash_perception.
            Will try to maintain within a certain inner and outer radius from the drone specified in config file.
   * 
   */
  void follow_target();

  /**
   * @brief Get the 'fast' flight speed (m/s) value. How fast should the drone fly when travelling long distances
   * 
   * @return double 
   */
  double get_rapid_speed() { return rapid_speed; }

  /**
   * @brief Get the regular flight speed (m/s) value. How fast should the drone fly in regular flight
   * 
   * @return double 
   */
  double get_normal_speed() { return normal_speed; }

  /**
   * @brief Get the regular flight altitude (m) value. How high should the drone fly in regular flight
   * 
   * @return double 
   */
  double get_normal_altitude() { return altitude; }

  /**
   * @brief Get the waypoint radius (m) value. Distance from waypoint point where drone can be considered reached.
   * 
   * @return double
   */
  double get_waypoint_radius() { return waypoint_radius; }

  /**
   * @brief Set beginning time as current time
   * 
   */
  void start_timer() { begin_time = ros::Time::now(); };

  /**
   * @brief Check if target has been seen in search_lost_time amount of time
   *
   * @return true
   * @return false
   */
  bool check_target_validity();

  /**
   * @brief Check if drone has exceeded maximum radius, height or flight time. If so, go home
   *        Note: Don't think it works. Need to improved
   * 
   */
  void check_safety_conditions();

  /**
   * @brief Publish all visited and current waypoints as rviz markers
   * 
   */
  void publish_waypoints();

  /**
   * @brief Publish tf transform between map <-> local_drone_frame (takeoff position), and local_drone_frame <-> camera_odom_frame (camera initial position)
   * 
   */
  void broadcast_local_frame();

  /**
   * @brief Publish current pose as a rviz marker
   * 
   */
  void publish_pose();

  /**
   * @brief Publish current flight mode as string
   * 
   */
  void publish_flight_mode();

  /**
   * @brief Publish pose data, flight mode status and rviz markers
   * 
   */
  void publish_topics();

  /**
   * @brief Read params from /cfg yaml file
   * 
   * @param nh ROS NodeHandle
   */
  void init_params(ros::NodeHandle nh);

  /**
   * @brief Subscribe and advertise topics
   * 
   * @param nh ROS NodeHandle
   */
  void init_publisher_subscriber(ros::NodeHandle nh);
};
