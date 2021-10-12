#include <ros/ros.h>

#include "FlightAlgorithm.cpp"

#include "mavros_msgs/CommandInt.h"
#include "mavros_msgs/CommandBool.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"

#include "ros/duration.h"
#include "ros/node_handle.h"
#include <utility>


enum state { Grounded, Hover, Flight, Search, Follow, RTL, Land, TakeOff };

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
      ros::ServiceServer command_server;
      ros::ServiceClient apriltag_client;

      geometry_msgs::Pose target_pose;
      ros::Time target_last_seen;

      ros::Time begin_time;

      state current_mission = Grounded;
      bool accepting_commands = false;

      std::vector<gnc_api_waypoint> waypointList;

      /** @brief FlightAlgorithm instance */
      FlightAlgorithm flight_algorithm;

    public:

        int waypoint_counter = 0;

        bool command_request(mavros_msgs::CommandInt::Request &req, mavros_msgs::CommandInt::Response &res) {
            
        }

        bool request_apriltag_detection(bool request) {
            mavros_msgs::CommandBool request_srv;
            request_srv.request.value = request;

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

        bool target_callback(const geometry_msgs::PoseStamped &msg) {
          target_last_seen = ros::Time::now();
          target_pose = msg.pose;
        };

        bool set_flight_mode(state mode) {
          current_mission = mode;

          if (mode == Grounded) {

          } else if (mode == Hover) {

          } else if (mode == Flight) {

          } else if (mode == Search) {
              grid_params params = {grid_length_x, grid_length_y, grid_spacing, max_waypoint_dist};
              gnc_api_waypoint search_origin;
              search_origin.x = grid_initial_x;
              search_origin.y = grid_initial_y;
              search_origin.z = altitude;
              search_origin.psi = 0;
              
              flight_algorithm.set_search_grid(waypointList, search_origin, params);

          } else if (mode == Follow) {

            if (check_target_validity()) {
            //   flight_algorithm.set_follow();
            } else {
              // set_flight_mode()
            }

          } else if (mode == RTL) {
            // flight_algorithm.return_home(2.0);

          } else if (mode == Land) {

          } else if (mode == TakeOff) {

          } else {
          }
        };


        state get_flight_mode() { return current_mission; };

        void set_accepting_commands(bool state) { accepting_commands = state; };

        std::vector<gnc_api_waypoint> get_waypoints() { return waypointList; };

        void clear_waypoints(int counter = 0) {

        //   waypointList.erase(counter, waypointList.size()); 
        };

        void absolute_move(gnc_api_waypoint &absolute_position) {

          gnc_api_waypoint nextWayPoint = absolute_position;
          waypointList.push_back(nextWayPoint);
        };

        void relative_move(gnc_api_waypoint &relative_position) {

          geometry_msgs::Point current_location = get_current_location();
          float current_heading = get_current_heading();

          gnc_api_waypoint nextWayPoint;
          nextWayPoint.x = current_location.x + relative_position.x;
          nextWayPoint.y = current_location.y + relative_position.y;
          nextWayPoint.z = current_location.z + relative_position.z;
          nextWayPoint.psi = current_heading + relative_position.psi;

          waypointList.push_back(nextWayPoint);
        };

        void relative_move_WP(float x, float y, float z, float psi) {

          gnc_api_waypoint relative_position;
          relative_position.x = x;
          relative_position.y = y;
          relative_position.z = z;
          relative_position.psi = psi;

          relative_move(relative_position);
        };

        void absolute_move_WP(float x, float y, float z, float psi) {

          gnc_api_waypoint absolute_position;
          absolute_position.x = x;
          absolute_position.y = y;
          absolute_position.z = z;
          absolute_position.psi = psi;

          absolute_move(absolute_position);
        };

        void circular_WP(float x, float y, float z) {
          geometry_msgs::Point centre;
          centre.x = x;
          centre.y = y;
          centre.z = z;

          circle_params params = {circle_radius, altitude, 0.0 , 2*M_PI};

          flight_algorithm.set_circular_waypoint(waypointList, centre, params);
        };

        double get_rapid_speed() { return rapid_speed; }

        double get_normal_speed() { return normal_speed; }

        double get_normal_altitude() { return altitude; }

        double get_waypoint_radius() { return waypoint_radius; }

        void start_timer() { begin_time = ros::Time::now(); };

        bool check_target_validity() {
          ros::Duration time_diff = ros::Time::now() - target_last_seen;
          return !(time_diff > ros::Duration(search_lost_time));
        }

        void check_safety_conditions() {

          geometry_msgs::Point current_position = get_current_location();

          bool time_check =
              (ros::Time::now() - begin_time) > ros::Duration(operation_time);
          bool radius_check =
              pow(pow(current_position.x, 2) + pow(current_position.y, 2),
                  1 / 2) > max_radius;
          bool altitude_check = current_position.z > max_altitude;

          if (time_check || radius_check || altitude_check) {
            set_flight_mode(RTL);
          }
        };

        void publish_waypoints(){
            geometry_msgs::PoseArray waypoint_array;
            // waypoint_array.header.frame_id;
            // waypoint_pub.publish(waypoint_array);
        };

        void init() {
            this->init_params();
            this->init_publisher_subscriber();
        }

        void init_params() {

          ros::NodeHandle nh("~");

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

        void init_publisher_subscriber() {

          ros::NodeHandle nh("~");

          std::string ros_namespace;
          if (!nh.hasParam("namespace")) {
            ROS_INFO("using default namespace");
          } else {
            nh.getParam("namespace", ros_namespace);
            ROS_INFO("using namespace %s", ros_namespace.c_str());
          }
          
          waypoint_pub = nh.advertise<geometry_msgs::PoseArray>((ros_namespace + "/monash_motion/waypoints").c_str(), 1);
        //   target_sub = nh.subscribe<geometry_msgs::PoseStamped>((ros_namespace + "/monash_perception/target").c_str(), 5, &FlightController::target_callback);

        //   command_server = nh.advertiseService("/monash_motion/request_command", &FlightController::command_request);
          apriltag_client = nh.serviceClient<mavros_msgs::CommandBool>((ros_namespace + "/monash_perception/detection_request"));

        };
};
