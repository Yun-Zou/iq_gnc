#include "../FlightController.cpp"
// include API

// search_follow.cpp
// Perform a search grid and then follow the target if found

int main(int argc, char **argv) {
  // initialize ros
  ros::init(argc, argv, "search_follow");
  ros::NodeHandle search_follow("~");

  FlightController flight_controller(search_follow);

  // initialize control publisher/subscribers
  init_publisher_subscriber(search_follow);

  // wait for FCU connection
  wait4connect();

  // wait for used to switch to mode GUIDED
  wait4start();

  // create local reference frame
  initialize_local_frame();

  set_speed(flight_controller.get_normal_speed());

  // request takeoff
  flight_controller.set_flight_mode(TakeOff);

  // specify some waypoints
  // flight_controller.absolute_move_WP(0,0,2,0);

  flight_controller.set_flight_mode(Search);

  std::vector<gnc_api_waypoint> waypointList = flight_controller.get_waypoints();
  state current_mode = flight_controller.get_flight_mode();

  // specify control loop rate. We recommend a low frequency to not over load
  // the FCU with messages. Too many messages will cause the drone to be
  // sluggish
  ros::Rate rate(2.0);
  int counter = flight_controller.counter;

  bool follow_searching = false;

  ros::Time follow_time;
  ros::Duration follow_duration = ros::Duration(30);

  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();

    waypointList = flight_controller.get_waypoints();
    current_mode = flight_controller.get_flight_mode();

    if ((current_mode == Flight && follow_searching) || (current_mode == Search &&
        flight_controller.check_target_validity())) {
      flight_controller.set_flight_mode(Follow);
      follow_time = ros::Time::now();

    }

    if (check_waypoint_reached(flight_controller.get_waypoint_radius()) == 1) {

      int counter = flight_controller.counter;

      if (current_mode == Follow) {
        if (flight_controller.check_target_validity()) {
          if ((ros::Time::now() - follow_time) < follow_duration) {
            flight_controller.follow_target();
            follow_searching = false;
          } else {
            flight_controller.set_flight_mode(RTL);
            continue;
          }
        } else {
          flight_controller.set_flight_mode(Flight);
          flight_controller.search_last_known(Inside);
          follow_searching = true;
          ROS_INFO("Can't set flight mode Follow. Setting flight mode to Flight");
        }
      }

      ROS_INFO("counter %f size %f",counter, waypointList.size());

      // Land if UAV has reached home position
      if (current_mode == RTL || current_mode == Land) {
        flight_controller.set_flight_mode(Land);

      }

      // Go to each waypoint
      else if (counter < waypointList.size()) {
        set_destination(waypointList[counter].x, waypointList[counter].y,
                        waypointList[counter].z, waypointList[counter].psi);

        flight_controller.counter++;

      } 
      
      // Go home if no remaining commands
      else {
        // return home after all waypoints are reached
        flight_controller.set_flight_mode(RTL);
      }
    }

    flight_controller.publish_topics();
  }
  return 0;
}