#include "../FlightController.cpp"
// include API

int main(int argc, char **argv) {
  // initialize ros
  ros::init(argc, argv, "partner");
  ros::NodeHandle partner("~");

  FlightController flight_controller(partner);

  // initialize control publisher/subscribers
  init_publisher_subscriber(partner);

  // wait for FCU connection
  wait4connect();

  // wait for used to switch to mode GUIDED
  wait4start();

  // create local reference frame
  initialize_local_frame();

  set_speed(flight_controller.get_normal_speed());

  // request takeoff
  flight_controller.set_flight_mode(TakeOff);

  flight_controller.set_accepting_commands(true);

  // specify some waypoints
  // flight_controller.absolute_move_WP(0,0,2,0);

  std::vector<gnc_api_waypoint> waypointList = flight_controller.get_waypoints();

  flight_controller.set_flight_mode(Flight);
  state current_mode = flight_controller.get_flight_mode();

  // specify control loop rate. We recommend a low frequency to not over load
  // the FCU with messages. Too many messages will cause the drone to be
  // sluggish
  ros::Rate rate(2.0);
  int counter = flight_controller.counter;

  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();

    waypointList = flight_controller.get_waypoints();
    current_mode = flight_controller.get_flight_mode();

    if (check_waypoint_reached(flight_controller.get_waypoint_radius()) == 1) {

      int counter = flight_controller.counter;

      if (current_mode == Follow) {
        if (flight_controller.check_target_validity()) {
          flight_controller.follow_target();
        } else {
          flight_controller.set_flight_mode(Flight);
          ROS_INFO( "Can't set flight mode Follow. Setting flight mode to Flight");
        }
      }
      
      // Land if UAV has reached home position
      if (current_mode == Land) {
        flight_controller.set_flight_mode(Land);
        flight_controller.set_accepting_commands(false);
      }

      // Go to each waypoint
      else if (counter < waypointList.size()) {
        set_destination(waypointList[counter].x, waypointList[counter].y,
                        waypointList[counter].z, waypointList[counter].psi);

        flight_controller.counter++;
      }
    }

    flight_controller.publish_topics();
  }
  return 0;
}