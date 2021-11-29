#include "../FlightController.cpp"
// include API

int main(int argc, char **argv) {
  // initialize ros
  ros::init(argc, argv, "example");
  ros::NodeHandle example("~");

  FlightController flight_controller(example);

  // initialize control publisher/subscribers
  init_publisher_subscriber(example);

  // wait for FCU connection
  wait4connect();

  // wait for used to switch to mode GUIDED
  wait4start();

  // create local reference frame
  initialize_local_frame();

  // Set speed to normal speed defined. May not work. Change in Mission Planner settings
  set_speed(flight_controller.get_normal_speed());

  // request takeoff
  flight_controller.set_flight_mode(TakeOff);

  // specify some waypoints
  flight_controller.absolute_move_WP(2, 0.0, 2, -90);
  flight_controller.absolute_move_WP(2, 2, 2, 0);
  flight_controller.absolute_move_WP(0, 2, 2, 90);
  flight_controller.absolute_move_WP(0, 0, 2, 180);
  flight_controller.absolute_move_WP(0, 0, 2, 0);
  flight_controller.circular_WP(0, 0, 2, 3);

  std::vector<gnc_api_waypoint> waypointList = flight_controller.get_waypoints();

  // Set flight mode to flight
  flight_controller.set_flight_mode(Flight);
  state current_mode = flight_controller.get_flight_mode();

  // specify control loop rate. We recommend a low frequency to not over load
  // the FCU with messages. Too many messages will cause the drone to be
  // sluggish
  ros::Rate rate(2.0);

  // Get current waypoint index we are travelling to 
  int counter = flight_controller.counter;

  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();

    waypointList = flight_controller.get_waypoints();
    current_mode = flight_controller.get_flight_mode();

    // If drone has reached the waypoint within the radius set
    if (check_waypoint_reached(flight_controller.get_waypoint_radius()) == 1) {

      int counter = flight_controller.counter;

      // Land if UAV is set to go home or is land mode, land
      if (current_mode == RTL || current_mode == Land) {
        flight_controller.set_flight_mode(Land);

      }

      // While not all waypoints reached, set drone to head to waypoint
      else if (counter < waypointList.size()) {
        set_destination(waypointList[counter].x, waypointList[counter].y,
                        waypointList[counter].z, waypointList[counter].psi);

        // Increment counter
        flight_controller.counter++;

      }

      // Go home if no remaining commands
      else {
        // return home after all waypoints are reached
        flight_controller.set_flight_mode(RTL);
      }
    }

    // Publish all topics
    flight_controller.publish_topics();
  }
  return 0;
}