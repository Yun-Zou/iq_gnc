#include "../FlightController.cpp"
// include API



int main(int argc, char **argv) {
  // initialize ros
  ros::init(argc, argv, "travel_and_return");
  ros::NodeHandle travel_and_return("~");

  FlightController flight_controller(travel_and_return);

  // initialize control publisher/subscribers
  init_publisher_subscriber(travel_and_return);

  // wait for FCU connection
  wait4connect();

  // wait for used to switch to mode GUIDED
  wait4start();

  // create local reference frame
  initialize_local_frame();

  set_speed(flight_controller.get_normal_speed());

  // request takeoff
  double alt = flight_controller.get_normal_altitude();
  flight_controller.set_flight_mode(TakeOff);

  // specify some waypoints
  flight_controller.absolute_move_WP(0, 0, alt, 0);
  flight_controller.absolute_move_WP(2, 0, alt, -90);
  flight_controller.absolute_move_WP(2, 5, alt, 0);
  flight_controller.absolute_move_WP(-3, 5, alt, 90);
  flight_controller.circular_WP(-3,5,alt);
  flight_controller.absolute_move_WP(0, 0, alt, 180);

  std::vector<gnc_api_waypoint> waypointList = flight_controller.get_waypoints();
  
  flight_controller.set_flight_mode(Flight);
  state current_mode = flight_controller.get_flight_mode();

  // specify control loop rate. We recommend a low frequency to not over load
  // the FCU with messages. Too many messages will cause the drone to be
  // sluggish
  ros::Rate rate(2.0);

  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();

    waypointList = flight_controller.get_waypoints();
    current_mode = flight_controller.get_flight_mode();

    if (check_waypoint_reached(flight_controller.get_waypoint_radius()) == 1) {

      int counter = flight_controller.counter;

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
  }
  return 0;
}