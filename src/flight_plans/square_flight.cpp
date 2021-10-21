#include "../FlightController.cpp"
// include API



int main(int argc, char **argv) {
  // initialize ros
  ros::init(argc, argv, "square_flight");
  ros::NodeHandle square_flight("~");

  FlightController flight_controller(square_flight);
  
  // initialize control publisher/subscribers
  init_publisher_subscriber(square_flight);

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
  flight_controller.absolute_move_WP(1.0,0.0,1.0,-90);
  flight_controller.absolute_move_WP(1,1,1,0);
  flight_controller.absolute_move_WP(0,1,1,90);
  flight_controller.absolute_move_WP(0,0,1,180);
  flight_controller.absolute_move_WP(0,0,1,0);

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