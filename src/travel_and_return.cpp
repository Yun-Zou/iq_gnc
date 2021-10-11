#include "FlightAlgorithm.cpp"
// include API

static FlightAlgorithm flight_algorithm;

int main(int argc, char **argv) {
  // initialize ros
  ros::init(argc, argv, "travel_and_return");
  ros::NodeHandle travel_and_return("~");

  // initialize control publisher/subscribers
  init_publisher_subscriber(travel_and_return);
  flight_algorithm.init_params(travel_and_return);

  // wait for FCU connection
  wait4connect();

  // wait for used to switch to mode GUIDED
  wait4start();

  // create local reference frame
  initialize_local_frame();

  
  // request takeoff
  takeoff(flight_algorithm.get_normal_altitude());

  set_speed(flight_algorithm.get_normal_speed());
  flight_algorithm.set_flight_mode(Flight);

  // specify some waypoints
  gnc_api_waypoint absolute_move;

  absolute_move.x = 0;
  absolute_move.y = 0;
  absolute_move.z = flight_algorithm.get_normal_altitude();
  absolute_move.psi = 0;
  flight_algorithm.absolute_move(absolute_move);

  absolute_move.x = 2;
  absolute_move.y = 0;
  absolute_move.z = flight_algorithm.get_normal_altitude();
  absolute_move.psi = -90;
  flight_algorithm.absolute_move(absolute_move);

  absolute_move.x = 2;
  absolute_move.y = 5;
  absolute_move.z = flight_algorithm.get_normal_altitude();
  absolute_move.psi = 0;
  flight_algorithm.absolute_move(absolute_move);

  absolute_move.x = -3;
  absolute_move.y = 5;
  absolute_move.z = flight_algorithm.get_normal_altitude();
  absolute_move.psi = 90;
  flight_algorithm.absolute_move(absolute_move);

  geometry_msgs::Point centre;
  centre.x = absolute_move.x;
  centre.y = absolute_move.y;
  centre.z = absolute_move.z;
  flight_algorithm.setCircularWaypoint(centre, 2, 1);

  absolute_move.x = 0;
  absolute_move.y = 0;
  absolute_move.z = flight_algorithm.get_normal_altitude();
  absolute_move.psi = 180;
  flight_algorithm.absolute_move(absolute_move);

  std::vector<gnc_api_waypoint> waypointList = flight_algorithm.getWayponts();

  // specify control loop rate. We recommend a low frequency to not over load
  // the FCU with messages. Too many messages will cause the drone to be
  // sluggish
  ros::Rate rate(2.0);
  int counter = 0;
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
    if (check_waypoint_reached(flight_algorithm.get_waypoint_radius()) == 1) {
      if (counter < waypointList.size()) {
        set_destination(waypointList[counter].x, waypointList[counter].y,
                        waypointList[counter].z, waypointList[counter].psi);
        counter++;
      } else {
        // land after all waypoints are reached
        flight_algorithm.set_flight_mode(RTL);
        set_destination(0, 0, flight_algorithm.get_normal_altitude(), 0);

        land();
        flight_algorithm.set_flight_mode(Grounded);
      }
    }
  }
  return 0;
}