/********************************************************************************
 * Copyright (C) 2024-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Marko Mizdrak
 *    Mikkel Skov Maarssø
 *    Sanath Himasekhar Konthala
 ********************************************************************************/

#include "decision_maker.hpp"

namespace adore
{

DecisionMaker::DecisionMaker() :
  Node( "decision_maker" )
{
  load_parameters();
  create_subscribers();
  create_publishers();
  print_init_info();
}

void
DecisionMaker::create_publishers()
{
  publisher_trajectory            = create_publisher<adore_ros2_msgs::msg::Trajectory>( "trajectory_decision", 10 );
  publisher_trajectory_suggestion = create_publisher<std_msgs::msg::String>( "/MAV/control/RO/path_suggestion", 10 );
  publisher_request_assistance_remote_operations
    = create_publisher<std_msgs::msg::String>( "/MAV/state/modes", 10 ); // Actuall topic name is /MAV/0/states/modes, but ROS doesn´t allow
                                                                         // the 0, so the topic change is done in the mqtt bridge
  publisher_position_remote_operation = create_publisher<std_msgs::msg::String>( "/MAV/state/position", 10 );
}

void
DecisionMaker::create_subscribers()
{
  // Subscriber for route information
  subscriber_route = create_subscription<adore_ros2_msgs::msg::Route>( "route", 1,
                                                                       std::bind( &DecisionMaker::route_callback, this,
                                                                                  std::placeholders::_1 ) );

  subscriber_vehicle_state = create_subscription<adore_ros2_msgs::msg::VehicleStateDynamic>(
    "vehicle_state/dynamic", 1, std::bind( &DecisionMaker::vehicle_state_callback, this, std::placeholders::_1 ) );

  // Subscriber for local map updates
  subscriber_local_map = create_subscription<adore_ros2_msgs::msg::Map>( "local_map", 1,
                                                                         std::bind( &DecisionMaker::local_map_callback, this,
                                                                                    std::placeholders::_1 ) );

  // Subscriber for traffic participants
  subscriber_traffic_participants = create_subscription<adore_ros2_msgs::msg::TrafficParticipantSet>(
    "traffic_participants", 1, std::bind( &DecisionMaker::traffic_participants_callback, this, std::placeholders::_1 ) );

  // Subscribers for safety corridor (either denm or json will be received)
  subscriber_safety_corridor = create_subscription<adore_ros2_msgs::msg::SafetyCorridor>(
    "safety_corridor", 1, std::bind( &DecisionMaker::safety_corridor_callback, this, std::placeholders::_1 ) );


  // Subscriber for state monitor
  subscriber_state_monitor = create_subscription<adore_ros2_msgs::msg::StateMonitor>( "vehicle_state/monitor", 1,
                                                                                      std::bind( &DecisionMaker::state_monitor_callback,
                                                                                                 this, std::placeholders::_1 ) );

  subscriber_waypoints = create_subscription<std_msgs::msg::String>( "/MAV/control/waypoint_suggestion", 1,
                                                                     std::bind( &DecisionMaker::waypoints_callback, this,
                                                                                std::placeholders::_1 ) );

  subscriber_traffic_participants = create_subscription<adore_ros2_msgs::msg::TrafficParticipantSet>(
    "traffic_participants", 1, std::bind( &DecisionMaker::traffic_participants_callback, this, std::placeholders::_1 ) );

  subscriber_reference_trajectory = this->create_subscription<adore_ros2_msgs::msg::Trajectory>(
    "planned_trajectory", 1, std::bind( &DecisionMaker::reference_trajectory_callback, this, std::placeholders::_1 ) );

  subscriber_traffic_signals = create_subscription<adore_ros2_msgs::msg::TrafficSignals>(
    "traffic_signals", 1, std::bind( &DecisionMaker::traffic_signals_callback, this, std::placeholders::_1 ) );

  main_timer = create_wall_timer( std::chrono::milliseconds( static_cast<size_t>( 1000 * dt ) ), std::bind( &DecisionMaker::run, this ) );
}

void
DecisionMaker::load_parameters()
{
  declare_parameter( "debug_mode_active", true );
  get_parameter( "debug_mode_active", debug_mode_active );

  declare_parameter( "use_reference_trajectory_as_is", true );
  get_parameter( "use_reference_trajectory_as_is", default_use_reference_trajectory_as_is );

  declare_parameter( "optinlc_route_following", false );
  get_parameter( "optinlc_route_following", use_opti_nlc_route_following );

  declare_parameter( "dt", 0.05 );
  get_parameter( "dt", dt );

  declare_parameter( "remote_operation_speed", 2.0 );
  get_parameter( "remote_operation_speed", remote_operation_speed );

  declare_parameter( "max_acceleration", 2.0 );
  declare_parameter( "min_acceleration", -2.0 );
  declare_parameter( "max_steering", 0.7 );
  get_parameter( "max_acceleration", command_limits.max_acceleration );
  get_parameter( "min_acceleration", command_limits.min_acceleration );
  get_parameter( "max_steering", command_limits.max_steering_angle );

  // Planner related parameters
  std::vector<std::string> keys;
  std::vector<double>      values;
  declare_parameter( "planner_settings_keys", keys );
  declare_parameter( "planner_settings_values", values );
  get_parameter( "planner_settings_keys", keys );
  get_parameter( "planner_settings_values", values );

  if( keys.size() != values.size() )
  {
    RCLCPP_ERROR( this->get_logger(), "planner settings keys and values size mismatch!" );
    return;
  }
  for( size_t i = 0; i < keys.size(); ++i )
  {
    planner_settings.insert( { keys[i], values[i] } );
    std::cerr << "keys: " << keys[i] << ": " << values[i] << std::endl;
  }

  lane_follow_planner.set_parameters( planner_settings );
  opti_nlc_trajectory_planner.set_parameters( planner_settings );
}

void
DecisionMaker::run()
{
  update_state();
  switch( state )
  {
    case FOLLOW_REFERENCE:
      follow_reference();
      break;
    case FOLLOW_ROUTE:
      follow_route();
      break;
    case STANDSTILL:
      standstill();
      break;
    case REMOTE_OPERATION:
      remote_operation();
      break;
    case SAFETY_CORRIDOR:
      safety_corridor();
      break;
    case REQUESTING_ASSISTANCE:
      request_assistance();
      break;
    default:
      emergency_stop();
      break;
  }

  if( debug_mode_active )
  {
    print_debug_info();
  }
}

void
DecisionMaker::update_state()
{

  int current_conditions = 0;
  if( gps_fix_standard_deviation < 0.2 && latest_vehicle_state )
    current_conditions |= VEHICLE_STATE_OK;
  if( latest_safety_corridor )
    current_conditions |= SAFETY_CORRIDOR_PRESENT;
  if( !latest_waypoints.empty() )
    current_conditions |= WAYPOINTS_AVAILABLE;
  if( latest_trajectory_valid() )
    current_conditions |= REFERENCE_TRAJECTORY_VALID;
  if( latest_route )
    current_conditions |= ROUTE_AVAILABLE;
  if( latest_local_map )
    current_conditions |= LOCAL_MAP_AVAILABLE;
  if( need_assistance )
    current_conditions |= NEED_ASSISTANCE;

  // Loop through the constexpr array in order of priority
  for( const auto& state_requirement : state_priority )
  {
    if( ( current_conditions & state_requirement.required_conditions ) == state_requirement.required_conditions )
    {
      state = state_requirement.state;
      return;
    }
  }
}

void
DecisionMaker::emergency_stop()
{
  dynamics::Trajectory emergency_stop_trajectory;
  emergency_stop_trajectory.label = "Emergency Stop";
  publisher_trajectory->publish( dynamics::conversions::to_ros_msg( emergency_stop_trajectory ) );
}

void
DecisionMaker::standstill()
{
  dynamics::Trajectory standstill_trajectory;
  standstill_trajectory.label = "Standstill";
  publisher_trajectory->publish( dynamics::conversions::to_ros_msg( standstill_trajectory ) );
}

void
DecisionMaker::request_assistance()
{
  dynamics::Trajectory standstill_trajectory;
  standstill_trajectory.label = "Requesting Assistane";
  publisher_trajectory->publish( dynamics::conversions::to_ros_msg( standstill_trajectory ) );
  std_msgs::msg::String assistance_request_message;
  assistance_request_message.data = "{\"current_driving_mode\":1,\"current_control_mode\": 1, \"general_system_state\":2, \"timestamp\":"
                                  + std::to_string( (int) ( now().seconds() ) ) + "}"; // change this
  publisher_request_assistance_remote_operations->publish( assistance_request_message );
}

void
DecisionMaker::remote_operation()
{
  dynamics::Trajectory trajectory = planner::waypoints_to_trajectory( *latest_vehicle_state, latest_waypoints, dt, remote_operation_speed,
                                                                      command_limits );
  trajectory.label                = "Remote Operation";
  publisher_trajectory->publish( dynamics::conversions::to_ros_msg( trajectory ) );
}

void
DecisionMaker::follow_reference()
{
  dynamics::Trajectory planned_trajectory;
  if( !default_use_reference_trajectory_as_is )
  {
    planner::OptiNLCTrajectoryOptimizer planner;
    planned_trajectory = planner.plan_trajectory( *latest_reference_trajectory, *latest_vehicle_state );
  }
  else
  {
    planned_trajectory = *latest_reference_trajectory;
  }
  planned_trajectory.label = "Follow Reference";
  publisher_trajectory->publish( dynamics::conversions::to_ros_msg( planned_trajectory ) );
}

void
DecisionMaker::follow_route()
{
  dynamics::Trajectory planned_trajectory;
  map::Route           cut_route = latest_route->get_shortened_route( 100.0 );
  for( auto& p : cut_route.center_lane )
  {
    if( std::any_of( stopping_points.begin(), stopping_points.end(),
                     [&]( const auto& s ) { return adore::math::distance_2d( s, p ) < 3.0; } ) )
      p.max_speed = 0;
  }
  if( use_opti_nlc_route_following )
  {
    planned_trajectory = opti_nlc_trajectory_planner.plan_trajectory( cut_route, *latest_vehicle_state, *latest_local_map,
                                                                      latest_traffic_participants );
  }
  else
  {
    planned_trajectory = lane_follow_planner.plan_trajectory( *latest_vehicle_state, cut_route, *latest_local_map, command_limits );
  }

  if( planned_trajectory.states.size() < 2 )
  {
    standstill();
    return;
  }

  planned_trajectory.adjust_start_time( latest_vehicle_state->time );
  planned_trajectory.label = "Follow Route";
  publisher_trajectory->publish( dynamics::conversions::to_ros_msg( planned_trajectory ) );
}

void
DecisionMaker::safety_corridor()
{
  dynamics::Trajectory planned_trajectory;

  if( latest_safety_corridor.has_value() )
  {
    // calculate trajectory getting out of safety corridor
    auto   right_forward_points = planner::filter_points_in_front( latest_safety_corridor->right_border, *latest_vehicle_state );
    auto   safety_waypoints     = planner::shift_points_right( right_forward_points, 1.5 );
    double target_speed         = planner::is_point_to_right_of_line( *latest_vehicle_state, right_forward_points ) ? 0 : 2.0;

    planned_trajectory = planner::waypoints_to_trajectory( *latest_vehicle_state, safety_waypoints, dt, target_speed, command_limits );
    planned_trajectory.adjust_start_time( latest_vehicle_state->time );
    if( !default_use_reference_trajectory_as_is )
    {
      planner::OptiNLCTrajectoryOptimizer planner;
      planned_trajectory = planner.plan_trajectory( planned_trajectory, *latest_vehicle_state );
    }
    planned_trajectory.label = "Safety Corridor";
    publisher_trajectory->publish( dynamics::conversions::to_ros_msg( planned_trajectory ) );
  }
}

bool
DecisionMaker::latest_trajectory_valid()
{
  if( !latest_vehicle_state && !latest_reference_trajectory )
    return false;

  if( latest_reference_trajectory->states.size() < 2 )
    return false;

  if( latest_vehicle_state->time - latest_reference_trajectory->states.front().time > 0.5 )
    return false;

  return true;
}

void
DecisionMaker::route_callback( const adore_ros2_msgs::msg::Route& msg )
{
  latest_route = map::conversions::to_cpp_type( msg );
  if( latest_route->center_lane.size() < 2 )
    latest_route = std::nullopt;
}

void
DecisionMaker::reference_trajectory_callback( const adore_ros2_msgs::msg::Trajectory& msg )
{
  latest_reference_trajectory = dynamics::conversions::to_cpp_type( msg );
}

void
DecisionMaker::traffic_signals_callback( const adore_ros2_msgs::msg::TrafficSignals& msg )
{
  stopping_points.clear();
  for( const auto& signal : msg.signals )
  {
    stopping_points.emplace_back( signal.x, signal.y );
  }
}

void
DecisionMaker::vehicle_state_callback( const adore_ros2_msgs::msg::VehicleStateDynamic& msg )
{
  latest_vehicle_state = dynamics::conversions::to_cpp_type( msg );
  // remove nearby waypoints if nearby
  while( !latest_waypoints.empty() && adore::math::distance_2d( latest_waypoints.front(), *latest_vehicle_state ) < 2.0 )
  {
    latest_waypoints.pop_front();
  }
}

void
DecisionMaker::local_map_callback( const adore_ros2_msgs::msg::Map& msg )
{
  latest_local_map = map::conversions::to_cpp_type( msg );
}

void
DecisionMaker::safety_corridor_callback( const adore_ros2_msgs::msg::SafetyCorridor& msg )
{
  latest_safety_corridor = msg;
}

void
DecisionMaker::traffic_participants_callback( const adore_ros2_msgs::msg::TrafficParticipantSet& msg )
{
  latest_traffic_participants = dynamics::conversions::to_cpp_type( msg );
}

void
DecisionMaker::state_monitor_callback( const adore_ros2_msgs::msg::StateMonitor& msg )
{
  gps_fix_standard_deviation = msg.localization_error;
}

/** subscribes the alternative path as string*/
void
DecisionMaker::waypoints_callback( const std_msgs::msg::String& waypoints )
{
  if( state != REQUESTING_ASSISTANCE )
  {
    return;
  }
  latest_waypoints = deserialize_waypoints_from_json( waypoints.data.c_str() );
  if( latest_waypoints.empty() )
  {
    std::cerr << "ERROR in decision maker remote operations, waypoints received does not contain any values" << std::endl;
    return;
  }
  dynamics::Trajectory  trajectory = planner::waypoints_to_trajectory( *latest_vehicle_state, latest_waypoints, dt, remote_operation_speed,
                                                                       command_limits );
  std_msgs::msg::String trajectory_prediction_message = serialize_trajectory_to_json( trajectory );
  publisher_trajectory_suggestion->publish( trajectory_prediction_message );
}

void
DecisionMaker::requester_callback( const std_msgs::msg::Bool& msg )
{
  // should_request_assistance = msg.data;
  // doing_request_assistance  = true;
}

void
DecisionMaker::print_init_info()
{
  std::cout << "DecisionMaker node initialized.\n";
  std::cout << "Debug mode: " << ( debug_mode_active ? "Active" : "Inactive" ) << std::endl;
}

void
DecisionMaker::print_debug_info()
{
  double current_time_seconds = now().seconds();
  std::cerr << "------- Decision Maker Debug Information -------" << std::endl;
  std::cerr << "Current Time: " << current_time_seconds << " seconds" << std::endl;
  std::cerr << "GPS Fix Standard Deviation: " << gps_fix_standard_deviation << std::endl;

  if( latest_route )
    std::cerr << "Route available.\n";
  else
    std::cerr << "No route available.\n";
  if( latest_local_map )
    std::cerr << "Local map data available.\n";
  else
    std::cerr << "No local map data.\n";
  if( latest_vehicle_state )
    std::cerr << "Vehicle state available.\n";
  else
    std::cerr << "No vehicle state available.\n";
  if( latest_safety_corridor )
    std::cerr << "Safety Corridor message received.\n";
  else
    std::cerr << "No Safety Corridor message.\n";

  std::string state_string;
  switch( state )
  {
    case FOLLOW_REFERENCE:
      state_string = "FOLLOW_REFERECE";
      break;
    case FOLLOW_ROUTE:
      state_string = "FOLLOW_ROUTE";
      break;
    case SAFETY_CORRIDOR:
      state_string = "SAFETY_CORRIDOR";
      break;
    case STANDSTILL:
      state_string = "STANDSTILL";
      break;
    case EMERGENCY_STOP:
      state_string = "EMERGENCY";
      break;
    case REMOTE_OPERATION:
      state_string = "REMOTE OPERATIONS";
      break;
    case REQUESTING_ASSISTANCE:
      state_string = "REMOTE OPERATIONS";
      break;
    default:
      state_string = "UNKNOWN";
      break;
  }
  std::cerr << "decision maker state - " << state_string << std::endl;

  std::cerr << "------- ============================== -------" << std::endl;
}
} // namespace adore
