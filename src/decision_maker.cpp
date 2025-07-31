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

#include "adore_ros2_msgs/msg/traffic_participant_set.hpp"
#include "adore_ros2_msgs/msg/vehicle_info.hpp"
#include <adore_math/point.h>

namespace adore
{

DecisionMaker::DecisionMaker( const rclcpp::NodeOptions& options ) :
  Node( "decision_maker", options )
{
  load_parameters();
  create_subscribers();
  create_publishers();
  print_init_info();
}

void
DecisionMaker::create_publishers()
{
  publisher_trajectory                           = create_publisher<adore_ros2_msgs::msg::Trajectory>( "trajectory_decision", 10 );
  publisher_trajectory_suggestion                = create_publisher<adore_ros2_msgs::msg::Trajectory>( "trajectory_suggestion", 10 );
  publisher_request_assistance_remote_operations = create_publisher<adore_ros2_msgs::msg::AssistanceRequest>( "request_assistance", 10 );
  publisher_traffic_participant                  = create_publisher<adore_ros2_msgs::msg::TrafficParticipant>( "traffic_participant", 10 );
  publisher_traffic_participant_with_trajectory_prediction = create_publisher<adore_ros2_msgs::msg::TrafficParticipantSet>( "traffic_prediction", 10 );
  publisher_caution_zones                        = create_publisher<adore_ros2_msgs::msg::CautionZone>( "caution_zones", 10 );
}

void
DecisionMaker::create_subscribers()
{
  subscriber_route = create_subscription<adore_ros2_msgs::msg::Route>( "route", 1,
                                                                       std::bind( &DecisionMaker::route_callback, this,
                                                                                  std::placeholders::_1 ) );

  subscriber_goal = create_subscription<adore_ros2_msgs::msg::GoalPoint>( "mission/goal_position", 10,
                                                                          std::bind( &DecisionMaker::goal_callback, this,
                                                                                     std::placeholders::_1 ) );

  subscriber_vehicle_state = create_subscription<adore_ros2_msgs::msg::VehicleStateDynamic>(
    "vehicle_state/dynamic", 1, std::bind( &DecisionMaker::vehicle_state_callback, this, std::placeholders::_1 ) );

  subscriber_local_map = create_subscription<adore_ros2_msgs::msg::Map>( "local_map", 1,
                                                                         std::bind( &DecisionMaker::local_map_callback, this,
                                                                                    std::placeholders::_1 ) );

  subscriber_safety_corridor = create_subscription<adore_ros2_msgs::msg::SafetyCorridor>(
    "safety_corridor", 1, std::bind( &DecisionMaker::safety_corridor_callback, this, std::placeholders::_1 ) );

  subscriber_vehicle_info = create_subscription<adore_ros2_msgs::msg::VehicleInfo>( "vehicle_info", 1,
                                                                                    std::bind( &DecisionMaker::vehicle_info_callback, this,
                                                                                               std::placeholders::_1 ) );

  subscriber_waypoints = create_subscription<adore_ros2_msgs::msg::Waypoints>( "remote_operation_waypoints", 1,
                                                                               std::bind( &DecisionMaker::waypoints_callback, this,
                                                                                          std::placeholders::_1 ) );

  subscriber_traffic_signals = create_subscription<adore_ros2_msgs::msg::TrafficSignals>(
    "traffic_signals", 1, std::bind( &DecisionMaker::traffic_signals_callback, this, std::placeholders::_1 ) );

  subscriber_suggested_trajectory_acceptance = create_subscription<std_msgs::msg::Bool>(
    "suggested_trajectory_accepted", 1,
    std::bind( &DecisionMaker::suggested_trajectory_acceptance_callback, this, std::placeholders::_1 ) );

  subscriber_traffic_participant_set = create_subscription<adore_ros2_msgs::msg::TrafficParticipantSet>(
    "traffic_participants", 1, std::bind( &DecisionMaker::traffic_participants_callback, this, std::placeholders::_1 ) );

  subscriber_infrastructure_traffic_participants = create_subscription<adore_ros2_msgs::msg::TrafficParticipantSet>(
    "infrastructure_traffic_participants", 10, std::bind( &DecisionMaker::traffic_participants_callback, this, std::placeholders::_1 ) );
  
  subscriber_time_headway = create_subscription<std_msgs::msg::Float64>( "time_headway", 1, std::bind( &DecisionMaker::time_headway_callback, this,
      std::placeholders::_1 ) );

  main_timer = create_wall_timer( std::chrono::milliseconds( static_cast<size_t>( 1000 * dt ) ), std::bind( &DecisionMaker::run, this ) );
}

void
DecisionMaker::load_parameters()
{
  std::string vehicle_model_file;
  declare_parameter( "vehicle_model_file", "" );
  get_parameter( "vehicle_model_file", vehicle_model_file );
  model = dynamics::PhysicalVehicleModel( vehicle_model_file, false );

  declare_parameter( "debug_mode_active", true );
  get_parameter( "debug_mode_active", debug_mode_active );

  declare_parameter( "use_reference_trajectory_as_is", true );
  get_parameter( "use_reference_trajectory_as_is", default_use_reference_trajectory_as_is );

  declare_parameter( "only_follow_reference_trajectories", false );
  get_parameter( "only_follow_reference_trajectories", only_follow_reference_trajectories );

  declare_parameter( "optinlc_route_following", false );
  get_parameter( "optinlc_route_following", use_opti_nlc_route_following );

  declare_parameter( "dt", 0.05 );
  get_parameter( "dt", dt );

  declare_parameter( "min_route_length", 4.0 );
  get_parameter( "min_route_length", min_route_length );

  declare_parameter( "min_reference_trajectory_size", 5 );
  get_parameter( "min_reference_trajectory_size", min_reference_trajectory_size );

  declare_parameter( "remote_operation_speed", 2.0 );
  get_parameter( "remote_operation_speed", remote_operation_speed );

  declare_parameter( "max_acceleration", 2.0 );
  declare_parameter( "min_acceleration", -2.0 );
  declare_parameter( "max_steering", 0.7 );
  get_parameter( "max_acceleration", command_limits.max_acceleration );
  get_parameter( "min_acceleration", command_limits.min_acceleration );
  get_parameter( "max_steering", command_limits.max_steering_angle );

  command_limits.max_steering_angle = std::min( command_limits.max_steering_angle, model.params.steering_angle_max );
  command_limits.max_acceleration   = std::min( command_limits.max_acceleration, model.params.acceleration_max );
  command_limits.min_acceleration   = std::max( command_limits.min_acceleration, model.params.acceleration_min );

  std::vector<std::string> keys;
  std::vector<double>      values;
  declare_parameter( "planner_settings_keys", keys );
  declare_parameter( "planner_settings_values", values );
  get_parameter( "planner_settings_keys", keys );
  get_parameter( "planner_settings_values", values );

  std::vector<double> ra_polygon_values;
  declare_parameter( "request_assistance_polygon", std::vector<double>{} );
  get_parameter( "request_assistance_polygon", ra_polygon_values );

  if( ra_polygon_values.size() >= 6 )
  {
    adore::math::Polygon2d polygon;
    polygon.points.reserve( ra_polygon_values.size() / 2 );

    for( size_t i = 0; i < ra_polygon_values.size(); i += 2 )
    {
      double x = ra_polygon_values[i];
      double y = ra_polygon_values[i + 1];
      polygon.points.push_back( { x, y } );
    }
    caution_zones["Request Assistance"] = polygon;
    RCLCPP_INFO( this->get_logger(), "Loaded request assistance polygon with %zu points", polygon.points.size() );
  }

  if( keys.size() != values.size() )
  {
    RCLCPP_ERROR( this->get_logger(), "Planner settings keys and values size mismatch! Keys: %zu, Values: %zu", keys.size(), values.size() );
    return;
  }
  
  for( size_t i = 0; i < keys.size(); ++i )
  {
    planner_settings.insert( { keys[i], values[i] } );
    RCLCPP_DEBUG( this->get_logger(), "Planner parameter: %s = %f", keys[i].c_str(), values[i] );
  }

  opti_nlc_trajectory_planner.set_parameters( planner_settings );
  multi_agent_PID_planner.set_parameters( planner_settings );
  
  RCLCPP_INFO( this->get_logger(), "Parameters loaded successfully" );
}

void
DecisionMaker::run()
{
  update_state();
  
  if( latest_vehicle_state )
  {
    dynamics::VehicleCommand last_control( latest_vehicle_state->steering_angle, latest_vehicle_state->ax );
    dynamics::integrate_up_to_time( *latest_vehicle_state, last_control, now().seconds(), model.motion_model );
  }

  try 
  {
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
        RCLCPP_WARN( this->get_logger(), "Unknown state, executing emergency stop" );
        emergency_stop();
        break;
    }
  }
  catch( const std::exception& e )
  {
    RCLCPP_ERROR( this->get_logger(), "Exception in decision maker run loop: %s", e.what() );
    emergency_stop();
  }

  publish_traffic_participant();

  if( debug_mode_active )
  {
    print_debug_info();
  }
}

void
DecisionMaker::update_state()
{
  check_caution_zones();
  int current_conditions = 0;
  
  if( gps_fix_standard_deviation < 0.2 && latest_vehicle_state )
    current_conditions |= VEHICLE_STATE_OK;
  if( latest_safety_corridor )
    current_conditions |= SAFETY_CORRIDOR_PRESENT;
  if( latest_waypoints.size() > 1 && !need_assistance )
    current_conditions |= WAYPOINTS_AVAILABLE;
  if( latest_trajectory_valid() )
    current_conditions |= REFERENCE_TRAJECTORY_VALID;
  if( latest_route_valid() && !only_follow_reference_trajectories )
    current_conditions |= ROUTE_AVAILABLE;
  if( latest_local_map )
    current_conditions |= LOCAL_MAP_AVAILABLE;
  if( need_assistance )
    current_conditions |= NEED_ASSISTANCE;

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
DecisionMaker::check_caution_zones()
{
  if( !latest_vehicle_state )
    return;
    
  for( const auto& [label, polygon] : caution_zones )
  {
    if( label == "Request Assistance" && polygon.point_inside( latest_vehicle_state.value() ) && state != REMOTE_OPERATION
        && state != REQUESTING_ASSISTANCE )
    {
      need_assistance = true;
      RCLCPP_WARN( this->get_logger(), "Vehicle entered request assistance zone" );
    }
    
    adore_ros2_msgs::msg::CautionZone caution_zone_msg;
    caution_zone_msg.label           = label;
    caution_zone_msg.polygon         = math::conversions::to_ros_msg( polygon );
    caution_zone_msg.header.frame_id = "world";
    publisher_caution_zones->publish( caution_zone_msg );
  }
}

void
DecisionMaker::emergency_stop()
{
  RCLCPP_WARN( this->get_logger(), "Executing emergency stop" );
  dynamics::Trajectory emergency_stop_trajectory;
  if( latest_vehicle_state )
    emergency_stop_trajectory.states.push_back( latest_vehicle_state.value() );
  emergency_stop_trajectory.label = "Emergency Stop";
  publisher_trajectory->publish( dynamics::conversions::to_ros_msg( emergency_stop_trajectory ) );
}

void
DecisionMaker::standstill()
{
  RCLCPP_DEBUG( this->get_logger(), "Executing standstill maneuver" );
  dynamics::Trajectory standstill_trajectory;
  standstill_trajectory.label = "Standstill";
  if( latest_vehicle_state )
    standstill_trajectory.states.push_back( latest_vehicle_state.value() );
  publisher_trajectory->publish( dynamics::conversions::to_ros_msg( standstill_trajectory ) );
}

void
DecisionMaker::request_assistance()
{
  RCLCPP_INFO_THROTTLE( this->get_logger(), *this->get_clock(), 5000, "Requesting assistance" );
  
  if( latest_local_map && latest_route && latest_vehicle_state->vx > 0.5 )
  {
    minimum_risk_maneuver();
  }
  else
  {
    standstill();
  }

  adore_ros2_msgs::msg::AssistanceRequest assistance_request;
  assistance_request.assistance_needed = true;
  assistance_request.state             = dynamics::conversions::to_ros_msg( latest_vehicle_state.value() );
  assistance_request.header.stamp      = now();

  publisher_request_assistance_remote_operations->publish( assistance_request );
  sent_assistance_request = true;
}

void
DecisionMaker::remote_operation()
{
  RCLCPP_DEBUG( this->get_logger(), "Executing remote operation with %zu waypoints", latest_waypoints.size() );
  
  if( latest_waypoints.empty() )
  {
    RCLCPP_WARN( this->get_logger(), "No waypoints available for remote operation, switching to standstill" );
    standstill();
    return;
  }
  
  try
  {
    dynamics::Trajectory trajectory = planner::waypoints_to_trajectory( *latest_vehicle_state, latest_waypoints, dt, remote_operation_speed,
                                                                        command_limits, traffic_participants, model );
    trajectory.label = "Remote Operation";
    publisher_trajectory->publish( dynamics::conversions::to_ros_msg( trajectory ) );
  }
  catch( const std::exception& e )
  {
    RCLCPP_ERROR( this->get_logger(), "Exception in remote operation planning: %s", e.what() );
    standstill();
  }
}

void
DecisionMaker::follow_reference()
{
  RCLCPP_DEBUG( this->get_logger(), "Following reference trajectory" );
  
  if( !latest_reference_trajectory || !latest_vehicle_state )
  {
    RCLCPP_ERROR( this->get_logger(), "Missing data for reference following: trajectory=%s, vehicle_state=%s",
                  latest_reference_trajectory ? "OK" : "MISSING",
                  latest_vehicle_state ? "OK" : "MISSING" );
    standstill();
    return;
  }
  
  try
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
  catch( const std::exception& e )
  {
    RCLCPP_ERROR( this->get_logger(), "Exception in reference trajectory planning: %s", e.what() );
    standstill();
  }
}

void
DecisionMaker::compute_trajectories_for_traffic_participant_set( dynamics::TrafficParticipantSet& traffic_participant_set )
{
  if( latest_local_map.has_value() )
  {
    compute_routes_for_traffic_participant_set( traffic_participant_set );
  }
  
  if( !latest_vehicle_info.has_value() )
  {
    RCLCPP_ERROR( this->get_logger(), "Cannot compute trajectories: vehicle info not available" );
    return;
  }
  
  if( !latest_vehicle_state.has_value() )
  {
    RCLCPP_ERROR( this->get_logger(), "Cannot compute trajectories: vehicle state not available" );
    return;
  }
  
  try
  {
    // CRITICAL: Always ensure ego vehicle is added to traffic participants
    dynamics::TrafficParticipant ego_vehicle;
    ego_vehicle.state = latest_vehicle_state.value();
    ego_vehicle.goal_point = goal;
    ego_vehicle.id = static_cast<int64_t>(latest_vehicle_info->v2x_station_id);
    ego_vehicle.route = latest_route;
    ego_vehicle.physical_parameters = model.params;
    
    // Always add/update ego vehicle
    traffic_participant_set.participants[ego_vehicle.id] = ego_vehicle;
    
    RCLCPP_DEBUG( this->get_logger(), "Added ego vehicle with ID %ld to traffic participant set (total: %zu participants)", 
                  ego_vehicle.id, traffic_participant_set.participants.size() );
    
    // Only run multi-agent planning if we have the ego vehicle
    multi_agent_PID_planner.plan_trajectories( traffic_participant_set );
  }
  catch( const std::exception& e )
  {
    RCLCPP_ERROR( this->get_logger(), "Exception in multi-agent trajectory planning: %s", e.what() );
    
    // Even if multi-agent planning fails, ensure we have at least ego vehicle for trajectory planning
    if( latest_vehicle_info.has_value() && latest_vehicle_state.has_value() )
    {
      traffic_participant_set.participants.clear();
      dynamics::TrafficParticipant ego_vehicle;
      ego_vehicle.state = latest_vehicle_state.value();
      ego_vehicle.goal_point = goal;
      ego_vehicle.id = static_cast<int64_t>(latest_vehicle_info->v2x_station_id);
      ego_vehicle.route = latest_route;
      ego_vehicle.physical_parameters = model.params;
      traffic_participant_set.participants[ego_vehicle.id] = ego_vehicle;
      
      RCLCPP_WARN( this->get_logger(), "Multi-agent planning failed, using ego vehicle only" );
    }
  }
}

void
DecisionMaker::compute_routes_for_traffic_participant_set( dynamics::TrafficParticipantSet& traffic_participant_set )
{
  for( auto& [id, participant] : traffic_participant_set.participants )
  {
    bool no_goal  = !participant.goal_point.has_value();
    bool no_route = !participant.route.has_value();

    if( !no_goal && no_route )
    {
      try
      {
        participant.route = map::Route( participant.state, participant.goal_point.value(), *latest_local_map );
        if( participant.route->center_lane.empty() )
        {
          participant.route = std::nullopt;
          RCLCPP_WARN( this->get_logger(), "No route found for traffic participant %ld", id );
        }
        else
        {
          RCLCPP_DEBUG( this->get_logger(), "Computed route for traffic participant %ld", id );
        }
      }
      catch( const std::exception& e )
      {
        RCLCPP_ERROR( this->get_logger(), "Exception computing route for participant %ld: %s", id, e.what() );
        participant.route = std::nullopt;
      }
    }
  }
}

void
DecisionMaker::follow_route()
{
  if( !latest_route || !latest_vehicle_state || !latest_local_map )
  {
    RCLCPP_ERROR( this->get_logger(), "Missing required data for route following: route=%s, vehicle_state=%s, local_map=%s",
                  latest_route ? "OK" : "MISSING",
                  latest_vehicle_state ? "OK" : "MISSING", 
                  latest_local_map ? "OK" : "MISSING" );
    standstill();
    return;
  }

  // CRITICAL FIX: Don't proceed without vehicle info - trajectory planner needs ego vehicle
  if( !latest_vehicle_info )
  {
    RCLCPP_ERROR( this->get_logger(), "Cannot follow route: vehicle info not available (needed for ego vehicle in traffic participants)" );
    standstill();
    return;
  }

  RCLCPP_DEBUG( this->get_logger(), "Starting route following with %zu traffic participants", traffic_participants.participants.size() );
  
  dynamics::Trajectory planned_trajectory;
  double state_s = latest_route->get_s( latest_vehicle_state.value() );
  auto cut_route = latest_route->get_shortened_route( state_s, 100.0 );
  
  for( auto& p : cut_route )
  {
    if( std::any_of( stopping_points.begin(), stopping_points.end(),
                     [&]( const auto& s ) { return adore::math::distance_2d( s, p ) < 3.0; } ) )
      p.max_speed = 0;
  }
  
  double start_time = now().seconds();
  
  try
  {
    // Create a copy and ensure ego vehicle is always present
    dynamics::TrafficParticipantSet planning_participants = traffic_participants;
    compute_trajectories_for_traffic_participant_set( planning_participants );
    
    double prediction_time = now().seconds() - start_time;
    RCLCPP_DEBUG( this->get_logger(), "Traffic prediction computation took %.6f seconds", prediction_time );
    
    // CRITICAL CHECK: Ensure ego vehicle is in participants before planning
    int64_t ego_id = static_cast<int64_t>(latest_vehicle_info->v2x_station_id);
    if( planning_participants.participants.find(ego_id) == planning_participants.participants.end() )
    {
      RCLCPP_ERROR( this->get_logger(), "Ego vehicle (ID: %ld) not found in traffic participants after computation. Participants: %zu", 
                    ego_id, planning_participants.participants.size() );
      
      // Log all participant IDs for debugging
      for( const auto& [id, participant] : planning_participants.participants )
      {
        RCLCPP_ERROR( this->get_logger(), "  Available participant ID: %ld", id );
      }
      
      standstill();
      return;
    }
    
    planned_trajectory = opti_nlc_trajectory_planner.plan_trajectory( latest_route.value(), *latest_vehicle_state, *latest_local_map,
                                                                      planning_participants );
  }
  catch( const std::out_of_range& e )
  {
    RCLCPP_ERROR( this->get_logger(), "Map access error in trajectory planning - likely invalid participant ID: %s", e.what() );
    
    RCLCPP_ERROR( this->get_logger(), "Current traffic participant IDs:" );
    for( const auto& [id, participant] : traffic_participants.participants )
    {
      RCLCPP_ERROR( this->get_logger(), "  ID: %ld", id );
    }
    
    if( latest_vehicle_info )
    {
      RCLCPP_ERROR( this->get_logger(), "Expected ego vehicle ID: %ld", static_cast<int64_t>(latest_vehicle_info->v2x_station_id) );
    }
    
    standstill();
    return;
  }
  catch( const std::exception& e )
  {
    RCLCPP_ERROR( this->get_logger(), "Exception in trajectory planning: %s", e.what() );
    standstill();
    return;
  }

  if( planned_trajectory.states.size() < 2 )
  {
    RCLCPP_WARN( this->get_logger(), "Planned trajectory too short (%zu states), switching to standstill", planned_trajectory.states.size() );
    standstill();
    return;
  }
  
  planned_trajectory.adjust_start_time( latest_vehicle_state->time );
  planned_trajectory.label = "Follow Route";
  publisher_trajectory->publish( dynamics::conversions::to_ros_msg( planned_trajectory ) );
  publisher_traffic_participant_with_trajectory_prediction->publish( dynamics::conversions::to_ros_msg( traffic_participants ) );
}

void
DecisionMaker::minimum_risk_maneuver()
{
  RCLCPP_INFO( this->get_logger(), "Executing minimum risk maneuver" );
  
  if( !latest_route || !latest_vehicle_state || !latest_local_map )
  {
    RCLCPP_ERROR( this->get_logger(), "Missing data for minimum risk maneuver, falling back to standstill" );
    standstill();
    return;
  }
  
  // CRITICAL FIX: Also check for vehicle info here
  if( !latest_vehicle_info )
  {
    RCLCPP_ERROR( this->get_logger(), "Cannot execute minimum risk maneuver: vehicle info not available" );
    standstill();
    return;
  }
  
  try
  {
    dynamics::Trajectory planned_trajectory;
    double state_s = latest_route->get_s( latest_vehicle_state.value() );
    auto cut_route = latest_route->get_shortened_route( state_s, 100.0 );
    
    for( auto& p : cut_route )
    {
      p.max_speed = 0;
    }

    // Ensure we have ego vehicle in traffic participants
    dynamics::TrafficParticipantSet planning_participants = traffic_participants;
    compute_trajectories_for_traffic_participant_set( planning_participants );

    planned_trajectory = opti_nlc_trajectory_planner.plan_trajectory( latest_route.value(), *latest_vehicle_state, *latest_local_map,
                                                                      planning_participants );

    if( planned_trajectory.states.size() < 2 )
    {
      standstill();
      return;
    }
    
    planned_trajectory.adjust_start_time( latest_vehicle_state->time );
    planned_trajectory.label = "Minimum Risk Maneuver";
    publisher_trajectory->publish( dynamics::conversions::to_ros_msg( planned_trajectory ) );
  }
  catch( const std::exception& e )
  {
    RCLCPP_ERROR( this->get_logger(), "Exception in minimum risk maneuver: %s", e.what() );
    standstill();
  }
}

void
DecisionMaker::safety_corridor()
{
  RCLCPP_DEBUG( this->get_logger(), "Executing safety corridor maneuver" );
  
  if( !latest_safety_corridor.has_value() || !latest_vehicle_state )
  {
    RCLCPP_WARN( this->get_logger(), "Missing data for safety corridor, falling back to standstill" );
    standstill();
    return;
  }
  
  try
  {
    dynamics::Trajectory planned_trajectory;

    auto   right_forward_points = planner::filter_points_in_front( latest_safety_corridor->right_border, *latest_vehicle_state );
    auto   safety_waypoints     = planner::shift_points_right( right_forward_points, 1.5 );
    double target_speed         = planner::is_point_to_right_of_line( *latest_vehicle_state, right_forward_points ) ? 0 : 2.0;

    planned_trajectory = planner::waypoints_to_trajectory( *latest_vehicle_state, safety_waypoints, dt, target_speed, command_limits,
                                                           traffic_participants, model );
    planned_trajectory.adjust_start_time( latest_vehicle_state->time );
    
    if( !default_use_reference_trajectory_as_is )
    {
      planner::OptiNLCTrajectoryOptimizer planner;
      planned_trajectory = planner.plan_trajectory( planned_trajectory, *latest_vehicle_state );
    }
    
    planned_trajectory.label = "Safety Corridor";
    publisher_trajectory->publish( dynamics::conversions::to_ros_msg( planned_trajectory ) );
  }
  catch( const std::exception& e )
  {
    RCLCPP_ERROR( this->get_logger(), "Exception in safety corridor planning: %s", e.what() );
    standstill();
  }
}

bool
DecisionMaker::latest_trajectory_valid()
{
  if( !latest_vehicle_state || !latest_reference_trajectory )
    return false;

  if( latest_reference_trajectory->states.size() < min_reference_trajectory_size )
  {
    RCLCPP_DEBUG( this->get_logger(), "Reference trajectory too short: %zu < %zu", 
                  latest_reference_trajectory->states.size(), min_reference_trajectory_size );
    return false;
  }

  double time_diff = latest_vehicle_state->time - latest_reference_trajectory->states.front().time;
  if( time_diff > 0.5 )
  {
    RCLCPP_DEBUG( this->get_logger(), "Reference trajectory too old: %.3f seconds", time_diff );
    return false;
  }

  return true;
}

bool
DecisionMaker::latest_route_valid()
{
  if( !latest_route || !latest_vehicle_state )
    return false;
    
  double remaining_route_length = latest_route->get_length() - latest_route->get_s( *latest_vehicle_state );
  bool valid = remaining_route_length > min_route_length;
  
  if( !valid )
  {
    RCLCPP_DEBUG( this->get_logger(), "Route invalid: remaining length %.2f < %.2f", 
                  remaining_route_length, min_route_length );
  }
  
  return valid;
}

void
DecisionMaker::time_headway_callback( const std_msgs::msg::Float64& msg )
{
  double time_headway = msg.data;
  std::map<std::string, double> params;
  params["time_headway"] = time_headway;
  multi_agent_PID_planner.set_parameters( params );
  RCLCPP_DEBUG( this->get_logger(), "Updated time headway parameter: %.3f", time_headway );
}

void
DecisionMaker::route_callback( const adore_ros2_msgs::msg::Route& msg )
{
  try
  {
    latest_route = map::conversions::to_cpp_type( msg );
    if( latest_route->center_lane.size() < 1 )
    {
      latest_route = std::nullopt;
      RCLCPP_WARN( this->get_logger(), "Received empty route" );
    }
    else
    {
      if( latest_local_map )
        latest_route->map = std::make_shared<map::Map>( latest_local_map.value() );
      
      RCLCPP_DEBUG( this->get_logger(), "Route received with %zu center lane points", latest_route->center_lane.size() );
    }
  }
  catch( const std::exception& e )
  {
    RCLCPP_ERROR( this->get_logger(), "Exception processing route: %s", e.what() );
    latest_route = std::nullopt;
  }
}

void
DecisionMaker::traffic_signals_callback( const adore_ros2_msgs::msg::TrafficSignals& msg )
{
  stopping_points.clear();
  for( const auto& signal : msg.signals )
  {
    stopping_points.emplace_back( signal.x, signal.y );
  }
  RCLCPP_DEBUG( this->get_logger(), "Received %zu traffic signals", msg.signals.size() );
}

void
DecisionMaker::vehicle_state_callback( const adore_ros2_msgs::msg::VehicleStateDynamic& msg )
{
  try
  {
    latest_vehicle_state = dynamics::conversions::to_cpp_type( msg );
    
    while( !latest_waypoints.empty() && adore::math::distance_2d( latest_waypoints.front(), *latest_vehicle_state ) < 2.0 )
    {
      latest_waypoints.pop_front();
    }
  }
  catch( const std::exception& e )
  {
    RCLCPP_ERROR( this->get_logger(), "Exception processing vehicle state: %s", e.what() );
  }
}

void
DecisionMaker::local_map_callback( const adore_ros2_msgs::msg::Map& msg )
{
  try
  {
    latest_local_map = map::conversions::to_cpp_type( msg );
    RCLCPP_DEBUG( this->get_logger(), "Local map received" );
  }
  catch( const std::exception& e )
  {
    RCLCPP_ERROR( this->get_logger(), "Exception processing local map: %s", e.what() );
  }
}

void
DecisionMaker::safety_corridor_callback( const adore_ros2_msgs::msg::SafetyCorridor& msg )
{
  latest_safety_corridor = msg;
  RCLCPP_DEBUG( this->get_logger(), "Safety corridor received" );
}

void
DecisionMaker::traffic_participants_callback( const adore_ros2_msgs::msg::TrafficParticipantSet& msg )
{
  if( !latest_vehicle_info )
  {
    RCLCPP_WARN_THROTTLE( this->get_logger(), *this->get_clock(), 1000, 
                          "Traffic participant callback received data but vehicle info is not available yet" );
    return;
  }

  try
  {
    auto new_participants_data = dynamics::conversions::to_cpp_type( msg );
    
    int64_t ego_id = static_cast<int64_t>( latest_vehicle_info.value().v2x_station_id );
    
    RCLCPP_DEBUG( this->get_logger(), "Processing %zu traffic participants, ego ID: %ld", 
                  new_participants_data.participants.size(), ego_id );
    
    for( const auto& [id, new_participant] : new_participants_data.participants )
    {
      RCLCPP_DEBUG( this->get_logger(), "Processing participant ID: %ld", id );
      
      if( id == ego_id )
      {
        if( new_participant.trajectory.has_value() )
        {
          latest_reference_trajectory = new_participant.trajectory.value();
          RCLCPP_DEBUG( this->get_logger(), "Updated reference trajectory from ego participant ID %ld", id );
        }
        continue;
      }
      
      if( id == 0 || id == std::numeric_limits<int64_t>::max() || id == std::numeric_limits<int64_t>::min() )
      {
        RCLCPP_WARN( this->get_logger(), "Received traffic participant with invalid ID: %ld, skipping", id );
        continue;
      }
      
      traffic_participants.update_traffic_participants( new_participant );
    }

    traffic_participants.remove_old_participants( 1.0, now().seconds() );
    
    RCLCPP_DEBUG( this->get_logger(), "Processed traffic participants successfully. Total active: %zu", 
                  traffic_participants.participants.size() );
  }
  catch( const std::exception& e )
  {
    RCLCPP_ERROR( this->get_logger(), "Exception processing traffic participants: %s", e.what() );
    traffic_participants.participants.clear();
  }
}

void
DecisionMaker::publish_traffic_participant()
{
  if( !latest_vehicle_state || !latest_route || !latest_vehicle_info )
    return;
    
  try
  {
    dynamics::TrafficParticipant ego_as_participant;
    ego_as_participant.state               = latest_vehicle_state.value();
    ego_as_participant.goal_point          = goal;
    ego_as_participant.id                  = latest_vehicle_info->v2x_station_id;
    ego_as_participant.v2x_id              = latest_vehicle_info->v2x_station_id;
    ego_as_participant.classification      = dynamics::CAR;
    ego_as_participant.route               = latest_route;
    ego_as_participant.physical_parameters = model.params;

    publisher_traffic_participant->publish( dynamics::conversions::to_ros_msg( ego_as_participant ) );
  }
  catch( const std::exception& e )
  {
    RCLCPP_ERROR( this->get_logger(), "Exception publishing traffic participant: %s", e.what() );
  }
}

void
DecisionMaker::vehicle_info_callback( const adore_ros2_msgs::msg::VehicleInfo& msg )
{
  gps_fix_standard_deviation = msg.localization_error;
  latest_vehicle_info        = msg;
  RCLCPP_DEBUG( this->get_logger(), "Vehicle info received: station_id=%u, localization_error=%.3f", 
                msg.v2x_station_id, msg.localization_error );
}

void
DecisionMaker::waypoints_callback( const adore_ros2_msgs::msg::Waypoints& waypoints_msg )
{
  if( state != REQUESTING_ASSISTANCE )
  {
    RCLCPP_DEBUG( this->get_logger(), "Received waypoints but not in requesting assistance state" );
    return;
  }

  try
  {
    latest_waypoints.clear();
    std::transform( waypoints_msg.waypoints.begin(), waypoints_msg.waypoints.end(), std::back_inserter( latest_waypoints ),
                    []( const adore_ros2_msgs::msg::Point2d& pt ) { return adore::math::Point2d( pt.x, pt.y ); } );

    double target_speed = remote_operation_speed;
    if( !waypoints_msg.speed_limits.empty() )
      target_speed = std::max( target_speed, *std::max_element( waypoints_msg.speed_limits.begin(), waypoints_msg.speed_limits.end() ) );

    dynamics::Trajectory trajectory = planner::waypoints_to_trajectory( *latest_vehicle_state, latest_waypoints, dt, remote_operation_speed,
                                                                        command_limits, traffic_participants, model );
    publisher_trajectory_suggestion->publish( dynamics::conversions::to_ros_msg( trajectory ) );
    sent_suggestion = true;
    
    RCLCPP_INFO( this->get_logger(), "Remote operation waypoints received: %zu points", waypoints_msg.waypoints.size() );
  }
  catch( const std::exception& e )
  {
    RCLCPP_ERROR( this->get_logger(), "Exception processing waypoints: %s", e.what() );
  }
}

void
DecisionMaker::suggested_trajectory_acceptance_callback( const std_msgs::msg::Bool& msg )
{
  if( !( state == REQUESTING_ASSISTANCE || state == REMOTE_OPERATION ) )
    return;
    
  need_assistance         = !msg.data;
  sent_suggestion         = false;
  sent_assistance_request = false;
  
  RCLCPP_INFO( this->get_logger(), "Trajectory suggestion %s", msg.data ? "accepted" : "rejected" );
}

void
DecisionMaker::goal_callback( const adore_ros2_msgs::msg::GoalPoint& msg )
{
  goal.x = msg.x_position;
  goal.y = msg.y_position;
  RCLCPP_INFO( this->get_logger(), "Goal received: (%.2f, %.2f)", goal.x, goal.y );
}

void
DecisionMaker::print_init_info()
{
  RCLCPP_INFO( this->get_logger(), "DecisionMaker node initialized" );
  RCLCPP_INFO( this->get_logger(), "Debug mode: %s", debug_mode_active ? "Active" : "Inactive" );
  RCLCPP_INFO( this->get_logger(), "Control loop rate: %.1f Hz", 1.0 / dt );
}

void
DecisionMaker::print_debug_info()
{
  double current_time_seconds = now().seconds();
  RCLCPP_DEBUG( this->get_logger(), "=== Decision Maker Debug Information ===" );
  RCLCPP_DEBUG( this->get_logger(), "Current Time: %.3f seconds", current_time_seconds );
  RCLCPP_DEBUG( this->get_logger(), "GPS Fix Standard Deviation: %.3f", gps_fix_standard_deviation );
  RCLCPP_DEBUG( this->get_logger(), "Reference trajectory: %s", latest_reference_trajectory ? "Available" : "Missing" );
  RCLCPP_DEBUG( this->get_logger(), "Route: %s", latest_route ? "Available" : "Missing" );
  RCLCPP_DEBUG( this->get_logger(), "Local map: %s", latest_local_map ? "Available" : "Missing" );
  RCLCPP_DEBUG( this->get_logger(), "Vehicle state: %s", latest_vehicle_state ? "Available" : "Missing" );
  RCLCPP_DEBUG( this->get_logger(), "Safety corridor: %s", latest_safety_corridor ? "Available" : "Missing" );
  RCLCPP_DEBUG( this->get_logger(), "Vehicle info: %s", latest_vehicle_info ? "Available" : "Missing" );
  RCLCPP_DEBUG( this->get_logger(), "Waypoints count: %zu", latest_waypoints.size() );
  RCLCPP_DEBUG( this->get_logger(), "Traffic participants count: %zu", traffic_participants.participants.size() );
  RCLCPP_DEBUG( this->get_logger(), "Needs assistance: %s", need_assistance ? "Yes" : "No" );

  std::string state_string;
  switch( state )
  {
    case FOLLOW_REFERENCE: state_string = "FOLLOW_REFERENCE"; break;
    case FOLLOW_ROUTE: state_string = "FOLLOW_ROUTE"; break;
    case SAFETY_CORRIDOR: state_string = "SAFETY_CORRIDOR"; break;
    case STANDSTILL: state_string = "STANDSTILL"; break;
    case EMERGENCY_STOP: state_string = "EMERGENCY_STOP"; break;
    case REMOTE_OPERATION: state_string = "REMOTE_OPERATION"; break;
    case REQUESTING_ASSISTANCE: state_string = "REQUESTING_ASSISTANCE"; break;
    default: state_string = "UNKNOWN"; break;
  }
  RCLCPP_DEBUG( this->get_logger(), "Decision maker state: %s", state_string.c_str() );
}

} // namespace adore

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE( adore::DecisionMaker )
