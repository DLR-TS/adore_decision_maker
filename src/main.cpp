#include "decision_maker.hpp"
#include "rclcpp/rclcpp.hpp"

int
main( int argc, char* argv[] )
{
  rclcpp::init( argc, argv );
  rclcpp::spin( std::make_shared<adore::DecisionMaker>(rclcpp::NodeOptions{}) );
  rclcpp::shutdown();
  return 0;
}