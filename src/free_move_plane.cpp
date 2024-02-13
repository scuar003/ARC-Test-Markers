#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>  // Main ROS2 client library.

// Messages for visualization and interactive markers.
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

// Includes for interactive marker server and menu handling.
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>

// Simplifies type names for ease of use in the code
using Marker = visualization_msgs::msg::Marker;
using Int_marker = visualization_msgs::msg::InteractiveMarker;
using Int_control = visualization_msgs::msg::InteractiveMarkerControl;
using Server = interactive_markers::InteractiveMarkerServer;

// Class for the free_move node inheriting from rclcpp::Node.
class free_move : public rclcpp::Node
{
    public:
      // Constructor for the class.
        free_move(): Node("free_move"){ }
         // Initialization function to set up the server and marker.
        void init ()
        {
            server_ = std::make_shared<Server>("server_plane", shared_from_this()); 
            free_plane();
        }


    private:
      // Function to define and set up the interactive plane marker
        void free_plane()
        {
            Int_marker plane;
            plane.header.frame_id = "map";  // Sets the reference frame
            plane.name = "my plane"; // Name of the marker
            plane.description = "Moving plane"; // Description of the marker
            // Set the initial position of the plane marker
            plane.pose.position.x = 0.0;
            plane.pose.position.y = 0.0;
            plane.pose.position.z = 0.0;


            // Define the shape and appearance of the marker (a green cube)
            Marker mType;
            mType.type = Marker::CUBE;
            mType.scale.x = 1.0;
            mType.scale.y = 1.0;
            mType.scale.z = 0.1;
            // Sets the color of the cylinder to green with full opacity.
            mType.color.r = 0.0f;
            mType.color.g = 1.0f;
            mType.color.b = 0.0f;
            mType.color.a = 1.0;

            //make a control that rotates around the view axis
            Int_control control;
            control.orientation_mode =Int_control::VIEW_FACING; // Control orientation mode.
            control.interaction_mode = Int_control::ROTATE_AXIS;  // Interaction mode for rotation
            control.orientation.w =1; // Orientation quaternion component.
            control.name = "rotate";
            plane.controls.push_back(control); // Add control to the marker.

            //move in the camara plane 
            control.orientation_mode = Int_control::VIEW_FACING; // Control orientation mode
            control.interaction_mode = Int_control::MOVE_PLANE; // Interaction mode for moving
            control.independent_marker_orientation = true;   // Marker orientation independence.
            control.name = "move";

            // Add the cube marker to the control.
            control.markers.push_back(mType);
            control.always_visible = true; // Control is always visible.
            
            plane.controls.push_back(control); // Add control to the marker.

             // Shared pointer to the interactive marker server.
            server_-> insert(plane, std::bind(&free_move::processFeedback, this, std::placeholders::_1));
            server_ -> applyChanges();



        }

        //does not let object under the grid 
        void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) 
        {
            // First, make a copy of the current pose
            auto new_pose = feedback->pose;

            // Check and enforce the boundary condition
            if (new_pose.position.z < 0.0) {
                // Modify the z position in the new pose
                new_pose.position.z = 0.0;

                // Update the server with the new pose
                server_->setPose(feedback->marker_name, new_pose);
                server_->applyChanges();

                // Log the adjustment
                RCLCPP_INFO(this->get_logger(), "Adjusted marker '%s' to z=0", feedback->marker_name.c_str());
            }
        }

        std::shared_ptr<Server> server_;



};


int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<free_move>();
    node -> init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}