// Includes the necessary header files: memory management utilities, ROS2 client library, and the marker message type from the visualization_msgs package, which is commonly used for visualization purposes.
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
//intercative markers
#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>


// Simplifies type names for ease of use in the code
using Marker = visualization_msgs::msg::Marker;
using Int_marker = visualization_msgs::msg::InteractiveMarker;
using Int_control = visualization_msgs::msg::InteractiveMarkerControl;
using Server = interactive_markers::InteractiveMarkerServer;

// Defines a new class  which inherits from rclcpp::Node, a base class for ROS2 nodes.
class Move_plane : public rclcpp::Node 
{
    public: 
    // Constructor of Move_plane class, initializes the node with the name "Plane_node"
        Move_plane () : Node("Plane_node"){}
        void init(){
            // Public initialization method to set up the interactive marker server and the moving plane
            //INITIALIZATION
            //server
            server_ = std::make_shared<Server>("plane_server", shared_from_this());
            // Calls the private method to create and initialize the moving plane interactive marker
            moving_plane();
        }
    private:


    // Private method to create and set up the moving plane interactive marker
        void moving_plane()
        {
            // Create an interactive marker for the plane
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
            mType.scale.z = 0.01;
            // Sets the color of the cylinder to green with full opacity.
            mType.color.r = 0.0f;
            mType.color.g = 1.0f;
            mType.color.b = 0.0f;
            mType.color.a = 1.0;

             // Set up an interactive marker control for displaying the cube
            Int_control plane_control;
            plane_control.always_visible = true; // The marker is always visible
            plane_control.markers.push_back(mType); // Add the cube to the control

            plane.controls.push_back(plane_control); // Add control to the interactive marker

            // Set up controls for moving the plane along the X, Y, and Z axes
            // and for rotation around each axis
            Int_control control;
            control.orientation.w = 1;
            control.orientation.x = 1;
            control.orientation.y = 0;
            control.orientation.z = 0;
            //move x
            control.name = "move_x";
            control.interaction_mode = Int_control::MOVE_AXIS;
            plane.controls.push_back(control);

            control.orientation.x = 0;
            control.orientation.y = 1;
            control.orientation.z = 0;
            control.name = "move_y";
            control.interaction_mode = Int_control::MOVE_AXIS;
            plane.controls.push_back(control);

            //move y
            control.orientation.x = 0;
            control.orientation.y = 0;
            control.orientation.z = 1;
            control.name = "move_z";
            control.interaction_mode = Int_control::MOVE_AXIS;
            plane.controls.push_back(control);

            //rotation controls
            Int_control rotateX_control;
            rotateX_control.name = "rotate_x";
            rotateX_control.interaction_mode = Int_control::ROTATE_AXIS;
            rotateX_control.orientation.w = 1;
            rotateX_control.orientation.x = 1;
            rotateX_control.orientation.y = 0;
            rotateX_control.orientation.z = 0;
            plane.controls.push_back(rotateX_control);

            Int_control rotateY_control;
            rotateY_control.name = "rotate_x";
            rotateY_control.interaction_mode = Int_control::ROTATE_AXIS;
            rotateY_control.orientation.w = 1;
            rotateY_control.orientation.x = 0;
            rotateY_control.orientation.y = 1;
            rotateY_control.orientation.z = 0;
            plane.controls.push_back(rotateY_control);

            Int_control rotateZ_control;
            rotateZ_control.name = "rotate_x";
            rotateZ_control.interaction_mode = Int_control::ROTATE_AXIS;
            rotateZ_control.orientation.w = 1;
            rotateZ_control.orientation.x = 1;
            rotateZ_control.orientation.y = 0;
            rotateZ_control.orientation.z = 0;
            plane.controls.push_back(rotateZ_control);

            // Add the interactive marker to the server and set a feedback callback

            server_->insert(plane, std::bind(&Move_plane::processFeedback, this, std::placeholders::_1));
            // Apply changes to the interactive marker server
            server_ ->applyChanges();


        }

        // Callback method for processing feedback from interactive marker interaction
        void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
        {
            //print ros info in consol
            //gets current location based on Rviz grid 
             // Log the current position of the plane marker in the ROS info stream
            RCLCPP_INFO_STREAM( this -> get_logger(), 
            "Plane " << feedback->marker_name << "is now at " << feedback->pose.position.x << 
            ", " << feedback->pose.position.y << 
            ", " << feedback->pose.position.z );
        }

         // Class members
    
        
        std::shared_ptr<Server> server_; // Shared pointer for interactive marker server

};



// Defines the main function.
int main (int argc, char **argv)
{
    // - Initializes the ROS2 client library.
    rclcpp::init(argc, argv);
    // - Creates an instance of the classs and spins it to handle callbacks.
    auto node =std::make_shared<Move_plane>();
    node-> init();
    rclcpp::spin(node);
   
    // Shuts down the ROS2 client library after the node is stopped.
    rclcpp::shutdown();

    return 0;
}

//for just visualization publisher sub approach can be use, since we dont need a request or a service from the object
//*since there is no need to publish therefore timer is not needed in this case 
//Since in this case we do need a request from object which to move, then we use a server client approach