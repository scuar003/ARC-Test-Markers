// Includes the necessary header files: memory management utilities, ROS2 client library, and the marker message type from the visualization_msgs package, which is commonly used for visualization purposes.
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "visualization_msgs/msg/marker.hpp"

//abreviations 
using Marker = visualization_msgs::msg::Marker;


// Defines a new class  which inherits from rclcpp::Node, a base class for ROS2 nodes.
class Plane : public rclcpp::Node 
{
    public: 
    // The public constructor for the  class. It initializes the node with the name ""
        Plane () : Node("Plane_node")
        {
            //INITIALIZATION
        // Creates a publisher named  for the Marker message type on the topic "" with a queue size of 10.
            pub_ = this ->create_publisher<Marker>("Plane_publisher", 10);
            // Initializes a timer that calls the timerCallback method every 500 milliseconds.
            timer_ = this ->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Plane::timerCallback, this));
        }
    private:
    // Defines a private method timerCallback that will be invoked by the timer.
        void timerCallback()
        {
            // Creates a Marker object named .
            Marker plane;
            // Sets the frame ID to "" indicating the reference frame for the marker.
            plane.header.frame_id = "base_link";
            // Sets the timestamp to the current time.
            plane.header.stamp = rclcpp::Clock().now();
            // Defines the namespace as "first_cylinder", assigns an ID of 0, and sets the marker type to CYLINDER.
            plane.ns = "plane";
            plane.id = 0;
            plane.type = Marker::CUBE;
            // Specifies the action for the marker as ADD, meaning the marker will be added to the display.
            plane.action = Marker::ADD;
            // Sets the position and orientation of the cylinder. Here, it's positioned at the origin (0,0,0) with no rotation.
            plane.pose.position.x = 0.0;
            plane.pose.position.y = 0.0;
            plane.pose.position.z = 0.0;
            plane.pose.orientation.x = 0.0;
            plane.pose.orientation.y = 0.0;
            plane.pose.orientation.z = 0.0;
            plane.pose.orientation.w = 1.0;
            // Sets the size of the object. In this case, it has a diameter of 0.1 units and a height of 0.25 units.
            plane.scale.x = 5.0;
            plane.scale.y = 5.0;
            plane.scale.z = 0.01;
            // Sets the color of the cylinder to green with full opacity.
            plane.color.r = 0.0f;
            plane.color.g = 0.0f;
            plane.color.b = 0.0f;
            plane.color.a = 1.0;

            pub_ -> publish(plane);
        }
        //DECLARATION
        //declare members of the class, shared pointers are used for automatic memory management 
        rclcpp::TimerBase::SharedPtr timer_;
        // Publishes the  marker to the  publisher.
        rclcpp::Publisher<Marker>::SharedPtr pub_;


};



// Defines the main function.
int main (int argc, char **argv)
{
    // - Initializes the ROS2 client library.
    rclcpp::init(argc, argv);
    // - Creates an instance of the classs and spins it to handle callbacks.
    rclcpp::spin(std::make_shared<Plane>());
    // Shuts down the ROS2 client library after the node is stopped.
    rclcpp::shutdown();

    return 0;
}

