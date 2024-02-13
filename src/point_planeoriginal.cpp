#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <Eigen/Dense>


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>

//alias
using namespace std::placeholders;
using Marker = visualization_msgs::msg::Marker;
using Server = interactive_markers::InteractiveMarkerServer;
using IntControl = visualization_msgs::msg::InteractiveMarkerControl;
using IntMarker = visualization_msgs::msg::InteractiveMarker;
using Menu = interactive_markers::MenuHandler;
using MarkerFeedback = visualization_msgs::msg::InteractiveMarkerFeedback;
using Evector = Eigen::Vector3d;


class menu_plane : public rclcpp::Node
{
    public:
    //constructor 
    //initializations 
    menu_plane(): Node("menu_plane"){}

    void init () 
    {
        server_ = std::make_unique<Server>("menu_server", shared_from_this());
        startMenu();

        pub_ = this->create_publisher<geometry_msgs::msg::Point>("plane_points", 10);
        sub_ = this ->create_subscription<geometry_msgs::msg::PointStamped>("/clicked_point", 10, std::bind(&menu_plane::pointCallback, this, _1));
        corner_pub_ = this->create_publisher<Marker>("corner_publisher", 10);

    }


    private:

    //////////////////////////////////////////////////////////////////////
        void pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
        {
            s_points.push_back(msg->point);
            if (s_points.size() == 4)
            {
                createPlane();
                s_points.clear(); // Clear points for next plane
                //plane_points.clear();
            }
        }
        void createPlane()
        {
            if (s_points.size() != 4) {
                RCLCPP_ERROR(this->get_logger(), "Insufficient points to form a plane");
                return;
            }

        

   

            Evector centroid(0, 0, 0);
            for (const auto& point : s_points) {
                centroid += Evector(point.x, point.y, point.z);
            }
            centroid /= s_points.size();

            // Compute normal vector
            Evector vec1 = Evector(s_points[1].x, s_points[1].y, s_points[1].z) - Evector(s_points[0].x, s_points[0].y, s_points[0].z);
            Evector vec2 = Evector(s_points[2].x, s_points[2].y, s_points[2].z) - Evector(s_points[0].x, s_points[0].y, s_points[0].z);
            Evector normal = vec1.cross(vec2).normalized();

            // Calculate perpendicular vectors on the plane
            Evector arbitraryVec = (std::abs(normal.z()) < 0.9) ? Evector(0, 0, 1) : Evector(1, 0, 0);
            Evector vecX = normal.cross(arbitraryVec).normalized();
            Evector vecY = normal.cross(vecX).normalized();

            Eigen::Quaterniond q;
            q.setFromTwoVectors(Evector::UnitZ(), normal);

            double length = distance(s_points[0], s_points[1]);
            double width = distance(s_points[1], s_points[2]);

            //vec1.normalize();
            //vec2.normalize();
            Evector halflengthVec = vecY *(length / 2.0);
            Evector halfwidthVec = vecX * (width / 2.0);

            //calculate four corners
            Evector corner1 = centroid + halflengthVec + halfwidthVec;
            Evector corner2 = centroid + halflengthVec - halfwidthVec;
            Evector corner3 = centroid - halflengthVec - halfwidthVec;
            Evector corner4 = centroid - halflengthVec + halfwidthVec;

            //send points

            plane_points.push_back(evectortoPoint(corner1));
            plane_points.push_back(evectortoPoint(corner2));
            plane_points.push_back(evectortoPoint(corner3));
            plane_points.push_back(evectortoPoint(corner4));

            // use to test
            RCLCPP_INFO(this->get_logger(), "Corner 1: (%.2f, %.2f, %.2f)", corner1.x(), corner1.y(), corner1.z());
            RCLCPP_INFO(this->get_logger(), "Corner 2: (%.2f, %.2f, %.2f)", corner2.x(), corner2.y(), corner2.z());
            RCLCPP_INFO(this->get_logger(), "Corner 3: (%.2f, %.2f, %.2f)", corner3.x(), corner3.y(), corner3.z());
            RCLCPP_INFO(this->get_logger(), "Corner 4: (%.2f, %.2f, %.2f)", corner4.x(), corner4.y(), corner4.z());

            publishCornerMarkers();

            //use to test 

            if (plane_points.size() > 3)
            {
                plane_points = std::vector<geometry_msgs::msg::Point>(plane_points.end() - 4, plane_points.end());
            }




            auto marker = makePlane(length, width); // Adjust the scale based on the calculated dimensions
            marker.pose.position.x = centroid.x();
            marker.pose.position.y = centroid.y();
            marker.pose.position.z = centroid.z();
            marker.pose.orientation.x = q.x();
            marker.pose.orientation.y = q.y();
            marker.pose.orientation.z = q.z();
            marker.pose.orientation.w = q.w(); // Adjust the scale as needed


            auto int_marker = makeMenuPlane("menu_plane", marker);
            server_->insert(int_marker, std::bind(&menu_plane::processFeedback, this, _1));
            menu_handler_.apply(*server_, "menu_plane");
            server_->applyChanges();

        }

        geometry_msgs::msg::Point evectortoPoint(const Evector & eigen_vec)
        {
            geometry_msgs::msg::Point point;
            point.x = eigen_vec.x();
            point.y = eigen_vec.y();
            point.z = eigen_vec.z();

            return point;
        }
      

        //////////////////////////////////////////////////////////////////////
        void startMenu ()
        {
            //first entry
            interactions = menu_handler_.insert("Interactions");
            grind = menu_handler_.insert(interactions, "Grind", std::bind(&menu_plane::grindCallback, this, _1));
            Menu::EntryHandle paint = menu_handler_.insert(interactions, "Paint");
            Menu::EntryHandle vacum = menu_handler_.insert(interactions, "Vacum");
            
            //next steps
            //Menu::EntryHandle whole_plane = menu_handler_.insert(grind, "whole_plane");
            //Menu::EntryHandle select_area = menu_handler_.insert(grind, "select_area");

            

            //second entry
            Menu::EntryHandle details = menu_handler_.insert("Details");

            //part of the example code 
            // entry = menu_handler_.insert(entry, "sub");
            // entry = menu_handler_.insert(entry, "menu", [this](const MarkerFeedback::ConstSharedPtr)
            //                                                                                         {
            //                                                                                             RCLCPP_INFO(get_logger(), "the menu has been found");
            //                                                                                         });
           //menu_handler_.setCheckState(menu_handler_.insert("something", std::bind(&menu_plane::enableCallback, this, _1)), Menu::CHECKED);

            //third entry
            Menu::EntryHandle sub_menu = menu_handler_.insert("switch");
            std::vector<std::string>types{"PLANE", "SPHERE", "CYLINDER", "CUBE"};
            for (int i = 0; i <4; ++i)
            {
                std::ostringstream s;
                s << types[i];
                next_entry = menu_handler_.insert(sub_menu, s.str(), std::bind(&menu_plane::modeCallback, this, _1));
                menu_handler_.setCheckState(next_entry, Menu::UNCHECKED);
            }

            menu_handler_.setCheckState(next_entry, Menu::CHECKED);



        }

        void grindCallback(const MarkerFeedback::ConstSharedPtr &feedback)
        {
            if (plane_points.size() > 3)
            {
                plane_points = std::vector<geometry_msgs::msg::Point>(plane_points.end() - 4, plane_points.end());
            }
            
            if (feedback ->menu_entry_id == grind)
            {
                for (const auto &point : plane_points)
                {
                    pub_ -> publish(point);
                    
                }
                
            }

        }

        void modeCallback(const MarkerFeedback::ConstSharedPtr &feedback)
        {
            menu_handler_.setCheckState(next_entry, Menu::UNCHECKED);
            next_entry = feedback -> menu_entry_id;
            menu_handler_.setCheckState(next_entry, Menu::CHECKED);

            RCLCPP_INFO(get_logger(), "Switching to menu entry #%d" , next_entry);

            menu_handler_.reApply(*server_);
            server_ ->applyChanges();

        }
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
        void publishCornerMarkers() 
        {
            int m_id =0;
            for (const auto& corner : plane_points) 
            {
                Marker corner_marker;
                corner_marker.header.frame_id = "base_link";
                corner_marker.header.stamp = rclcpp::Clock().now();
                corner_marker.ns = "corner_markers";
                corner_marker.id = m_id;
                corner_marker.type = Marker::SPHERE;
                corner_marker.pose.position.x = corner.x;
                corner_marker.pose.position.y = corner.y;
                corner_marker.pose.position.z = corner.z;
                corner_marker.scale.x = 0.01; // small size
                corner_marker.scale.y = 0.01;
                corner_marker.scale.z = 0.01;
                corner_marker.color.r = 1.0; // red color
                corner_marker.color.g = 0.0;
                corner_marker.color.b = 0.0;
                corner_marker.color.a = 1.0;
                m_id++;

                if (m_id >= 4)
                {
                    m_id = 0;
                }

                corner_pub_->publish(corner_marker);
            
            }
        }

        Marker makePlane( double length, double width)
        {
            Marker pType;
            pType.type = Marker::CUBE;
            //type, scale, color
            pType.scale.x = length;
            pType.scale.y = width;
            pType.scale.z = 0.01;
            pType.color.r = 0.0f;
            pType.color.g = 1.0f;
            pType.color.b = 0.0f;
            pType.color.a = 1.0;

            





            return pType;
        }
        IntControl makePlaneControl(const Marker& p_)
        {
            IntControl planeControl;
            planeControl.always_visible = true;
            planeControl.markers.push_back(p_);
            planeControl.interaction_mode = IntControl::BUTTON;
       

            return planeControl;
        }
        IntMarker makeMenuPlane(const std::string &name, const Marker& p_)
        {
            IntMarker plane;
            plane.header.frame_id = "base_link";
            plane.name = name;
            plane.description = "Interactive Plane";
            plane.scale = 1.0;
            

            plane.controls.push_back(makePlaneControl(p_));

            //free move

            //rotational controls
            // IntControl rControl;
            // rControl.orientation_mode = IntControl::VIEW_FACING;
            // rControl.interaction_mode = IntControl::ROTATE_AXIS;
            // rControl.orientation.w = 1.0;
            // rControl.name = "rotate";
            // plane.controls.push_back(rControl);

            IntControl mControl;
            mControl.orientation_mode = IntControl::VIEW_FACING;
            mControl.interaction_mode = IntControl::MOVE_PLANE;
            mControl.independent_marker_orientation = true;
            mControl.name = "move";
            mControl.markers.push_back(p_);
            mControl.always_visible = true;
            plane.controls.push_back(mControl);

   


            return plane;
        }

        double distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2)
        {
            return std::sqrt(
                std::pow(p2.x - p1.x, 2) + 
                std::pow(p2.y - p1.y, 2) +
                std::pow(p2.z - p1.z, 2)
                );
        }

        //Declarations
        std::unique_ptr<Server> server_;
        Menu menu_handler_;
        Menu::EntryHandle interactions;
        Menu::EntryHandle grind;
        Menu::EntryHandle next_entry;
        ////////////////////////////////////////////////////////////////////////////////////////////////
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_;
        rclcpp::Publisher<Marker>::SharedPtr corner_pub_;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
        std::vector<geometry_msgs::msg::Point> s_points;
        std::vector<geometry_msgs::msg::Point> plane_points;



};



int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<menu_plane>();
    node -> init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}