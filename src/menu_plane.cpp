#include <memory>
#include <string>
#include <vector>
#include <sstream>

#include <rclcpp/rclcpp.hpp>

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
        auto marker = makePlane(1.0);
        server_ ->insert(makeMenuPlane("menu_plane", marker), std::bind(&menu_plane::processFeedback, this, _1));
        menu_handler_.apply(*server_, "menu_plane");
        server_ ->applyChanges();
        RCLCPP_INFO(get_logger(), "ready");

    }


    private:
   

        void startMenu ()
        {
            //first entry
            interactions = menu_handler_.insert("Interactions");
            Menu::EntryHandle grind = menu_handler_.insert(interactions, "Grind");
            Menu::EntryHandle paint = menu_handler_.insert(interactions, "Paint");
            Menu::EntryHandle vacum = menu_handler_.insert(interactions, "Vacum");

            //second entry
            Menu::EntryHandle details = menu_handler_.insert("Details");

            //part of the example code 
            // entry = menu_handler_.insert(entry, "sub");
            // entry = menu_handler_.insert(entry, "menu", [this](const MarkerFeedback::ConstSharedPtr)
            //                                                                                         {
            //                                                                                             RCLCPP_INFO(get_logger(), "the menu has been found");
            //                                                                                         });
           //menu_handler_.setCheckState(menu_handler_.insert("something", std::bind(&menu_plane::enableCallback, this, _1)), Menu::CHECKED);


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
        void enableCallback(const MarkerFeedback::ConstSharedPtr &feedback)
        {
            Menu::EntryHandle handle = feedback -> menu_entry_id;
            Menu::CheckState state;
            menu_handler_.getCheckState(handle, state);


            if (state == Menu::CHECKED)
            {
                menu_handler_.setCheckState(handle, Menu::UNCHECKED);
                RCLCPP_INFO(get_logger(), "hiding first entry");
                menu_handler_.setVisible(interactions, false);

            } else 
            {
                menu_handler_.setCheckState(handle, Menu::CHECKED);
                RCLCPP_INFO(get_logger(), "Showing first menu entry");
                menu_handler_.setVisible(interactions, true);
            }   

            menu_handler_.reApply(*server_);
            RCLCPP_INFO(get_logger(), "update");
            server_ ->applyChanges();

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

        Marker makePlane(const float scale)
        {
            Marker pType;
            pType.type = Marker::CUBE;
            //type, scale, color
            pType.scale.x = scale * 1.0;
            pType.scale.y = scale * 1.0;
            pType.scale.z = scale * 0.01;
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
            plane.header.frame_id = "map";
            plane.name = name;
            plane.description = "Interactive Plane";
            plane.scale = 1.0;

            plane.controls.push_back(makePlaneControl(p_));

            //free move

            IntControl rControl;
            rControl.orientation_mode = IntControl::VIEW_FACING;
            rControl.interaction_mode = IntControl::ROTATE_AXIS;
            rControl.orientation.w = 1.0;
            rControl.name = "rotate";
            plane.controls.push_back(rControl);

            IntControl mControl;
            mControl.orientation_mode = IntControl::VIEW_FACING;
            mControl.interaction_mode = IntControl::MOVE_PLANE;
            mControl.independent_marker_orientation = true;
            mControl.name = "move";
            mControl.markers.push_back(p_);
            mControl.always_visible = true;
            plane.controls.push_back(mControl);

            //Arrow movement 

            // IntControl control;
            // control.orientation.w = 1;
            // control.orientation.x = 1;
            // control.orientation.y = 0;
            // control.orientation.z = 0;
            // //move x
            // control.name = "move_x";
            // control.interaction_mode = IntControl::MOVE_AXIS;
            // plane.controls.push_back(control);

            // control.orientation.x = 0;
            // control.orientation.y = 1;
            // control.orientation.z = 0;
            // control.name = "move_y";
            // control.interaction_mode = IntControl::MOVE_AXIS;
            // plane.controls.push_back(control);

            // //move y
            // control.orientation.x = 0;
            // control.orientation.y = 0;
            // control.orientation.z = 1;
            // control.name = "move_z";
            // control.interaction_mode = IntControl::MOVE_AXIS;
            // plane.controls.push_back(control);

            // //rotation controls
            // IntControl rotateX_control;
            // rotateX_control.name = "rotate_x";
            // rotateX_control.interaction_mode = IntControl::ROTATE_AXIS;
            // rotateX_control.orientation.w = 1;
            // rotateX_control.orientation.x = 1;
            // rotateX_control.orientation.y = 0;
            // rotateX_control.orientation.z = 0;
            // plane.controls.push_back(rotateX_control);

            // IntControl rotateY_control;
            // rotateY_control.name = "rotate_x";
            // rotateY_control.interaction_mode = IntControl::ROTATE_AXIS;
            // rotateY_control.orientation.w = 1;
            // rotateY_control.orientation.x = 0;
            // rotateY_control.orientation.y = 1;
            // rotateY_control.orientation.z = 0;
            // plane.controls.push_back(rotateY_control);

            // IntControl rotateZ_control;
            // rotateZ_control.name = "rotate_x";
            // rotateZ_control.interaction_mode = IntControl::ROTATE_AXIS;
            // rotateZ_control.orientation.w = 1;
            // rotateZ_control.orientation.x = 1;
            // rotateZ_control.orientation.y = 0;
            // rotateZ_control.orientation.z = 0;
            // plane.controls.push_back(rotateZ_control);



            return plane;
        }
        //Declarations
        std::unique_ptr<Server> server_;
        Menu menu_handler_;
        Menu::EntryHandle interactions;
        Menu::EntryHandle next_entry;

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