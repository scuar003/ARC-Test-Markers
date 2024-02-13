#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

using Marker = visualization_msgs::msg::Marker;
using Vector = Eigen::Vector3d;

Vector pose2vector(const geometry_msgs::msg::Pose &pose) {
    return Vector(pose.position.x, pose.position.y, pose.position.z);
}

geometry_msgs::msg::Point vector2point(const Vector &v) {
    geometry_msgs::msg::Point p;
    p.x = v[0];
    p.y = v[1];
    p.z = v[2];
    return p;
}

std::vector<Vector> poses2vectors(const geometry_msgs::msg::PoseArray &poses) {
    std::vector<Vector> v;
    for (const auto &pose : poses.poses)
        v.push_back(pose2vector(pose));
    return v;
}

class Plane {
public:
    Plane(const Vector &c1, const Vector &c2, const Vector &c3, const Vector &c4) {
        m_corners.resize(4);
        m_corners[0] = c1;
        m_corners[1] = c2;
        m_corners[2] = c3;
        m_corners[3] = c4;
        normalize();
    }

    Vector vertex(int i) const { return m_corners[i]; }

    Vector centroid() const {
        Vector c;
        for (const auto &point : m_corners)
            c += point;
        c /= m_corners.size();
        return c;
    }

    Vector normal() const {
        Vector u = m_corners[1] - m_corners[0];
        Vector v = m_corners[2] - m_corners[0];
        Vector n = u.cross(v).normalized();
        return n;
    }

    Eigen::Quaterniond quaternion() const {
        Vector n = normal();
        Eigen::Quaterniond q;
        q.setFromTwoVectors(Vector::UnitZ(), n);
        return q;
    }

    double length() const {
        return (m_corners[1] - m_corners[0]).norm();
    }

    double width() const {
        return (m_corners[2] - m_corners[1]).norm();
    }

    void normalize() {
        Vector c = centroid();
        Vector n = normal();
        // Calculate perpendicular vectors on the plane
        Vector v = (std::abs(n.z()) < 0.9) ? Vector(0, 0, 1) : Vector(1, 0, 0);
        Vector i = n.cross(v).normalized();
        Vector j = n.cross(i).normalized();
        // Sides
        double l = length();
        double w = width();
        // calculate four corners
        m_corners[0] = c + 0.5 * l * i + 0.5 * w * j;
        m_corners[1] = c + 0.5 * l * i - 0.5 * w * j;
        m_corners[2] = c - 0.5 * l * i - 0.5 * w * j;
        m_corners[3] = c - 0.5 * l * i + 0.5 * w * j;
    }
private:
    std::vector<Vector> m_corners;
    Vector m_size;
};

class RepairInterface : public rclcpp::Node {
public:
    RepairInterface() : Node("repair_interface") {
        poses_subscriber_ = create_subscription<geometry_msgs::msg::PoseArray>("repair_planes", 10, std::bind(&RepairInterface::planeFinderCallback, this, std::placeholders::_1));
        renderer_publisher_ = create_publisher<Marker>("repair_plane_marker", 10);
    }
protected:
    void clearMarkers() {
        Marker marker;
        marker.header.stamp = rclcpp::Clock().now();
        marker.action = Marker::DELETEALL;
        marker.header.frame_id = "base_link";
        renderer_publisher_->publish(marker);
    }
    void showPlane(const Plane &plane, int id) {
        Marker marker;
        marker.type = Marker::CUBE;
        marker.header.frame_id = "base_link";
        marker.header.stamp = rclcpp::Clock().now();
        marker.action = Marker::ADD;
        marker.ns = "repair_planes";
        marker.id = id;
        auto c = plane.centroid();
        auto q = plane.quaternion();
        auto l = plane.length();
        auto w = plane.width();
        marker.pose.position.x = c.x();
        marker.pose.position.y = c.y();
        marker.pose.position.z = c.z();
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        marker.scale.x = l;
        marker.scale.y = w;
        marker.scale.z = 0.01;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.5f;
        renderer_publisher_->publish(marker);
    }

    void showEdge(const Plane &plane, int id) {
        Marker marker;
        marker.type = Marker::LINE_STRIP;
        marker.header.frame_id = "base_link";
        marker.header.stamp = rclcpp::Clock().now();
        marker.action = Marker::ADD;
        marker.ns = "repair_planes";
        marker.id = id + 4;
        marker.pose.orientation.w = 1.0f;
        marker.scale.x = 0.01f;

        Vector p1(-1, -1,0);
        Vector p2(-1,  1,0);
        Vector p3( 1,  1,0);
        Vector p4( 1, -1,0);
        Vector p5(-1, -1,0);
        
        marker.points.push_back( vector2point(plane.vertex(0)) );    
        marker.points.push_back( vector2point(plane.vertex(1)) );    
        marker.points.push_back( vector2point(plane.vertex(2)) );    
        marker.points.push_back( vector2point(plane.vertex(3)) );    
        marker.points.push_back( vector2point(plane.vertex(0)) );    

        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.5f;
        renderer_publisher_->publish(marker);
    }

    void update() {
        clearMarkers();
        for (long unsigned int i = 0; i < m_planes.size(); ++i) {
            showPlane(m_planes[i], i);
            showEdge(m_planes[i], i);
        }
    }
private:
    void planeFinderCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        auto v = poses2vectors(*msg);
        int n = v.size();
        // Check if multiple of 4 corners
        if (!n % 4) {
            std::cout << "Ops, " << n << " not engough plane corners!" << std::endl;
            return;
        }
        // Create planes
        m_planes.clear();
        for (int i = 0; i < n; i += 4) {
            m_planes.push_back(Plane(v[i], v[i + 1], v[i + 2], v[i + 3]));
            std::cout << "Plane:" << v[i] << ", " << v[i + 1] << ", " << v[i + 2] << ", " << v[i + 3] << std::endl;
        }
        update();
    }
private:
    rclcpp::Publisher<Marker>::SharedPtr renderer_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr poses_subscriber_;
    std::vector<Plane> m_planes;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RepairInterface>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
/////////////////////////////////////////////////////////////////////
