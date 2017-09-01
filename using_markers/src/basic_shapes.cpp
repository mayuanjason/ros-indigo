# include <ros/ros.h>
# include <visualization_msgs/Marker.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle nh;
    ros::Rate r(1);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;

    while (ros::ok())
    {
        visualization_msgs::Marker marker;

        // Set the frame ID and timestamp.
        marker.header.frame_id = "/my_frame";
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker. This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "basic_shapes";
        marker.id = 0;

        // Set the marker type. Initially this is CUBE(1), and cycles between that and SPHERE(2), ARROW(0), and CYLINDER(3)
        marker.type = shape;

        // Set the marker action. Options are ADD/MODIFY(1), DELETE(2), DELETEALL(3)
        marker.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker. This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means the object will be 1m by 1m by 1m.
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        // Set the color, specified as r/g/b/a, with values in the range of [0, 1] -- Don't forget to set a or it will default to 0 and be invisible.
        // Note, an alpha (a) value of 0 means completely transparent (invisible), and 1 is completely opaque.
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        // The lifetime field specifies how long this marker should stick around before being automatically deleted. 
        // A value of ros::Duration() means never to auto-delete.
        marker.lifetime = ros::Duration();

        // Publish the marker
        while (marker_pub.getNumSubscribers() < 1)
        {
            if (!ros::ok())
            {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }

        marker_pub.publish(marker);

        // Cycle between different shapes
        switch (shape)
        {
            case visualization_msgs::Marker::CUBE:
                shape = visualization_msgs::Marker::SPHERE;
            break;

            case visualization_msgs::Marker::SPHERE:
                shape = visualization_msgs::Marker::ARROW;
            break;

            case visualization_msgs::Marker::ARROW:
                shape = visualization_msgs::Marker::CYLINDER;
            break;

            case visualization_msgs::Marker::CYLINDER:
                shape = visualization_msgs::Marker::CUBE;
            break; 
        }

        r.sleep();
    }
    
    return 0;
}