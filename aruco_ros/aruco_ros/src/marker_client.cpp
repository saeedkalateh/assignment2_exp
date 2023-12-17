#include <ros/ros.h>
#include <assignment2/RoomConnection.h>
#include <assignment2/RoomInformation.h>
#include <std_msgs/Int32.h>

class MarkerClient
{
    private:
        std::vector<std::uint32_t> roomsID;
        ros::Subscriber sub;
        ros::ServiceClient client;

    public:
        MarkerClient()
        {
            ros::NodeHandle nh;
            sub = nh.subscribe("/image_id", 1000, &MarkerClient::markerCallback, this);
            client = nh.serviceClient<assignment2::RoomInformation>("room_info");
        }

        void markerCallback(const std_msgs::Int32::ConstPtr& msg)
        {   
            if(!std::count(roomsID.begin(), roomsID.end(), msg->data) && msg->data > 10 && msg->data < 18)
            {
                roomsID.push_back(msg->data);
                ROS_INFO("Image id detected: [%d]", msg->data);
                assignment2::RoomInformation srv;
                srv.request.id = msg->data;
                if(client.call(srv))
                {
                    ROS_INFO_STREAM("Semantic map updated: room " << srv.response.room << " detected");
                    ROS_INFO("Center position is: [%f, %f]", srv.response.x,  srv.response.y);
                    for(int i = 0; i < srv.response.connections.size(); i++)
                    {
                        ROS_INFO_STREAM("Room " << srv.response.room << " is connected to " << srv.response.connections[i].connected_to << " through door " <<  srv.response.connections[i].through_door);
                    }   
                }
                else
                {
                    ROS_ERROR("Failed to call service room_info");
                }
            }
        }
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "marker_client");
    
    MarkerClient node;

    ros::spin();
	ros::shutdown();
	return 0;
}
