#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <two_scara_collaboration/cylinder_blocks_poses.h>
#include <two_scara_collaboration/pool_claim_msg.h>
#include <std_msgs/Int8MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include "two_scara_collaboration/claim_track.h"
#include <float.h>

// global variables
std::vector<double> g_cylinder_x;  // only x y coordinates matter
std::vector<double> g_cylinder_y;
std::vector<int8_t> g_cylinder_pool;  // the active pool

void cylinderPosesCallback(const two_scara_collaboration::cylinder_blocks_poses& cylinder_poses_msg) {
    // update poses message in global variables
    int cylinder_quantity = cylinder_poses_msg.x.size();
    g_cylinder_x.resize(cylinder_quantity);
    g_cylinder_y.resize(cylinder_quantity);
    g_cylinder_x = cylinder_poses_msg.x;
    g_cylinder_y = cylinder_poses_msg.y;

}

void cylinderPoolCallback(const std_msgs::Int8MultiArray& cylinder_pool_msg) {
    // update the cylinder active pool
    int pool_size = cylinder_pool_msg.data.size();
    g_cylinder_pool.resize(pool_size);
    g_cylinder_pool = cylinder_pool_msg.data;
}
// int to string converter
std::string intToString(int a) {
    std::stringstream ss;
    ss << a;
    return ss.str();
}

ros::ServiceClient cylinder_pool_claim_client;
two_scara_collaboration::pool_claim_msg cylinder_claim_srv_msg;
geometry_msgs::Point cylinder_position;  // the dynamic cylinder position
int cylinder_index;

bool claim_n_track(two_scara_collaboration::claim_track::Request  &req,
         two_scara_collaboration::claim_track::Response &res)
{

    bool inspect_single = req.inspect_single;
    bool cylinder_claimed = false, cylinders_claimed = false;
    int global_i;
    int cylinder_index_temp;  // the index of claimed cylinder
    double left_most_y=-DBL_MAX; 
    res.success = false;

    while ((inspect_single == 1 && !cylinder_claimed) || (inspect_single == 0 && !cylinders_claimed)) {

        cylinder_index=-1;
        int i;
        // finding left most
        for (i=0; i<g_cylinder_pool.size(); i++) {
            cylinder_index_temp = g_cylinder_pool[i];
            if (g_cylinder_y[cylinder_index_temp] > left_most_y) {
                cylinder_index = cylinder_index_temp;
                left_most_y = g_cylinder_y[cylinder_index_temp];
            }
        }

        // if there is a left most, call the cylinder claim service
        if(cylinder_index != -1) {
            cylinder_claim_srv_msg.request.cylinder_index = cylinder_index;
            if (cylinder_pool_claim_client.call(cylinder_claim_srv_msg)) {
                if (cylinder_claim_srv_msg.response.cylinder_claimed) {
                    // cylinder has been successfully claimed
                    cylinder_claimed = true;
                    res.success = true;
                }
            }
        }

        // all cylinders are claimed in case of multiple inspect
        if (inspect_single == 0) {
            for (i=0; global_i<g_cylinder_pool.size(); i++) {
                cylinder_claimed = true;
                cylinder_claim_srv_msg.request.cylinder_index = g_cylinder_pool[i];
                if (cylinder_pool_claim_client.call(cylinder_claim_srv_msg)) {
                    if (cylinder_claim_srv_msg.response.cylinder_claimed) {
                        // cylinder has been successfully claimed
                        cylinders_claimed = true;
                    } else{
                        cylinders_claimed = false;
                        break;
                    }

                }
            }
        }    
    }
    res.success = true;
    return true;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "claim_n_track_onion");
    ros::NodeHandle node_handle;
    // initialize subscriber to "/cylinder_blocks_poses"
    ros::Subscriber cylinder_poses_subscriber = node_handle.subscribe("/cylinder_blocks_poses"
    , 1, cylinderPosesCallback);
    // subscribe to topic "/cylinder_active_pool"
    ros::Subscriber cylinder_pool_subscriber = node_handle.subscribe("/SAWYER_cylinder_active_pool"
     , 1, cylinderPoolCallback);
    // service client to service "/cylinder_pool_claim"
    
    cylinder_pool_claim_client  = node_handle.serviceClient<two_scara_collaboration::pool_claim_msg>("/cylinder_pool_claim");

    int count;  // the motion count

    ros::ServiceServer service = node_handle.advertiseService("claim_n_track", claim_n_track);

    ros::Publisher pub_cylinderClaimed = node_handle.advertise<geometry_msgs::Point>("pose_claimedobject", 1000);
    ros::Rate loop_rate(30);
    cylinder_index=-1;
    
    while (ros::ok()) {

        if (cylinder_index!=-1) {
            cylinder_position.x= g_cylinder_x[cylinder_index];
            cylinder_position.y= g_cylinder_y[cylinder_index];
            pub_cylinderClaimed.publish(cylinder_position);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }


    ros::spin();
	return 0;
}
