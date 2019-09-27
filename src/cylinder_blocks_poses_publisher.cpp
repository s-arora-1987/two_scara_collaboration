// this node will publish the topic "cylinder_blocks_poses"
// including all current cylinder blocks, pose is 3-D position

// ros communication:
    // subscribe to topic "/current_cylinder_blocks"
    // subscribe to topic "/gazebo/model_states"
    // publish the topic "/cylinder_blocks_poses"

#include <ros/ros.h>
#include <vector>
#include <string>
#include <std_msgs/Int8MultiArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <two_scara_collaboration/cylinder_blocks_poses.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/DeleteModel.h>

// global variables
int g_cylinder_quantity;
std::vector<int8_t> g_current_cylinder_blocks;
std::vector<double> g_x;
std::vector<double> g_y;
std::vector<double> g_z;
bool g_current_cylinder_callback_started = false;
bool g_cylinder_poses_updated = false;  // act as frequency control of publish loop

ros::ServiceClient apply_wrench_client;
bool call_service;
gazebo_msgs::ApplyBodyWrench apply_wrench_srv_msg;  // service message
double SAWYERRANGE_UPPER_LIMIT, SAWYERRANGE_LOWER_LIMIT, initial_pose_y,
initial_pose_y_to_RANGE_LIMIT;
ros::ServiceClient delete_model_client;
gazebo_msgs::DeleteModel delete_model_srv_msg;
std::vector<int> indices_deleted;
int goodOnionsConvEnd=0;
int badOnionsConvEnd=0;
int goodOnionsInBin=0;
int badOnionsInBin=0;

std::string intToString(int a) {
    std::stringstream ss;
    ss << a;
    return ss.str();
}

void currentCylinderCallback(const std_msgs::Int8MultiArray& current_cylinder_blocks) {
    // this topic contains information of what cylinder blocks have been spawned
    std::cout << "received ros msg on /current_cylinder_blocks in file cylinder_blocks_poses_publisher.cpp";
    if (!g_current_cylinder_callback_started) {
        // set first time started flag to true
        g_current_cylinder_callback_started = true;
        ROS_INFO("current cylinder callback has been invoked first time");
    }
    g_cylinder_quantity = current_cylinder_blocks.data.size();
    g_current_cylinder_blocks.resize(g_cylinder_quantity);
    g_current_cylinder_blocks = current_cylinder_blocks.data;
}

void modelStatesCallback(const gazebo_msgs::ModelStates& current_model_states) {
    // this callback update global values of cylinder positions
    if (g_current_cylinder_callback_started) {
        // only update when currentCylinderCallback has been invoked the first time
        // get cylinder blocks positions according to settings in g_current_cylinder_blocks
        std::vector<double> cylinder_x;
        std::vector<double> cylinder_y;
        std::vector<double> cylinder_z;
        cylinder_x.resize(g_cylinder_quantity);
        cylinder_y.resize(g_cylinder_quantity);
        cylinder_z.resize(g_cylinder_quantity);
        // find position of all current cylinders in topic message
        bool cylinder_poses_completed = true;
        // std::cout << "g_cylinder_quantity - " << g_cylinder_quantity << std::endl;
        for (int i=0; i<g_cylinder_quantity; i++) {
            
            // get index of ith cylinder
            std::string indexed_model_name;
            if (g_current_cylinder_blocks[i] == 0) {
                indexed_model_name = "red_cylinder_" + intToString(i);
            } else {
                indexed_model_name = "blue_cylinder_" + intToString(i);
            }
            int index = -1;
            int model_quantity = current_model_states.name.size();  // number of models measured
            // std::cout << "model_quantity - " << model_quantity << std::endl;
            for (int j=0; j<model_quantity; j++) {
                if (current_model_states.name[j] == indexed_model_name) {
                    index = j; break;
                }
            }
            if (index != -1) {
                // this model name exists and has been successfully indexed
                cylinder_x[i] = current_model_states.pose[index].position.x;
                cylinder_y[i] = current_model_states.pose[index].position.y;
                cylinder_z[i] = current_model_states.pose[index].position.z;

                // update global counter for Onions in Bin and delete them
                // if (cylinder_x[i] > SAWYERRANGE_UPPER_LIMIT) {

                // update global counter for Onions reaching end of conveyor and delete them
                if (cylinder_y[i] > SAWYERRANGE_UPPER_LIMIT ||
                 cylinder_y[i] < (SAWYERRANGE_LOWER_LIMIT+initial_pose_y_to_RANGE_LIMIT-0.02)) {
                    if (g_current_cylinder_blocks[i] == 0) {
                        goodOnionsConvEnd += 1;
                    } else {
                        badOnionsConvEnd += 1;
                    }

                    delete_model_srv_msg.request.model_name = indexed_model_name;
                    // call apply body wrench service

                    call_service = delete_model_client.call(delete_model_srv_msg);
                    if (call_service) {                        
                        // Trying twice to make sure it is deleted
                        // Not using while loop because sometimes model is deleted 
                        // gazebo service doesn't return success
                        if (!delete_model_srv_msg.response.success) {
                            call_service = delete_model_client.call(delete_model_srv_msg);
                            if (delete_model_srv_msg.response.success) {
                                std::cout << "adding model index " << i << " in vector. " << std::endl; 
                                indices_deleted.push_back(i);
                                ROS_INFO_STREAM("deleted model " << indexed_model_name);
                                // break;
                                // ROS_INFO_STREAM(indexed_model_name << " speed re-initialized");
                            } else {
                                ROS_INFO_STREAM(" fail to delete_model " << indexed_model_name);
                            }
                        } else {
                            std::cout << "adding model index " << i << " in vector. " << std::endl; 
                            indices_deleted.push_back(i);
                            ROS_INFO_STREAM("deleted model " << indexed_model_name);
                        }
                        // break;
                    } else {
                        ROS_ERROR("fail to connect with gazebo server for delete_model");
                        // return 0;
                    }                    
                } 

                // for rest of objects objects on conveyor, keep pushing forward
                if ((cylinder_z[i] >= 0.75) && (cylinder_z[i] <= 0.85)) {
                    // prepare apply body wrench service message
                    apply_wrench_srv_msg.request.body_name = indexed_model_name + "::base_link";
                    // call apply body wrench service
                    call_service = apply_wrench_client.call(apply_wrench_srv_msg);
                    if (call_service) {
                        if (apply_wrench_srv_msg.response.success) {
                            // ROS_INFO_STREAM(indexed_model_name << " speed re-initialized");
                        } else {
                            ROS_INFO_STREAM(indexed_model_name << " fail to re-initialize speed");
                        }
                    }
                    else {
                        ROS_ERROR("fail to connect with gazebo server for apply wrench");
                        // return 0;
                    }                    
                }
            } else {

                bool model_deleted=false;
                int ind_del;
                // for (auto j = indices_deleted.begin(); j != indices_deleted.end(); ) {
                //     ind_del = *j;// model index
                for (int ind_del: indices_deleted) {    
                    std::cout << "current indices in indices_deleted " << ind_del;
                    if (ind_del == i) {
                        // model has been deleted
                        // std::cout << "found model index" << ind_del << "in vector " << std::endl; 
                        cylinder_x[i] = -100.0;
                        cylinder_y[i] = -100.0;
                        cylinder_z[i] = -100.0;
                        std::cout << "cylinder_x[i] = -100.0 for i " << i << std::endl;
                        model_deleted=true; 
                        break;
                    }
                }

                if (model_deleted == false) {
                    // cylinder is not accounted for in either deleted or spawned models
                    std::cout << "cylinder_poses_completed = false at i " << i << std::endl;
                    cylinder_poses_completed = false;
                }
                // ROS_ERROR("fail to find model name in the model_states topic");
                // in the test run, there is chance that the last cylinder is not in the topic message
                // and g_cylinder_quantity (fron spawner node) is larger than the cylinder quantity here
                // because /gazebo/model_states are sampled at a high rate of about 1000Hz
                // so the position data should be aborted if fail to find the last cylinder
                // cylinder_poses_completed = false;
            }
        }

        if (cylinder_poses_completed) {
            // only pass data to globals when they are completed
            g_x.resize(g_cylinder_quantity);
            g_y.resize(g_cylinder_quantity);
            g_z.resize(g_cylinder_quantity);
            g_x = cylinder_x;
            g_y = cylinder_y;
            g_z = cylinder_z;
            // std::cout << "g_x, g_y, g_z updated" << std::endl;
            if (!g_cylinder_poses_updated) {
                // reset flag to true, after updating global value of g_x, g_y, g_z
                g_cylinder_poses_updated = true;
            }
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cylinder_blocks_poses_publisher");
    ros::NodeHandle nh; 

    // initialize subscribers for "/current_cylinder_blocks" and "/gazebo/model_states"
    ros::Subscriber current_cylinder_subscriber = nh.subscribe("/current_cylinder_blocks"
        , 1, currentCylinderCallback);
    ros::Subscriber model_states_subscriber = nh.subscribe("/gazebo/model_states", 1, modelStatesCallback);
    // initialize publisher for "/cylinder_blocks_poses"
    ros::Publisher cylinder_poses_publisher
        = nh.advertise<two_scara_collaboration::cylinder_blocks_poses>("cylinder_blocks_poses", 1);
    two_scara_collaboration::cylinder_blocks_poses current_poses_msg;

    double wrench_force_x, wrench_force_y, wrench_torque_y;
    nh.getParam("/wrench_force_y", wrench_force_y);
    nh.getParam("/SAWYERRANGE_UPPER_LIMIT", SAWYERRANGE_UPPER_LIMIT);
    nh.getParam("/initial_pose_y", initial_pose_y);
    nh.getParam("/SAWYERRANGE_LOWER_LIMIT", SAWYERRANGE_LOWER_LIMIT);
    nh.getParam("/initial_pose_y_to_RANGE_LIMIT", initial_pose_y_to_RANGE_LIMIT);

    apply_wrench_client
    = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
     // make sure /gazebo/apply_body_wrench service is ready
    bool service_ready = false;
    while (!service_ready) {
        service_ready = ros::service::exists("/gazebo/apply_body_wrench", true);
        ROS_INFO("waiting for apply_body_wrench service");
        ros::Duration(0.5).sleep();
    }
    ros::Time time_temp(0, 0);
    ros::Duration duration_temp(0, 350); 
    // ros::Duration duration_temp(-1); 
    apply_wrench_srv_msg.request.wrench.force.x = 0.0; 
    apply_wrench_srv_msg.request.wrench.force.y = wrench_force_y;
    apply_wrench_srv_msg.request.wrench.force.z = 0.0;
    apply_wrench_srv_msg.request.wrench.torque.x = 0.0;
    apply_wrench_srv_msg.request.wrench.torque.y = 0.0;
    apply_wrench_srv_msg.request.wrench.torque.z = 0.0;
    apply_wrench_srv_msg.request.start_time = time_temp;
    apply_wrench_srv_msg.request.duration = duration_temp;
    apply_wrench_srv_msg.request.reference_frame = "world";
    ROS_INFO("apply_body_wrench service and msg are ready");

    delete_model_client
    = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
    // make sure Delete Model service is ready
    service_ready = false;
    while (!service_ready) {
        service_ready = ros::service::exists("/gazebo/delete_model", true);
        ROS_INFO("waiting for delete_urdf_model service");
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("delete_urdf_model service is ready");

    ros::Rate rate_timer(50);
    // publishing loop
    while (ros::ok()) {
        if (g_cylinder_poses_updated) {
            // only publish when cylinder positions are updated
            // no need to publish repeated data
            g_cylinder_poses_updated = false;  // set flag to false
            // there is tiny possibility that g_x is not in the length of g_cylinder_quantity
            int local_cylinder_quantity = g_x.size();  // so get length of g_x
            current_poses_msg.x.resize(local_cylinder_quantity);
            current_poses_msg.y.resize(local_cylinder_quantity);
            current_poses_msg.z.resize(local_cylinder_quantity);
            current_poses_msg.x = g_x;
            current_poses_msg.y = g_y;
            current_poses_msg.z = g_z;
            cylinder_poses_publisher.publish(current_poses_msg);
        }
        rate_timer.sleep();
        ros::spinOnce();
    }
    return 0;
}



