// this node maintain a pool of reachable blocks
// reachable means in the range of the the scara robots
// and could be fetched before it leaves robot's range
// this cylinder active pool has follow read/write property:
    // published for public read access
    // write by sending a service request

// add into the pool only when it goes into the reachable range
// delete from the pool
    // when it goes out the range
    // when it is claimed by one of the robot motion planner

// ros communication:
    // subscribe to topic "/cylinder_blocks_poses"
    // publish the topic "/cylinder_active_pool"
    // host a service for pool change "/cylinder_pool_claim"

#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <two_scara_collaboration/cylinder_blocks_poses.h>
#include <two_scara_collaboration/pool_claim_msg.h>
#include <std_msgs/Int8MultiArray.h>

// the upper and lower limit of reachable range along the direction of conveyor
const double RANGE_UPPER_LIMIT = sqrt(pow(1.8, 2) - pow(1.5, 2));
const double RANGE_LOWER_LIMIT = -sqrt(pow(1.8, 2) - pow(1.5, 2)) + 0.5;
double sawyer_pose_x;// = 0.5; //w.r.t. global frame
// max distance reachable by sawyer arm is 1 m. 
// sawyer pose is 0.0, 0.0, 0.93
// in 3d, condition for reaching object is 
// (maxlength sawyer)^2 > (sawyer_pose_x - max_object_x)^2 + (sawyer_pose_y - max_object_y)^2 + (0.93-(0.75+0.07366/2)^2
// (1)^2 > (0 - (0.75+0.38/2))^2 + (0 - max_object_y)^2 + (0.93-0.81)^2
// max_object_y^2 < (1)^2 - (0 - (0.75+0.38/2))^2 - (0.93-0.81)^2 = 0.095
// the upper and lower limit of reachable range along the direction of conveyor
double SAWYERRANGE_LOWER_LIMIT;// = -0.31;
double SAWYERRANGE_UPPER_LIMIT;// = 0.31;

double ConveyorWidthRANGE_UPPER_LIMIT;// = 0.94;
double ConveyorWidthRANGE_LOWER_LIMIT;// = 0.56;
double ObjectPoseZ_RANGE_UPPER_LIMIT;
double ObjectPoseZ_RANGE_LOWER_LIMIT;

// global variables
std::vector<int8_t> g_cylinder_active_pool;  // the POOL, elements are index of cylinders
std::vector<int8_t> SAWYERg_cylinder_active_pool;
std::vector<int8_t> removable_objects;
// add cylinder to pool from this index when checking x position
// avoid repeated adding, some cylinders may be grasped by scara and back in range
int cylinder_add_start_index = 0;
int SAWYERcylinder_add_start_index = 0;


// add and delete from the pool when in or out of the range
void cylinderPosesCallback(const two_scara_collaboration::cylinder_blocks_poses& cylinder_poses_msg) {
    // get the x coordinates
    std::vector<double> current_cylinder_x;
    std::vector<double> current_cylinder_y;
    std::vector<double> current_cylinder_z;
    int cylinder_quantity = cylinder_poses_msg.x.size();
    current_cylinder_x.resize(cylinder_quantity);
    current_cylinder_x = cylinder_poses_msg.x;
    current_cylinder_y.resize(cylinder_quantity);
    current_cylinder_y = cylinder_poses_msg.y; 
    current_cylinder_z.resize(cylinder_quantity); 
    current_cylinder_z = cylinder_poses_msg.z; 

    // check for pool addition
    // add cylinders only after the cylinder_start_index
    // for (int i=cylinder_add_start_index; i<cylinder_quantity; i++) {
    //     if (current_cylinder_x[i] > RANGE_LOWER_LIMIT
    //         && current_cylinder_x[i] < RANGE_UPPER_LIMIT
    //         && current_cylinder_y[i] > ConveyorWidthRANGE_LOWER_LIMIT
    //         && current_cylinder_y[i] < ConveyorWidthRANGE_UPPER_LIMIT
    //         ) {
    //         bool in_pool = false;  // presume not in the pool
    //         for (int j=0; j<g_cylinder_active_pool.size(); j++) {
    //             if (g_cylinder_active_pool[j] == i) {
    //                 in_pool = true;
    //                 break;  // jump out once found in pool
    //             }
    //         }
    //         if (!in_pool) {
    //             // push into the pool
    //             g_cylinder_active_pool.push_back(i);
    //             cylinder_add_start_index = i + 1;  // update index to the next cylinder
    //         }
    //     }
    // }
    bool SAWYERin_pool;

    for (int i=SAWYERcylinder_add_start_index; i<cylinder_quantity; i++) {
        // if (current_cylinder_x[i] > SAWYERRANGE_LOWER_LIMIT
        //     && current_cylinder_x[i] < SAWYERRANGE_UPPER_LIMIT
        //     && current_cylinder_y[i] > ConveyorWidthRANGE_LOWER_LIMIT
        //     && current_cylinder_y[i] < ConveyorWidthRANGE_UPPER_LIMIT
        //     
        if (current_cylinder_y[i] > SAWYERRANGE_LOWER_LIMIT
            && current_cylinder_y[i] < SAWYERRANGE_UPPER_LIMIT
            && current_cylinder_x[i] > ConveyorWidthRANGE_LOWER_LIMIT
            && current_cylinder_x[i] < ConveyorWidthRANGE_UPPER_LIMIT
            && current_cylinder_z[i] > ObjectPoseZ_RANGE_LOWER_LIMIT
            && current_cylinder_z[i] < ObjectPoseZ_RANGE_UPPER_LIMIT
            ) {

            SAWYERin_pool = false;  // presume not in the pool
            for (int j=0; j<SAWYERg_cylinder_active_pool.size(); j++) {
                if (SAWYERg_cylinder_active_pool[j] == i) {
                    SAWYERin_pool = true;
                    break;  // jump out once found in pool
                }
            }
            if (!SAWYERin_pool) {
                // push into the pool
                SAWYERg_cylinder_active_pool.push_back(i);
                SAWYERcylinder_add_start_index = i + 1;  // update index to the next cylinder
            }
        } else
        {
            // std::cout << "1 - removing it or not " << i << std::endl;
            auto it = SAWYERg_cylinder_active_pool.begin();
            if (current_cylinder_y[i] < SAWYERRANGE_LOWER_LIMIT
                || current_cylinder_y[i] > SAWYERRANGE_UPPER_LIMIT
                || current_cylinder_x[i] < ConveyorWidthRANGE_LOWER_LIMIT
                || current_cylinder_x[i] > ConveyorWidthRANGE_UPPER_LIMIT
                || current_cylinder_z[i] < ObjectPoseZ_RANGE_LOWER_LIMIT
                || current_cylinder_z[i] > ObjectPoseZ_RANGE_UPPER_LIMIT
                ) {

                // ROS_INFO_STREAM(" 2- removing it or not " << i);
                SAWYERin_pool = false;  // presume not in the pool
                for (int j=0; j<SAWYERg_cylinder_active_pool.size(); j++) {
                    // increment iterator
                    ++it;

                    if (SAWYERg_cylinder_active_pool[j] == i) {
                        SAWYERin_pool = true;
                        break;  // jump out once found in pool
                    }
                }

                if (SAWYERin_pool) {
                    // ROS_INFO_STREAM("removing model with id " << i);
                    // SAWYERcylinder_add_start_index = i - 1;  
                    // update index to the next cylinder
                    // it = SAWYERg_cylinder_active_pool.erase(it);
                }
            }             
        }
    }

    // check for pool removal
    // this should not happen if the robots are working claim cylinder from pool
    // for (int i=0; i<g_cylinder_active_pool.end() - g_cylinder_active_pool.begin(); i++) {
    //     // need use end()-begin(), because the vector length may change
    //     if (current_cylinder_x[g_cylinder_active_pool[i]] < RANGE_LOWER_LIMIT) {
    //         // remove ith cylinder from pool
    //         g_cylinder_active_pool.erase(g_cylinder_active_pool.begin() + i);
    //     }
    // }

    int8_t i;
    for (auto j = SAWYERg_cylinder_active_pool.begin(); j != SAWYERg_cylinder_active_pool.end(); ) {
        // std::cout << "1 - removing it or not " << i << std::endl;
        i=*j;// model index
        if (current_cylinder_y[i] < SAWYERRANGE_LOWER_LIMIT
            || current_cylinder_y[i] > SAWYERRANGE_UPPER_LIMIT
            || current_cylinder_x[i] < ConveyorWidthRANGE_LOWER_LIMIT
            || current_cylinder_x[i] > ConveyorWidthRANGE_UPPER_LIMIT
            || current_cylinder_z[i] < ObjectPoseZ_RANGE_LOWER_LIMIT
            || current_cylinder_z[i] > ObjectPoseZ_RANGE_UPPER_LIMIT
           ) {
            std::cout << " removing from pool  " << i << std::endl;
            j = SAWYERg_cylinder_active_pool.erase(j);
            SAWYERcylinder_add_start_index = SAWYERcylinder_add_start_index - 1;            
        } else {
            ++j;
        }
    }

    // for (int j=0; j<(SAWYERg_cylinder_active_pool.end() - SAWYERg_cylinder_active_pool.begin()); j++) {
    //     // ROS_INFO_STREAM("looking in active pool for removable objects");
    //     i=SAWYERg_cylinder_active_pool[j];// model index
    //     if (current_cylinder_y[i] < SAWYERRANGE_LOWER_LIMIT
    //         && current_cylinder_y[i] > SAWYERRANGE_UPPER_LIMIT
    //         && current_cylinder_x[i] < ConveyorWidthRANGE_LOWER_LIMIT
    //         && current_cylinder_x[i] > ConveyorWidthRANGE_UPPER_LIMIT
    //         && current_cylinder_z[i] < ObjectPoseZ_RANGE_LOWER_LIMIT
    //         && current_cylinder_z[i] > ObjectPoseZ_RANGE_UPPER_LIMIT
    //         ) {
    //         // remove ith cylinder from pool
    //         ROS_INFO_STREAM("removing model with id " << i);
    //         SAWYERg_cylinder_active_pool.erase(SAWYERg_cylinder_active_pool.begin()+j);
    //     }
    // }
}

bool cylinderPoolCallback(two_scara_collaboration::pool_claim_msgRequest& request
    , two_scara_collaboration::pool_claim_msgResponse& response) {
    // service callback for robot to claim cylinder from the active pool
    ROS_INFO_STREAM("");
    ROS_WARN("in the /cylinder_pool_claim service");  // just to highlight the result
    int requested_cylinder_index = request.cylinder_index;
    response.cylinder_claimed = false;  // preset to not found
    // for (int i=0; i<g_cylinder_active_pool.size(); i++) {
    //     if (g_cylinder_active_pool[i] == requested_cylinder_index) {
    //         // found in the pool, remove the cylinder_y[g_cylinder_pool[i]],i+1,i+2:0.240.190.14

    //         g_cylinder_active_pool.erase(g_cylinder_active_pool.begin() + i);
    //         response.cylinder_claimed = true;
    //         break;
    //     }
    // }
    for (int i=0; i<SAWYERg_cylinder_active_pool.size(); i++) {
        if (SAWYERg_cylinder_active_pool[i] == requested_cylinder_index) {
            // found in the pool, remove the cylinder
            SAWYERg_cylinder_active_pool.erase(SAWYERg_cylinder_active_pool.begin() + i);
            response.cylinder_claimed = true;
            break;
        }
    }
    // print out result of this call
    if (response.cylinder_claimed == false)
        ROS_INFO_STREAM("cylinder " << requested_cylinder_index << " fail to be claimed");
    else
        ROS_INFO_STREAM("cylinder " << requested_cylinder_index << " claimed");
    ROS_INFO_STREAM("");  // another blank line to space out above message
    return response.cylinder_claimed;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "cylinder_active_pool");
    ros::NodeHandle nh;
    nh.getParam("/sawyer_pose_x", sawyer_pose_x);
    nh.getParam("/SAWYERRANGE_LOWER_LIMIT", SAWYERRANGE_LOWER_LIMIT);
    nh.getParam("/SAWYERRANGE_UPPER_LIMIT", SAWYERRANGE_UPPER_LIMIT);
    nh.getParam("/ConveyorWidthRANGE_UPPER_LIMIT", ConveyorWidthRANGE_UPPER_LIMIT);
    nh.getParam("/ConveyorWidthRANGE_LOWER_LIMIT", ConveyorWidthRANGE_LOWER_LIMIT);
    nh.getParam("/ObjectPoseZ_RANGE_UPPER_LIMIT", ObjectPoseZ_RANGE_UPPER_LIMIT);
    nh.getParam("/ObjectPoseZ_RANGE_UPPER_LIMIT", ObjectPoseZ_RANGE_UPPER_LIMIT);

    // std::cout << std::endl << "cylinderactivepool, sawyer_pose_x:" << sawyer_pose_x << std::endl;
    std::cout << "SAWYERRANGE_LOWER_LIMIT-"<< SAWYERRANGE_LOWER_LIMIT <<
    ",SAWYERRANGE_UPPER_LIMIT-" << SAWYERRANGE_UPPER_LIMIT << ",ConveyorWidthRANGE_UPPER_LIMIT-" <<
    ConveyorWidthRANGE_UPPER_LIMIT << ",ConveyorWidthRANGE_LOWER_LIMIT-" << ConveyorWidthRANGE_LOWER_LIMIT
    << ",ObjectPoseZ_RANGE_UPPER_LIMIT-" << ObjectPoseZ_RANGE_UPPER_LIMIT <<
    ",ObjectPoseZ_RANGE_UPPER_LIMIT-" << ObjectPoseZ_RANGE_UPPER_LIMIT << std::endl;

    // initialize the active pool
    g_cylinder_active_pool.resize(0);
    g_cylinder_active_pool.clear();

    // initialize subscriber to "/cylinder_blocks_poses"
    ros::Subscriber cylinder_poses_subscriber = nh.subscribe("/cylinder_blocks_poses"
        , 1, cylinderPosesCallback);
    // initialize publisher to "/cylinder_active_pool"
    ros::Publisher cylinder_pool_publisher
        = nh.advertise<std_msgs::Int8MultiArray>("/cylinder_active_pool", 1);
    std_msgs::Int8MultiArray cylinder_active_pool_msg;
    // initialize service server of "/cylinder_pool_claim"
    ros::ServiceServer cylinder_pool_server
        = nh.advertiseService("/cylinder_pool_claim", cylinderPoolCallback);

    ros::Publisher SAWYERcylinder_pool_publisher
        = nh.advertise<std_msgs::Int8MultiArray>("/SAWYER_cylinder_active_pool", 1);
    std_msgs::Int8MultiArray SAWYERcylinder_active_pool_msg;

    // publish loop for the pool
    // inside is publish frequency, should be lower than "/cylinder_blocks_poses"
    // "/cylinder_blocks_poses" is measured at about 200hz
    ros::Rate rate_timer(100);
    while (ros::ok()) {
        cylinder_active_pool_msg.data.clear();
        cylinder_active_pool_msg.data = g_cylinder_active_pool;
        cylinder_pool_publisher.publish(cylinder_active_pool_msg);

        SAWYERcylinder_active_pool_msg.data.clear();
        SAWYERcylinder_active_pool_msg.data = SAWYERg_cylinder_active_pool;
        SAWYERcylinder_pool_publisher.publish(SAWYERcylinder_active_pool_msg);

        rate_timer.sleep();  // slow down publish rate
        ros::spinOnce();
    }

}

