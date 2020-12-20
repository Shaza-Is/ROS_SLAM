#include "ros/ros.h"
#include "robot_mover/DriveToTarget.h"
#include <vector>
#include <math.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Moving the bot");

    // Request velocity change to lin_x, ang_z
    robot_mover::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the drive_bot service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}
std::vector<double> unicycle(std::vector<double>u, std::vector<double>q, double T=0.1){
    double new_x = q[0] + T*u[0]*std::cos(q[2]);
    double new_y = q[1] + T*u[0]*std::sin(q[2]);
    double new_theta = q[2] + T*u[1];
    std::vector<double> new_q{new_x, new_y, new_theta}; //= np.array([new_x, new_y, new_theta])
    return new_q;
}

int main(int argc, char** argv)
{
    // Initialize the process_motion node and create a handle to it
    ros::init(argc, argv, "process_motion");
    ros::NodeHandle n;
    
    // Create a rate
    ros::Rate rate(10);
    std::vector<double> q{0, 0, 0};
    std::vector<double> u{0, 0};
    std::vector<double> new_q = unicycle(u,q);
    //for (std::vector<double>::const_iterator i = new_q.begin(); i != new_q.end(); ++i)
    //    ROS_INFO_STREAM( << *i << ' ');
    ROS_INFO_STREAM("new_q:" <<  new_q[0] << new_q[1]<< new_q[2]);

    
    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<robot_mover::DriveToTarget>("/robot_mover/command_robot");
    /*std::std::vector<double> time;
    for (size_t i = 0; i < 100; i++)
    {
        time.push_back(i*T);
    }*/
    int i = 0;
    while (ros::ok())
    {
        i++;
        if(i % 3 !=0)
            u = std::vector<double>{0.5, 0.1};
        else
        {
            u = std::vector<double>{0.0, 0.5};
        }
        
        drive_robot(u[0], u[1]);
        new_q = unicycle(u,new_q);
        ROS_INFO_STREAM("new_q:" <<  new_q[0] << new_q[1]<< new_q[2]);
        rate.sleep();
    }


    

    

    // Handle ROS communication events
    ros::spin();

    return 0;
}