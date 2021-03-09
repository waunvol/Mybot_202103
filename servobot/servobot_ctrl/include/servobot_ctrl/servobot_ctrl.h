#ifndef SERVOBOT_CTRL_H
#define SERVOBOT_CTRL_H
#include <std_msgs/Float32MultiArray.h>
#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>



class servobot_hwinterface : public hardware_interface::RobotHW
{
    private:
        /* data */
        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::PositionJointInterface jnt_pos_interface;
        // std::string sub_name[6]={"joint_state0", "joint_state1", "joint_state2", "joint_state3", "joint_state4", "joint_state5"};
        // std::string pub_name[6]={"cmd0", "cmd1", "cmd2", "cmd3", "cmd4", "cmd5"};

        double cmd[6] = {0, 0, 0, 0, 0, 0};
        double pos[6] = {0, 0, 0, 0, 0, 0};
        double vel[6] = {0, 0, 0, 0, 0, 0};
        double eff[6] = {0, 0, 0, 0, 0, 0};
        std::vector<float> pub_cmd = {0,0,0,0,0,0};

        ros::NodeHandle n;
        ros::Publisher cmd_pub;
        ros::Subscriber pos_sub;
        std_msgs::Float32MultiArray cmdmsg;  


    public:
        servobot_hwinterface(/* args */);
        ~servobot_hwinterface();

        void write(void);
        void read(const std_msgs::Float32MultiArray::ConstPtr& rec_msg)
        {
            for(int i=0; i<6; i++)
            {
                if(i==4)
                    pos[i] = -rec_msg->data[i];
                else
                    pos[i] = rec_msg->data[i];
                    
            }
                

        };
        ros::Time getTime() const {return ros::Time::now();}
        ros::Duration getPeriod() const {return ros::Duration(0.01);}



};


#endif