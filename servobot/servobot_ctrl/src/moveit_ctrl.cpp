#include "servobot_ctrl.h"

servobot_hwinterface::servobot_hwinterface(/* args */)
{
    // connect and register the joint state interface
    for(int i=0; i<6; i++)
    {
        std::string joint_name = "joint";
        std::string joint_num = std::to_string(i);
        joint_name.append(joint_num);
        hardware_interface::JointStateHandle jnt_state_handle_tmp(joint_name, &pos[i], &vel[i], &eff[i]);
        jnt_state_interface.registerHandle(jnt_state_handle_tmp);
    }
    registerInterface(&jnt_state_interface);

    // connect and register the joint velocity interface
    for(int i=0; i<6; i++)
    {
        std::string joint_name = "joint";
        std::string joint_num = std::to_string(i);
        joint_name.append(joint_num);
        hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(joint_name), &cmd[i]);
        jnt_pos_interface.registerHandle(pos_handle);
    }
    registerInterface(&jnt_pos_interface);


    cmd_pub = n.advertise<std_msgs::Float32MultiArray>("cmd", 1);
    pos_sub = n.subscribe("jointstates", 1, &servobot_hwinterface::read, this);

}

servobot_hwinterface::~servobot_hwinterface()
{
}

void servobot_hwinterface::write()
{
    for(int i=0; i<6; i++)
    {
        if(i==4)
            pub_cmd[i] = -cmd[i];
        else
            pub_cmd[i] = cmd[i];
        

        // pos[i] = cmd[i];
    }
    cmdmsg.data = pub_cmd;
    cmd_pub.publish(cmdmsg);
    

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "servobot_hwinterface");
    ros::NodeHandle nh;

    servobot_hwinterface bot;
    controller_manager::ControllerManager ctrl(&bot);	//class of controller，link to bot

	ros::AsyncSpinner spinner(2); //mutiple thread
	spinner.start();
    ros::Rate rate(25);

	while(ros::ok())
	{
        ctrl.update(bot.getTime(), bot.getPeriod());

        bot.write();	
        //ros::spinOnce();
        rate.sleep();
	}
	spinner.stop();

	return 0;

    // nh.createTimer

}
