#include <ros/ros.h>
#include <memory>
#include <thread>
#include "visualization.h"
#include "zcm_message_manager.h"

int main(int argc, char **argv) {

    planning::ZcmMessageManager *message_manager =
      planning::ZcmMessageManager::GetInstance();
    // std::thread
    // ZcmHandleThread(&planning::ZcmMessageManager::ReceiveMessagesOnIpc,
    //     message_manager);
    message_manager->Start(); // start receiving zcm message

    ros::init(argc, argv, "visualization");
    ros::NodeHandle nh;
    std::unique_ptr<planning::Visualization> visualizer =
        std::make_unique<planning::Visualization>(nh);
    ros::Rate loop_rate(100);
    while (ros::ok()) {
        clock_t start,finish;
        start = clock(); 
        ros::spinOnce();
        ros::Time begin = ros::Time::now();

        visualizer->RunOnce(); 
        
        loop_rate.sleep();
        ros::Time end = ros::Time::now();
        ros::Duration d(end - begin);
        finish = clock();
        double totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
        totaltime = totaltime*1000;
        // std::cout<<"t_:"<<totaltime<<"ms!"<<std::endl;
        ROS_WARN("[RUNONCE Time: %lf ms]", d.toSec() * 1000.0);

    }
    // ZcmHandleThread.join();
    message_manager->Stop();

    return 0;
}
