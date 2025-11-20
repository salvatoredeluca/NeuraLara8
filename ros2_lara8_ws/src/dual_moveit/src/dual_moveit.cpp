#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;


class CoordinatedMove: public rclcpp::Node
{
    public:

    CoordinatedMove(std::string gname,geometry_msgs::msg::Pose tl,geometry_msgs::msg::Pose tr):
     Node("dual_move",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        target_pose_right=tr;
        target_pose_left=tl;
        moveit_group_name=gname;
        timer_=this->create_wall_timer(500ms,std::bind(&CoordinatedMove::Start_Moveit,this));
    }
              
    private:

    void Start_Moveit()
    {
        timer_->cancel();
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_=std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(),moveit_group_name);
        const std::vector<geometry_msgs::msg::Pose> target_poses{target_pose_right,target_pose_left};

        const std::vector<double> r_joint_values={0.0,-0.5,0.0,0.0,0.5,0.0};
        const std::vector<double> l_joint_values={1.57,0.0,0.0,1.57,0.0,-1.57};
        const std::vector<double> joints_values{0.0,-0.5,0.0,0.0,0.5,0.0,
                                                1.57,0.0,0.0,1.57,0.0,-1.57};
       
        //move_group_interface_->setPoseTarget(target_pose_right,"right_lara8_link6");


        move_group_interface_->setJointValueTarget(joints_values);

        // printf("\nECCOO%s",move_group_interface_->getEndEffectorLink().c_str());

        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        
        auto const ok = static_cast<bool>(move_group_interface_->plan(plan));
        
        if (ok){move_group_interface_->execute(plan);}     
        else{RCLCPP_ERROR(this->get_logger(),"Planning failed");}


        
        
    }

    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Pose target_pose_left;
    geometry_msgs::msg::Pose target_pose_right;
    std::string moveit_group_name;
    
};


int main(int argc, char* argv[])
{
   
    rclcpp::init(argc, argv);

    auto const target_pose_left = []{
            geometry_msgs::msg::Pose msg;
            msg.orientation.w = 1.0;
            msg.position.x = 0.28;
            msg.position.y = -0.2;
            msg.position.z = 0.5;
            return msg;
            }();
    auto const target_pose_right = []{
            geometry_msgs::msg::Pose msg;
            msg.orientation.w = -1.0;
            msg.position.x = -0.28;
            msg.position.y = +0.2;
            msg.position.z = 0.5;
            return msg;
            }();
    

    
    auto dual=std::make_shared<CoordinatedMove>("dual_lara8",target_pose_left,target_pose_right);
   
    rclcpp::spin(dual);  
 
    rclcpp::shutdown();

   return 0;
}