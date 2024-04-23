#include <chrono>

#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <cansend.h>
#include <unistd.h>
#include <boost/thread/thread.hpp>
#include <mutex> 

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
// #include "rclcpp_components/register_node_macro.hpp"


#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
// #include <std_srvs/srv/set_bool.h>
// #include "std_srvs/srv/Trigger.h"
using namespace std::placeholders;
using namespace std::chrono_literals;
std::mutex velo_lock;
std::mutex servo_lock;
std::mutex arm_status_lock;

// class MinimalSubscriber : public rclcpp::Node
// {
//   public:
//     MinimalSubscriber()
//     : Node("minimal_subscriber")
//     {
//       subscription_ = this->create_subscription<std_msgs::msg::String>(
//       "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
//     }

//   private:
//     void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
//     {
//       RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
//     }
//     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
// };



class VescInterface : public rclcpp::Node
{
    static int steering_cmd;
    static int velo_cmd;
    static int velo_cmd_ratio;
    static int servo_cmd_ratio;

    static int rpm_scale;
    static bool arm_status_;
    static int publish_ratio;

    public:
    VescInterface(): Node("VescInterface_node")
    {

        manual_override_engaged_ = false;

        memset(cmd, '0', sizeof(cmd));

    	cmd[6] = '3';
    	cmd[7] = '3';
        cmd[8] = '#';
        cmd[17] = '\0';
        manual_override_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "low_level_control/manual_override", 10, std::bind(&VescInterface::manualOverrideCallback, this, _1));

        twist_control_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "low_level_control/cmd_vel", 10, std::bind(&VescInterface::twistControlCallback, this, _1));


        armed_status_pub_ = this->create_publisher<std_msgs::msg::Bool>("low_level_control/arm_status", 10);
        manual_override_status_pub_ = this->create_publisher<std_msgs::msg::Bool>("low_level_control/manual_override_status", 10);        
        timer_ = this->create_wall_timer(500ms, std::bind(&VescInterface::timer_callback, this));
        
        arming_server_ = this->create_service<std_srvs::srv::SetBool>("low_level_control/arming", std::bind(&VescInterface::armDisarmCallback, this, _1, _2));
    }

    ~VescInterface()
    {}

    template <typename I>
    static std::string n2hexstr(I w, size_t hex_len = sizeof(I)<<1 )
    {
        static const char* digits = "0123456789ABCDEF";
        std::string rc(hex_len, '0');
        for(size_t i=0,j=(hex_len-1)*4; i<hex_len;++i,j-=4 )
        {
            rc[i] = digits[(w>>j) & 0x0f];
        }
        return rc;
    }





    void timer_callback()
    {
        // Print current state.
        // if(std::fabs((ros::Time::now() - last_manual_mode_).toSec()) > manual_mode_timeout_) {
        // manual_override_engaged_ = false;
        // }

        // ROS_INFO_THROTTLE(4, "Armed: %d, Manual: %d",
        //         arm_status_,
        //         static_cast<int>(manual_override_engaged_)
        //         );

        // Publish Arming Status
        std_msgs::msg::Bool arm_msg;

        arm_status_lock.lock();
        arm_msg.data = arm_status_;
        arm_status_lock.unlock();
                
        armed_status_pub_->publish(arm_msg);

        // Publish Manual Override Status
        std_msgs::msg::Bool manual_override_msg;
        manual_override_msg.data = manual_override_engaged_;
        manual_override_status_pub_->publish(manual_override_msg);
    }

    void armDisarmCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) 
    {
        response->message = arm_status_ ? "Was armed." : "Was disarmed.";
        response->success = true;

        // mavros_msgs::CommandBool arm_cmd;
        // arm_cmd.request.value = request.data;

        arm_status_lock.lock();
        arm_status_ = !arm_status_ ;
        // if(DEBUG)
        // {
        //     std::cout<< "vesc arm: "<< arm_status_ << "\n";
        // }
        arm_status_lock.unlock();
        
        // return true;
    }

    void twistControlCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        if(manual_override_engaged_) {
            // ROS_WARN_THROTTLE(5, "[Command] Unable to use command, manual override engaged.");
            RCLCPP_INFO(this->get_logger(), "test 1");
            return;
        }
        // double x = msg->twist.linear.x;
        // double z = msg->twist.angular.z;
        // publishCommand(x, z);
        publishCommand(msg->twist.linear.x, msg->twist.angular.z);
    };


    void manualOverrideCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg){
        // last_manual_mode_ = ros::Time::now();
        // manual_override_engaged_ = true;
        RCLCPP_INFO(this->get_logger(), " test 2: %f %f", msg->twist.linear.x, msg->twist.angular.z);
        publishCommand(msg->twist.linear.x, msg->twist.angular.z);
    };

    double servo_convert(double& angular)
    {
        return ( -1.0*angular + max_steering_angle_ ) / (2.0 * max_steering_angle_);
    }

    void send_velo_cmd(int velo_cmd)
    {
        std::string st = n2hexstr(velo_cmd,8);
        // cmd ID
        cmd[4] = '0';
        cmd[5] = '3';

    	cmd[9] = st[0];
    	cmd[10] = st[1];
        cmd[11] = st[2];
	    cmd[12] = st[3];
        cmd[13] = st[4];
        cmd[14] = st[5];
	    cmd[15] = st[6];
    	cmd[16] = st[7];
    	cansend(cmd);
    }
    
    void send_steering_cmd(int steering_cmd)
    {
        std::string st = n2hexstr(steering_cmd, 4);
        
        // cmd ID
        cmd[4] = '3';
    	cmd[5] = 'F';

    	cmd[9] = '0';
    	cmd[10] = '0';
        cmd[11] = '0';
	    cmd[12] = '0';
    	cmd[13] = st[0];
    	cmd[14] = st[1];
    	cmd[15] = st[2];
    	cmd[16] = st[3];
    	cansend(cmd);
    }

    void publishCommand(double linear_x, double angular_z) {

        steering_cmd = ( int(servo_convert(angular_z) * servo_scale) + servo_bias );

        if(steering_cmd < servo_lower_bound)
                steering_cmd = servo_lower_bound;
        if(steering_cmd > servo_upper_bound)
                steering_cmd = servo_upper_bound;
        
        velo_cmd = int( (linear_x * rpm_scale) );
        
        if(DEBUG)
        {
            std::cout << "linear_x: " <<( linear_x ) << std::endl;
            std::cout << "velo_cmd : " <<( velo_cmd  ) << "\n\n";  

            std::cout << "angular_z: " <<( angular_z ) << std::endl;
            std::cout << "steering_cmd : " <<( steering_cmd ) << "\n\n";  
        }

        if( !arm_status_ )
        {
            return;
        }

        send_steering_cmd(steering_cmd);
        send_velo_cmd(velo_cmd);

    }

    //private:
        bool manual_override_engaged_ = true;

        double manual_mode_timeout_ = 1;
        double max_steering_angle_ = 0.524;
        double forward_steering_trim_ = -90.0;
        double reverse_steering_trim_ = 75.0;
        double forward_scale_a_ = 160;
        double forward_scale_b_ = 12;
        double reverse_scale_a_ = 160;
        double reverse_scale_b_ = -12;
        bool invert_throttle_ = false;
        bool invert_steering_ = false;

        int servo_scale = 900;
        int servo_bias = 50;
        int servo_upper_bound = 950;
        int servo_lower_bound = 50;
        bool DEBUG = false;

        char cmd[18];

        // subscribers
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr manual_override_sub_;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_control_sub_;

        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr manual_override_status_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_status_pub_;

        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arming_server_;

        size_t count_;
};

int VescInterface::steering_cmd = 500;
int VescInterface::velo_cmd = 0;

int VescInterface::velo_cmd_ratio = 100;
int VescInterface::servo_cmd_ratio = 100;
int VescInterface::publish_ratio = 10;
int VescInterface::rpm_scale = 8000;
bool VescInterface::arm_status_ = true;


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VescInterface>());
  rclcpp::shutdown();
  return 0;
}