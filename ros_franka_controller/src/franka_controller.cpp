
#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <string>

#include <franka/exception.h>
#include <franka/robot.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "motion_generator.hpp"

#define MAX_JOINT_ANGLE_DIFF 0.1

using std::placeholders::_1;
using namespace std::chrono_literals;

const double REST_POSE[N_DOF] = {
    0.00,
    -0.25 * M_PI,
    0.00,
    -0.75 * M_PI,
    0.00,
    0.50 * M_PI,
    0.25 * M_PI,
};

const double LOWER_LIMITS[N_DOF] = {
    -2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159};

const double UPPER_LIMITS[N_DOF] = {
    2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159};

const std::string DOF_NAMES[N_DOF] = {
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7"};

static int move_robot(RobotJointState &out_state, std::mutex &state_mtx, std::array<double, N_DOF> &target_joint_pos, bool &shutdown, std::string robot_ip)
{

    try
    {
        franka::Robot robot(robot_ip);

        robot.setCollisionBehavior(
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
        robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
        robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

        std::array<double, 7> q_goal = {{REST_POSE[0], REST_POSE[1], REST_POSE[2], REST_POSE[3], REST_POSE[4], REST_POSE[5], REST_POSE[6]}};
        std::cout << "moving robot to rest pose" << std::endl;
        MotionGenerator motion_generator(1.0, q_goal, &out_state, &state_mtx);
        robot.control(motion_generator);
        std::cout << "done moving robot to rest pose" << std::endl;
        std::this_thread::sleep_for(500ms);

        bool exit_flag = false;

        while (!(exit_flag))
        {
            {
                const std::lock_guard<std::mutex> lock(state_mtx);
                exit_flag = shutdown;
                for (unsigned int i = 0; i < N_DOF; i++)
                {
                    q_goal[i] = target_joint_pos[i];
                    double joint_diff = q_goal[i] - out_state.pos[i];
                    if (abs(joint_diff) > MAX_JOINT_ANGLE_DIFF)
                    {
                        double sign_var = 0.0;
                        if (joint_diff < 0.0)
                        {
                            sign_var = -1.0;
                        }
                        else
                        {
                            sign_var = 1.0;
                        }
                        double new_change = std::min(0.01, 0.1 * abs(joint_diff));
                        q_goal[i] = new_change * sign_var + out_state.pos[i];
                    }

                    if (q_goal[i] < LOWER_LIMITS[i])
                    {
                        q_goal[i] = LOWER_LIMITS[i];
                    }
                    else if (q_goal[i] > UPPER_LIMITS[i])
                    {
                        q_goal[i] = UPPER_LIMITS[i];
                    }
                }
            }

            MotionGenerator motion_generator(1.0, q_goal, &out_state, &state_mtx);
            robot.control(motion_generator);
            std::this_thread::sleep_for(500ms);
        }
    }
    catch (const franka::Exception &e)
    {
        std::cout << e.what() << std::endl;
        return -1;
    }
    return 0;
}

class FrankaRosController : public rclcpp::Node
{
public:
    FrankaRosController()
        : Node("franka_ros_controller")
    {

        this->declare_parameter("franka_address", "");
        franka_address_ = this->get_parameter("franka_address").as_string();
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("target_joint_states", 10, std::bind(&FrankaRosController::joint_state_cb, this, _1));
        for (unsigned int i = 0; i < N_DOF; i++)
        {
            target_joint_pos_[i] = REST_POSE[i];
            franka_state_.pos[i] = 0.0;
            franka_state_.vel[i] = 0.0;
            franka_state_.tau[i] = 0.0;
        }
        shutdown_flag_ = false;

        robot_thread_ = std::thread(move_robot, std::ref(franka_state_), std::ref(franka_state_mtx_), std::ref(target_joint_pos_), std::ref(shutdown_flag_), franka_address_);
        timer_ = this->create_wall_timer(
            10ms, std::bind(&FrankaRosController::timer_callback, this));
    }

    void set_shutdown()
    {
        std::cout << "setting shutdown" << std::endl;
        const std::lock_guard<std::mutex> lock(franka_state_mtx_);
        shutdown_flag_ = true;
        std::cout << "set shutdown" << std::endl;
    }

    void wait_robot_exit()
    {
        robot_thread_.join();
    }

private:
    void joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (unsigned int i = 0; i < N_DOF; i++)
        {
            const std::string &name = DOF_NAMES[i];

            for (unsigned int j = 0; j < msg->name.size(); j++)
            {
                const std::string &j_name = msg->name[j];
                if (name == j_name)
                {
                    const double &new_pos = msg->position[j];

                    {
                        const std::lock_guard<std::mutex> lock(franka_state_mtx_);
                        target_joint_pos_[i] = new_pos;
                    }
                }
            }
        }

        return;
    }

    void timer_callback()
    {

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->get_clock()->now();
        for (unsigned int i = 0; i < N_DOF; i++)
        {
            msg.name.push_back(DOF_NAMES[i]);
        }

        {
            const std::lock_guard<std::mutex> lock(franka_state_mtx_);
            for (unsigned int i = 0; i < N_DOF; i++)
            {
                msg.position.push_back(franka_state_.pos[i]);
                msg.velocity.push_back(franka_state_.vel[i]);
                msg.effort.push_back(franka_state_.tau[i]);
            }
        }
        publisher_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string franka_address_;
    std::array<double, N_DOF> target_joint_pos_;

    RobotJointState franka_state_;
    std::mutex franka_state_mtx_;
    bool shutdown_flag_;
    std::thread robot_thread_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<FrankaRosController> franka_ptr = std::make_shared<FrankaRosController>();
    rclcpp::spin(franka_ptr);
    franka_ptr->set_shutdown();
    franka_ptr->wait_robot_exit();
    rclcpp::shutdown();
    return 0;
}