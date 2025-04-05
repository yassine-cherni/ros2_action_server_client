#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "custom_interfaces/action/wait.hpp"
#include <chrono>
#include <mutex>

using Wait = custom_interfaces::action::Wait;
using GoalHandleWait = rclcpp_action::ServerGoalHandle<Wait>;

class ActionServer : public rclcpp::Node
{
public:
    ActionServer() : Node("action_server", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        action_server_ = rclcpp_action::create_server<Wait>(
            this, "wait_action",
            std::bind(&ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&ActionServer::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Action server initialized with ROS 2 Jazzy");
    }

private:
    rclcpp_action::Server<Wait>::SharedPtr action_server_;
    std::shared_ptr<GoalHandleWait> current_goal_handle_;
    std::mutex mutex_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Wait::Goal> goal)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_goal_handle_ && current_goal_handle_->is_active())
        {
            if (goal->priority > current_goal_handle_->get_goal()->priority)
            {
                current_goal_handle_->abort(make_result(current_goal_handle_->get_goal()->goal_id, 
                                                       false, "Preempted by higher priority goal"));
                RCLCPP_WARN(this->get_logger(), "Preempting active goal %d for higher priority %d",
                           current_goal_handle_->get_goal()->goal_id, goal->goal_id);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Rejecting goal %d: lower priority than active goal", goal->goal_id);
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Accepting goal %d with priority %d", goal->goal_id, goal->priority);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleWait> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Cancel request received for goal %d", goal_handle->get_goal()->goal_id);
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleWait> goal_handle)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            current_goal_handle_ = goal_handle;
        }
        std::thread([this, goal_handle]() { this->execute_goal(goal_handle); }).detach();
    }

    void execute_goal(const std::shared_ptr<GoalHandleWait> goal_handle)
    {
        const int total_duration = 5;  // 5 seconds
        auto feedback = std::make_shared<Wait::Feedback>();
        auto result = std::make_shared<Wait::Result>();
        int goal_id = goal_handle->get_goal()->goal_id;
        
        rclcpp::Rate rate(1);  // 1 Hz feedback
        for (int i = 0; i < total_duration; ++i)
        {
            if (!rclcpp::ok() || goal_handle->is_canceling())
            {
                result = make_result(goal_id, false, "Goal canceled by client");
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal %d canceled after %d seconds", goal_id, i);
                cleanup_goal();
                return;
            }

            feedback->status = "Processing";
            feedback->progress = static_cast<float>(i + 1) / total_duration;
            feedback->seconds_remaining = total_duration - (i + 1);
            goal_handle->publish_feedback(feedback);
            rate.sleep();
        }

        result = make_result(goal_id, true, "Goal completed successfully");
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal %d completed successfully", goal_id);
        cleanup_goal();
    }

    std::shared_ptr<Wait::Result> make_result(int32_t goal_id, bool success, const std::string& message)
    {
        auto result = std::make_shared<Wait::Result>();
        result->goal_id = goal_id;
        result->success = success;
        result->message = message;
        return result;
    }

    void cleanup_goal()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_goal_handle_ = nullptr;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}