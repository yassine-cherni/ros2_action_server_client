#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/int32.hpp"
#include <custom_interfaces/action/wait.hpp>
#include <memory>

class ActionClient : public rclcpp::Node
{
public:
    using Wait = custom_interfaces::action::Wait;
    
    ActionClient() : Node("action_client", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        client_ = rclcpp_action::create_client<Wait>(this, "wait_action");
        subscription_ = create_subscription<std_msgs::msg::Int32>(
            "goal_topic", 10,
            std::bind(&ActionClient::topic_callback, this, std::placeholders::_1));
            
        this->declare_parameter("default_priority", 1);
        default_priority_ = this->get_parameter("default_priority").as_int();
        
        RCLCPP_INFO(this->get_logger(), "Action client started with ROS 2 Jazzy, listening to goal_topic");
    }

private:
    rclcpp_action::Client<Wait>::SharedPtr client_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    rclcpp_action::ClientGoalHandle<Wait>::SharedPtr current_goal_handle_;
    std::mutex mutex_;
    int default_priority_;

    void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_goal_handle_ && 
            (current_goal_handle_->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
             current_goal_handle_->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING))
        {
            client_->async_cancel_goal(current_goal_handle_);
            RCLCPP_INFO(this->get_logger(), "Requesting cancellation of previous goal for new goal %d", msg->data);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Give time for cancellation
        }
        send_goal(msg->data, default_priority_);
    }

    void send_goal(int goal_id, int priority)
    {
        if (!client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after 5s timeout");
            return;
        }

        auto goal_msg = Wait::Goal();
        goal_msg.goal_id = goal_id;
        goal_msg.priority = priority;

        auto send_goal_options = rclcpp_action::Client<Wait>::SendGoalOptions();
        send_goal_options.feedback_callback = 
            [this](rclcpp_action::ClientGoalHandle<Wait>::SharedPtr,
                  const std::shared_ptr<const Wait::Feedback> feedback) {
                RCLCPP_INFO(this->get_logger(), "Feedback: %s, Progress: %.1f%%, Remaining: %ds",
                           feedback->status.c_str(), feedback->progress * 100, feedback->seconds_remaining);
            };

        send_goal_options.result_callback = 
            [this](const rclcpp_action::ClientGoalHandle<Wait>::WrappedResult &result) {
                std::lock_guard<std::mutex> lock(mutex_);
                current_goal_handle_ = nullptr;
                log_result(result);
            };

        send_goal_options.goal_response_callback = 
            [this, goal_id](const rclcpp_action::ClientGoalHandle<Wait>::SharedPtr &goal_handle) {
                std::lock_guard<std::mutex> lock(mutex_);
                current_goal_handle_ = goal_handle;
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Goal %d was rejected by server", goal_id);
                } else {
                    RCLCPP_INFO(this->get_logger(), "Goal %d accepted by server", goal_id);
                }
            };

        RCLCPP_INFO(this->get_logger(), "Sending goal %d with priority %d", goal_id, priority);
        client_->async_send_goal(goal_msg, send_goal_options);
    }

    void log_result(const rclcpp_action::ClientGoalHandle<Wait>::WrappedResult &result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal %d succeeded: %s", 
                           result.result->goal_id, result.result->message.c_str());
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(this->get_logger(), "Goal %d canceled: %s", 
                           result.result->goal_id, result.result->message.c_str());
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_WARN(this->get_logger(), "Goal %d aborted: %s", 
                           result.result->goal_id, result.result->message.c_str());
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Goal %d failed with unknown result code", 
                            result.result->goal_id);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}