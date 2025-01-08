#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <cstdlib>
#include <string>
#include <memory>
#include <vector>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

class SwitchMode : public rclcpp::Node {
public:
    SwitchMode() : Node("switch_mode"), current_process_pid(-1), current_mode("") {
        // Get robot model from environment variable or use default
        const char *env_p = std::getenv("ROM_ROBOT_MODEL");
        robot_model = (env_p != nullptr) ? std::string(env_p) : "rom2109";

        // Initialize paths to launch files
        nav2_launch_file = robot_model + "_nav2/navigation.launch.py";
        mapping_launch_file = robot_model + "_nav2/mapping.launch.py";
        remapping_launch_file = robot_model + "_nav2/remapping.launch.py";

        // Subscription to the "switch_mode" topic
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "switch_mode",
            10,
            std::bind(&SwitchMode::modeCallback, this, std::placeholders::_1)
        );
    }

    ~SwitchMode() {
        // Ensure the process is cleaned up on node destruction
        terminateCurrentProcess();
    }

private:
    void modeCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string new_mode = msg->data;

        if (new_mode == current_mode) {
            RCLCPP_INFO(this->get_logger(), "Mode is already '%s'. No change needed.", new_mode.c_str());
            return;
        }

        // Terminate the current process if any
        terminateCurrentProcess();

        // Launch the new process based on the requested mode
        if (new_mode == "navi") {
            RCLCPP_INFO(this->get_logger(), "Switching to navigation mode...");
            launchProcess(nav2_launch_file);
        } else if (new_mode == "mapping") {
            RCLCPP_INFO(this->get_logger(), "Switching to mapping mode...");
            launchProcess(mapping_launch_file);
        } else if (new_mode == "remapping") {
            RCLCPP_INFO(this->get_logger(), "Switching to remapping mode...");
            launchProcess(remapping_launch_file);
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid mode: '%s'. No action taken.", new_mode.c_str());
        }

        current_mode = new_mode;
    }

    void launchProcess(const std::string &launch_file) {
        // Fork and execute the launch file in the child process
        pid_t pid = fork();
        if (pid == 0) {
            // Child process
            execlp("ros2", "ros2", "launch", launch_file.c_str(), nullptr);
            // If execlp fails
            perror("execlp");
            std::exit(EXIT_FAILURE);
        } else if (pid > 0) {
            // Parent process
            current_process_pid = pid;
            RCLCPP_INFO(this->get_logger(), "Launched process with PID: %d", pid);
        } else {
            // Fork failed
            RCLCPP_ERROR(this->get_logger(), "Failed to fork a new process.");
        }
    }

    void terminateCurrentProcess() {
        if (current_process_pid > 0) {
            RCLCPP_INFO(this->get_logger(), "Terminating process with PID: %d", current_process_pid);
            kill(current_process_pid, SIGTERM);
            waitpid(current_process_pid, nullptr, 0);  // Wait for the process to terminate
            current_process_pid = -1;
            RCLCPP_INFO(this->get_logger(), "Process terminated.");
        }
    }

    // Member variables
    std::string robot_model;
    std::string nav2_launch_file;
    std::string mapping_launch_file;
    std::string remapping_launch_file;
    std::string current_mode;
    pid_t current_process_pid;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SwitchMode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
