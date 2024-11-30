#include <GLFW/glfw3.h>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <unordered_map>
#include <iostream>

enum class RobotCommand {
    STOP = 0,
    MOVE_FORWARD = 1,
    MOVE_BACKWARD = 2,
    TURN_LEFT = 3,
    TURN_RIGHT = 4,
    TURN_LEFT_OPP = 5,
    TURN_RIGHT_OPP = 6,
    FASTER = 7,
    SLOWER = 8,
    INVALID = -1
};

static const std::unordered_map<std::string, RobotCommand> command_map = {
    {"x", RobotCommand::STOP},
    {"w", RobotCommand::MOVE_FORWARD},
    {"s", RobotCommand::MOVE_BACKWARD},
    {"a", RobotCommand::TURN_LEFT_OPP},
    {"d", RobotCommand::TURN_RIGHT_OPP},
    {"-", RobotCommand::SLOWER},
    {"+", RobotCommand::FASTER}
};

const std::unordered_map<std::string, std::string> command_descriptions = {
    {"x", "Stop the robot"},
    {"w", "Move forward"},
    {"s", "Move backward"},
    {"a", "Turn left"},
    {"d", "Turn right"},
    {"-", "Decrease Speed"},
    {"+", "Increase Speed"}
};

class RobotGUI : public rclcpp::Node {
    public:
        RobotGUI() : Node("robot_gui") {
            publisher_ = this->create_publisher<std_msgs::msg::String>("motor_control", 10);
        }
        void run();

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        std::string wait_for_user(GLFWwindow* window);
};

void RobotGUI::run() {
    if (!glfwInit()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize GLFW");
        return;
    }

    GLFWwindow *window = glfwCreateWindow(800, 600, "Slambot GUI", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return;
    }

    glfwMakeContextCurrent(window);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        std::string valid_user_input = wait_for_user(window);

        if (command_map.find(valid_user_input) == command_map.end()) {
            continue;  // Ignore invalid input
        }

        std_msgs::msg::String message;
        message.data = valid_user_input;

        try {
            std::string log_message = "Publishing: '" + command_descriptions.at(valid_user_input) + "'";
            RCLCPP_INFO(this->get_logger(), "%s", log_message.c_str());
        } catch (const std::out_of_range&) {
            RCLCPP_WARN(this->get_logger(), "Invalid input!");
        }

        publisher_->publish(message);
        glfwSwapBuffers(window);
    }

    glfwDestroyWindow(window);
    glfwTerminate();
}

std::string RobotGUI::wait_for_user(GLFWwindow* window) {
    // stop takes precedent
    static const std::unordered_map<int, std::string> key_map = {
        {GLFW_KEY_X, "x"},
        {GLFW_KEY_W, "w"},
        {GLFW_KEY_S, "s"},
        {GLFW_KEY_A, "a"},
        {GLFW_KEY_D, "d"},
        {GLFW_KEY_MINUS, "-"},
        {GLFW_KEY_LEFT_SHIFT, "-"},
        {GLFW_KEY_EQUAL, "+"},
        {GLFW_KEY_SPACE, "+"},
    };

    for (const auto& [key, action] : key_map) {
        if (glfwGetKey(window, key) == GLFW_PRESS) {
            return action;
        }
    }
    return "";  // Default if no valid key is pressed
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto gui = std::make_shared<RobotGUI>();
    gui->run();
    rclcpp::shutdown();
    return 0;
}
