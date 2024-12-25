#include <GLFW/glfw3.h>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <unordered_map>
#include <iostream>

enum class RobotMovement: char{
    STOP = 'x',
    MOVE_FORWARD = 'w',
    MOVE_BACKWARD = 's',
    MOVE_LEFT = 'l',
    MOVE_RIGHT = 'r',
    ROTATE_LEFT = 'a',
    ROTATE_RIGHT = 'd',
    DIAG_FORWARD_RIGHT = 'e',
    DIAG_BACKWARD_RIGHT = 'c',
    DIAG_FORWARD_LEFT = 'q',
    DIAG_BACKWARD_LEFT = 'z',
    FASTER = '+',
    SLOWER = '-',
    INVALID = '?'
};

static const std::unordered_map<std::string, RobotMovement> command_map = {
    {"x", RobotMovement::STOP},
    {"w", RobotMovement::MOVE_FORWARD},
    {"s", RobotMovement::MOVE_BACKWARD},
    {"a", RobotMovement::ROTATE_LEFT},
    {"d", RobotMovement::ROTATE_RIGHT},
    {"q", RobotMovement::DIAG_FORWARD_LEFT},
    {"e", RobotMovement::DIAG_FORWARD_RIGHT},
    {"z", RobotMovement::DIAG_BACKWARD_LEFT},
    {"c", RobotMovement::DIAG_FORWARD_RIGHT},
    {"-", RobotMovement::SLOWER},
    {"+", RobotMovement::FASTER}
};

const std::unordered_map<std::string, std::string> command_descriptions = {
    {"x", "Stop the robot"},
    {"w", "Move forward"},
    {"s", "Move backward"},
    {"a", "Turn left"},
    {"d", "Turn right"},
    {"q", "Move diagonally forward left"},
    {"e", "Move diagonally forward right"},
    {"z", "Move diagonally backward left"},
    {"c", "Move diagonally forward right"},
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
void glfwErrorCallback(int error, const char* description) {
    std::cerr << "GLFW Error [" << error << "]: " << description << std::endl;
}

void RobotGUI::run() {

    if (!glfwInit()) {
        glfwSetErrorCallback(glfwErrorCallback); 
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
        {GLFW_KEY_Q, "q"},
        {GLFW_KEY_E, "e"},
        {GLFW_KEY_C, "c"},
        {GLFW_KEY_Z, "z"},
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
