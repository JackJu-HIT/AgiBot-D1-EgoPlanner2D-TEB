/**
    MIT License
    Copyright (c) 2025 Chunyu Ju
    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
 */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>

class KeyboardReader {
public:
    KeyboardReader() {
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    }

    ~KeyboardReader() {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, F_SETFL, oldf);
    }

    int readKey() {
        int ch = getchar();
        if (ch != EOF) {
            return ch;
        }
        return -1;
    }

private:
    struct termios oldt, newt;
    int oldf;
};

class DogKeyboardNode : public rclcpp::Node {
public:
    DogKeyboardNode() : Node("dog_keyboard_node") {
        pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        pub_action_ = this->create_publisher<std_msgs::msg::String>("cmd_action", 10);

        printUsage();
        
        // 使用高频定时器检测按键 (50Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&DogKeyboardNode::keyLoop, this));
    }

      // 【关键 4】这就是析构函数！它负责在节点关闭时回收线程
    ~DogKeyboardNode() override 
    {
        RCLCPP_INFO(this->get_logger(), "Stopping remote kyboard, and make robot Stop!...");
        publishVel(0.0, 0.0, 0.0);   
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_action_;
    rclcpp::TimerBase::SharedPtr timer_;
    KeyboardReader kb_reader_;

    void printUsage() {
        std::cout << "=========================================" << std::endl;
        std::cout << "Robot Dog Keyboard Controller" << std::endl;
        std::cout << "-----------------------------------------" << std::endl;
        std::cout << "Movement:" << std::endl;
        std::cout << "  w: Forward   (1.0 m/s)" << std::endl;
        std::cout << "  s: Backward  (-1.0 m/s)" << std::endl;
        std::cout << "  a: Left      (1.0 m/s)" << std::endl;
        std::cout << "  d: Right     (-1.0 m/s)" << std::endl;
        std::cout << "  q: Turn Left (1.0 rad/s)" << std::endl;
        std::cout << "  e: Turn Right(-1.0 rad/s)" << std::endl;
        std::cout << "  c: Stop Move" << std::endl;
        std::cout << "Actions:" << std::endl;
        std::cout << "  0: Passive/Stop" << std::endl;
        std::cout << "  1: Lie Down" << std::endl;
        std::cout << "  2: Stand Up" << std::endl;
        std::cout << "  3: Jump" << std::endl;
        std::cout << "  4: Front Jump" << std::endl;
        std::cout << "  6: Backflip" << std::endl;
        std::cout << "  7: Attitude Demo" << std::endl;
        std::cout << "  8: Shake Hand" << std::endl;
        std::cout << "  9: Two Leg Stand" << std::endl;
        std::cout << "  z: Cancel Two Leg" << std::endl;
        std::cout << "Ctrl+C to quit" << std::endl;
        std::cout << "=========================================" << std::endl;
    }

    void publishVel(float x, float y, float z) {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = x;
        msg.linear.y = y;
        msg.angular.z = z;
        pub_vel_->publish(msg);
    }

    void publishAction(const std::string& action) {
        auto msg = std_msgs::msg::String();
        msg.data = action;
        pub_action_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Sent action: %s", action.c_str());
        // 重新打印菜单以防被日志刷屏
        // printUsage(); 
    }

    void keyLoop() {
        int ch = kb_reader_.readKey();
        if (ch == -1) return;

        switch (ch) {
            // Movement
            case 'w': publishVel(0.5, 0.0, 0.0); break;
            case 's': publishVel(-0.5, 0.0, 0.0); break;
            case 'a': publishVel(0.0, 0.10, 0.0); break;
            case 'd': publishVel(0.0, -0.10, 0.0); break;
            case 'q': publishVel(0.0, 0.0, 0.30); break;
            case 'e': publishVel(0.0, 0.0, -0.30); break;
            case 'c': publishVel(0.0, 0.0, 0.0); break;

            // Actions
            case '0': publishAction("passive"); break;
            case '1': publishAction("lie_down"); break;
            case '2': publishAction("stand_up"); break;
            case '3': publishAction("jump"); break;
            case '4': publishAction("front_jump"); break;
            case '6': publishAction("backflip"); break;
            case '7': publishAction("attitude_demo"); break;
            case '8': publishAction("shake_hand"); break;
            case '9': publishAction("two_leg_stand"); break;
            case 'z': publishAction("cancel_two_leg"); break;

            default: break;
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DogKeyboardNode>());
    rclcpp::shutdown();
    return 0;
}