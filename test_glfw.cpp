#include <GLFW/glfw3.h>
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    if (!glfwInit()) {
        std::cerr << "Failed to init GLFW" << std::endl;
        return -1;
    }
    
    std::cout << "Starting RAW GLFW Axis dump (Test for 5 seconds)..." << std::endl;
    for (int i = 0; i < 50; i++) {
        glfwPollEvents();
        bool found = false;
        for (int j = 0; j <= GLFW_JOYSTICK_LAST; j++) {
            if (glfwJoystickPresent(j)) {
                found = true;
                const char* name = glfwGetJoystickName(j);
                int count = 0;
                const float* axes = glfwGetJoystickAxes(j, &count);
                int btnCount = 0;
                const unsigned char* btns = glfwGetJoystickButtons(j, &btnCount);
                
                std::cout << "[" << name << "] ";
                if (count == 0) std::cout << "NO AXES. ";
                else {
                    for (int a = 0; a < count; a++) {
                        if (std::abs(axes[a]) > 0.05f) {
                            std::cout << "AX" << a << ":" << axes[a] << "  ";
                        }
                    }
                }
                
                if (btnCount > 0) {
                    for (int b = 0; b < btnCount; b++) {
                        if (btns[b]) std::cout << "BTN" << b << "  ";
                    }
                }
                std::cout << std::endl;
            }
        }
        if (!found) {
            std::cout << "No joysticks..." << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    glfwTerminate();
    return 0;
}
