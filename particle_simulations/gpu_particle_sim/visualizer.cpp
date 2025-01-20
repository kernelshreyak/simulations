/**
 * This C++ program visualizes particle trajectories from a .npy file using OpenGL.
 *
 * The program loads trajectory data, calculates particle velocities,
 * and displays the motion using colored particle points in a 3D space.
 *
 * Key Features:
 * 1. Loads and processes 3D particle trajectories from a NumPy (.npy) file.
 * 2. Computes velocities for color mapping to indicate speed.
 * 3. Renders a point cloud of particles where color intensity represents velocity,
 *    using OpenGL and GLFW for graphics and window management.
 * 4. Provides user interaction capabilities such as mouse-based camera rotation
 *    and zooming using the scroll wheel.
 * 5. Continuously updates and displays particle frames with a specified frame delay.
 *
 * Libraries Used:
 * - cnpy: For loading .npy files.
 * - OpenGL/GLFW: For rendering the 3D point cloud.
 * - GLEW: For managing OpenGL extensions.
 *
 * User Controls:
 * - Arrow keys to rotate the view.
 * - Mouse drag for additional rotation.
 * - Scroll for zooming in and out.
 *
 * This tool is valuable for visualizing and analyzing particle movement data
 * in simulations or experimental results.
 */

#include <algorithm>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <cnpy.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>

// Globals for camera interaction
float camera_zoom = 45.0f;
float camera_x = 0.0f, camera_y = 0.0f;
float rotation_angle = 0.0f;

bool is_mouse_pressed = false;


// Function to normalize velocities for color mapping
void normalizeVelocities(std::vector<float>& velocities) {
    float max_velocity = *std::max_element(velocities.begin(), velocities.end());
    if (max_velocity > 0) {
        for (auto& v : velocities) {
            v /= max_velocity; // Normalize to range [0, 1]
        }
    }
}

// Function to load the .npy file and extract data
bool loadTrajectories(const std::string& filename, std::vector<std::vector<float>>& trajectories, int& num_frames, int& num_particles) {
    cnpy::NpyArray arr = cnpy::npy_load(filename);
    float* data = arr.data<float>();
    std::vector<size_t> shape = arr.shape;

    if (shape.size() != 3 || shape[2] != 3) {
        std::cerr << "Invalid .npy file shape. Expected (num_frames, num_particles, 3)." << std::endl;
        return false;
    }

    num_frames = shape[0];
    num_particles = shape[1];
    trajectories.resize(num_frames, std::vector<float>(num_particles * 3));

    for (int frame = 0; frame < num_frames; ++frame) {
        for (int particle = 0; particle < num_particles; ++particle) {
            for (int coord = 0; coord < 3; ++coord) {
                trajectories[frame][particle * 3 + coord] = data[frame * num_particles * 3 + particle * 3 + coord];
            }
        }
    }

    return true;
}

// Function to compute velocities between consecutive frames
std::vector<std::vector<float>> computeVelocities(const std::vector<std::vector<float>>& trajectories, int num_frames, int num_particles) {
    std::vector<std::vector<float>> velocities(num_frames, std::vector<float>(num_particles, 0.0f));

    for (int frame = 1; frame < num_frames; ++frame) {
        for (int particle = 0; particle < num_particles; ++particle) {
            float dx = trajectories[frame][particle * 3] - trajectories[frame - 1][particle * 3];
            float dy = trajectories[frame][particle * 3 + 1] - trajectories[frame - 1][particle * 3 + 1];
            float dz = trajectories[frame][particle * 3 + 2] - trajectories[frame - 1][particle * 3 + 2];
            velocities[frame][particle] = std::sqrt(dx * dx + dy * dy + dz * dz);
        }
    }

    return velocities;
}

// GLFW callbacks for camera manipulation
void scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    camera_zoom -= yoffset;
    if (camera_zoom < 1.0f) camera_zoom = 1.0f;
    if (camera_zoom > 90.0f) camera_zoom = 90.0f;
}

void cursorPositionCallback(GLFWwindow* window, double xpos, double ypos) {
    static double last_x = xpos, last_y = ypos;

    if (is_mouse_pressed) {
        double dx = xpos - last_x;
        rotation_angle += dx * 0.1f; // Adjust rotation speed as needed
    }

    last_x = xpos;
    last_y = ypos;
}

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) {
            is_mouse_pressed = true;
        } else if (action == GLFW_RELEASE) {
            is_mouse_pressed = false;
        }
    }
}


void renderPointCloud(const std::vector<std::vector<float>>& trajectories, const std::vector<std::vector<float>>& velocities, int num_frames, int num_particles, float frame_delay) {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW." << std::endl;
        return;
    }

    GLFWwindow* window = glfwCreateWindow(1920, 1080, "Particle Simulation", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window." << std::endl;
        glfwTerminate();
        return;
    }

    glfwMakeContextCurrent(window);
    glewInit();

    glfwSetScrollCallback(window, scrollCallback);
    glfwSetCursorPosCallback(window, cursorPositionCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);

    glEnable(GL_DEPTH_TEST);
    glPointSize(3.0f);

    while (!glfwWindowShouldClose(window)) {
        for (int frame = 0; frame < num_frames; ++frame) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // White background
            glLoadIdentity();

            // Set up camera
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            gluPerspective(camera_zoom, 1024.0 / 768.0, 0.1, 100.0);
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            gluLookAt(camera_x, camera_y, 10.0, camera_x, camera_y, 0.0, 0.0, 1.0, 0.0);

            // Rotate scene
            glRotatef(rotation_angle, 0.0f, 1.0f, 0.0f);

            // Render particles
            glBegin(GL_POINTS);
            for (int particle = 0; particle < num_particles; ++particle) {
                float velocity = velocities[frame][particle];
                glColor3f(velocity, 0.0f, 1.0f - velocity); // Map velocity to color (red -> high, blue -> low)

                float x = trajectories[frame][particle * 3];
                float y = trajectories[frame][particle * 3 + 1];
                float z = trajectories[frame][particle * 3 + 2];
                glVertex3f(x, y, z);
            }
            glEnd();

            glfwSwapBuffers(window);
            glfwPollEvents();

            // Delay for visualization
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(frame_delay * 1000)));

            // Handle user input for rotation
            if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) rotation_angle -= 1.0f;
            if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) rotation_angle += 1.0f;
        }
    }

    glfwDestroyWindow(window);
    glfwTerminate();
}

int main() {
    std::string npy_file = "./trajectories.npy";
    int num_frames, num_particles;
    std::vector<std::vector<float>> trajectories;

    // Load trajectories from .npy file
    if (!loadTrajectories(npy_file, trajectories, num_frames, num_particles)) {
        return -1;
    }

    // Compute velocities
    std::vector<std::vector<float>> velocities = computeVelocities(trajectories, num_frames, num_particles);

    // Normalize velocities for color mapping
    for (auto& frame_velocities : velocities) {
        normalizeVelocities(frame_velocities);
    }

    // Render point cloud
    float frame_delay = 0.05f;
    renderPointCloud(trajectories, velocities, num_frames, num_particles, frame_delay);

    return 0;
}
