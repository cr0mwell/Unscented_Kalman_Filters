/* \authors: Aaron Brown, Oleksandr Kashkevich*/
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "highway.h"


template<typename Type>
void processInput(Type &option) {
    std::cin >> option;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

int getIntOption(string message, int min_, int max_) {
    int option;
    std::cout << message;
    processInput(option);
    
    while (typeid(option) != typeid(int) || option < min_ || option > max_) {
        std::cout << "\nInvalid value entered, please enter an integer value in " << min_ << "-" << max_ << " range: " << std::endl;
        processInput(option);
    }
    
    return option;
}

bool getBoolOption(string message)
{
    char option;
    std::cout << message;
    processInput(option);
    
    while (std::tolower(option) != 'y' && std::tolower(option) != 'n') {
        std::cout << "\nInvalid value entered, please enter 'y' or 'n': ";
        processInput(option);
    }
    
    return (option == 'y');
}

int main(int argc, char** argv)
{
    // Set environment variables
    bool use_lidar_ukf = getBoolOption("Use Unscented Kalman filter for the LiDAR measurement updates (y) or its simple variant (n)? ");
    int prediction_time = getIntOption("Time period to predict using Kalman filter (1-3 sec, integer): ", 1, 3);
    int prediction_frequency = getIntOption("Prediction frequency (1-30 Hz): ", 1, 30);
    
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    // Set camera position and angle
    viewer->initCameraParameters();
    float x_pos = 0;
    viewer->setCameraPosition ( x_pos-26, 0, 15.0, x_pos+25, 0, 0, 0, 0, 1);
    
    Highway highway(viewer);

    int frame_per_sec = 30;
    int sec_interval = 10;
    int frame_count = 0;
    int time_us = 0;
    double egoVelocity = 25;
    
    size_t step = 1;
    while (frame_count < (frame_per_sec*sec_interval))
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        std::cout << "STEP " << step << std::endl;
        step++;
        highway.stepHighway(egoVelocity, time_us, frame_per_sec, prediction_time, prediction_frequency, viewer, use_lidar_ukf);
        viewer->spinOnce(1000/frame_per_sec);
        frame_count++;
        time_us = 1000000*frame_count/frame_per_sec;
    }
}