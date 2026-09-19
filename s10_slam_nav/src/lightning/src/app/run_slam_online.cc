//
// Created by xiang on 25-3-18.
//

#include <vector>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <csignal>

#include "core/system/slam.h"
#include "utils/timer.h"
#include "wrapper/bag_io.h"
#include "wrapper/ros_utils.h"

DEFINE_string(config, "./config/default.yaml", "配置文件");

// Global pointer to SlamSystem instance
lightning::SlamSystem* slam_instance = nullptr;

// Signal handler for SIGINT
void SignalHandler(int signum) {
    if (slam_instance) {
        LOG(INFO) << "SIGINT received, saving map and path...";
        slam_instance->SaveMap();
        slam_instance->SavePath();
    }
    rclcpp::shutdown();
    LOG(INFO) << "Shutdown complete.";
    std::exit(signum);
}

/// 运行一个LIO前端，带可视化
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    // ros2 launch appends "--ros-args" to node command lines; gflags rejects
    // the unknown flag.  Parse only the leading gflags tokens on a private
    // copy (ParseCommandLineFlags nulls removed slots, which would corrupt
    // argv for the rclcpp::init call below), and leave "--ros-args" intact.
    int gflags_argc = argc;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--ros-args") {
            gflags_argc = i;
            break;
        }
    }
    std::vector<char*> gflags_argv(argv, argv + gflags_argc);
    char** gflags_argv_ptr = gflags_argv.data();
    google::ParseCommandLineFlags(&gflags_argc, &gflags_argv_ptr, true);

    using namespace lightning;

    /// Initialize ROS2
    rclcpp::init(argc, argv);

    SlamSystem::Options options;
    options.online_mode_ = true;

    SlamSystem slam(options);
    slam_instance = &slam;  // Assign global pointer

    if (!slam.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init slam";
        return -1;
    }

    // Register the signal handler
    std::signal(SIGINT, SignalHandler);

    slam.StartSLAM("");
    slam.Spin();

    Timer::PrintAll();

    rclcpp::shutdown();

    LOG(INFO) << "done";

    return 0;
}