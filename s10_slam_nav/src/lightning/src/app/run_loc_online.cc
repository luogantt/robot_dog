//
// Created by xiang on 25-3-18.
//

#include <vector>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "core/system/loc_system.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

DEFINE_string(config, "./config/default.yaml", "配置文件");

/// 运行定位的测试
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

    rclcpp::init(argc, argv);

    LocSystem::Options opt;
    // 默认开启，Init内会读取yaml覆盖这些值
    opt.pub_tf_ = true;
    opt.pub_odom_ = true;
    opt.log_pose_opt_ = true;
    LocSystem loc(opt);

    if (!loc.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init loc";
    }

    /// Init() loads YAML into LocSystem's internal options.  Use those loaded
    /// options rather than the stale local copy passed to the constructor.
    const auto& configured_opt = loc.GetOptions();
    if (configured_opt.use_init_pose_) {
        loc.SetInitPose(configured_opt.init_pose_);
    } else {
        loc.SetInitPose(SE3());
    }
    loc.Spin();

    rclcpp::shutdown();

    return 0;
}
