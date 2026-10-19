

#include "lio/params.h"

using namespace std;
using namespace BASIC;

namespace LI2Sup{
  double g_max_scan_duration = 0.2;

  const std::string g_root_dir = std::string(ROOT);
  std::atomic<bool> g_flag_run = true;
  bool g_flg_map_init = true;

  /// evaluation
  bool g_time_evo = false;

  bool   g_save_map = false;
  bool   g_if_filter = true;
  string g_save_map_dir = "outputs/map";
  string g_map_name = "map.pcd";
  float  g_map_ds_size = 0.25f;
  int    g_pcd_save_interval = 500;

  string g_imu_topic = "/livox/imu";
  string g_lidar_topic = "/livox/lidar";

  int    g_lidar_type = 1;
  float  g_blind2 = 0.64f;
  float  g_maxrange2 = 3600.0f;
  int    g_filter_rate = 3;
  bool   g_enable_downsample = true;
  float  g_voxel_fliter_size = 0.5f;

  int    g_imu_type = 1;
  double g_gravity_norm = 9.7946;
  double g_imu_na = 0.1;
  double g_imu_ng = 0.1;
  double g_imu_nba = 0.0001;
  double g_imu_nbg = 0.0001;

  SE3 g_lidar_imu;
  SE3 g_odom_robo;
  M3  g_lidar_robo_yaw;

  /// hash_map
  std::size_t g_ivox_capacity = 100000;
  float       g_ivox_resolution = 0.5;

  /// kf
  int g_kf_type = 1;                // 1: ESKF, 2: InESKF
  int g_kf_max_iterations = 4;
  bool g_kf_align_gravity = true;
  double g_kf_quit_eps = 0.001;
  int g_imu_int_frame = 50;         // fallback for yamls without kf/imu_int_frame
  bool g_dyn_filter_before_observe = false; // default: filter after Observe(), as before

  /// submap
  double g_submap_resolution = 20.0;
  int    g_submap_capacity = 100;

  /// output
  bool g_2_robot    = false;
  bool g_2_plan_env_world = false;
  bool g_2_plan_env_body  = false;
  bool g_2_ml_map = false;
  bool g_visual_map = true;
  bool g_visual_dense = false;
  int  g_pub_step = 1;

  /// for planner
  bool g_planner_enable = false;

  ResidualType g_residual_type = PROB;

  /// for relocation
  bool g_update_map = false;
  double g_init_px, g_init_py, g_init_pz, g_init_roll, g_init_pitch, g_init_yaw;

  /// pose output
  bool g_pose_output_en = false;
  std::string g_seq_name = "trajectory";
  std::string g_pose_output_dir = "outputs/trajectory";

}
