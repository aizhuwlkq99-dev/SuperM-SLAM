

#ifndef SUPER_LIO_H_
#define SUPER_LIO_H_

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <cassert>
#include <filesystem>
#include <iostream>
#include <queue>
#include <super_lio_interfaces/msg/frame_committed.hpp>
#include <super_lio_interfaces/msg/mapping_status.hpp>
#include <super_lio_mapping/store.hpp>
#include <vector>

#include "ESKF.h"
#include "OctVoxMap/OctVoxMap.hpp"
#include "OctVoxMap/VoxelGridFilter.h"
#include "basic/alias.h"
#include "common/ds.h"
#include "common/timer.h"
#include "lio/dyn_filter.h"
#include "params.h"
#include "ros/ROSWrapper.h"

namespace LI2Sup{

class SuperLIO{
public:
  SuperLIO(){};
  virtual ~SuperLIO() {
    if (frame_store_) {
      try {
        frame_store_->close();
      } catch (...) {
      }
    }
  };

  void setROSWrapper(const ROSWrapper::Ptr& wrapper){
    data_wrapper_ = wrapper;
  }
  virtual void init();
  void process();
  void saveMap();
  void printTimeRecord();

protected:
  void stateWaitKFInit();
  void stateWaitMapInit();
  void stateProcess();
  virtual bool kf_init();
  virtual bool map_init();
  void Propagation_Undistort();
  void DownSample();
  void FillBodyPoints();
  void Observe();
  void DynamicFilter();
  virtual void UpdateMap();
  virtual void Output();
  void caceData();
  void ProcessCaceMap();

  using StateFn = void (SuperLIO::*)();
  using OctVoxMapType = OctVoxMap<BASIC::V3, BASIC::scalar>;
  using KNNHeapType = KNNHeap<5, BASIC::V3>;
  StateFn state_fn_;
  ESKF::Ptr kf_;
  OctVoxMapType::Ptr ivox_;
  VoxelGridClosest<BASIC::PointType> voxel_grid_fliter_;
  ROSWrapper::Ptr data_wrapper_;
  MeasureGroup measures_;
  DynObjFilterWrapper::Ptr dyn_filter_;

  bool flg_init_ = false;
  bool flg_first_scan_ = true;
  std::vector<DynamicState> propagate_states_;
  BASIC::CloudPtr scan_undistort_full_;
  BASIC::CloudPtr ds_undistort_;
  BASIC::CloudPtr point_map_, world_pc_, ds_world_;
  BASIC::CloudPtr dyn_cloud_world_, steady_cloud_world_;
  int frame_num_ = 0;
  BASIC::SE3 sys_init_pose_;
  BASIC::SE3 last_pose_;

  std::size_t effect_knn_num_ = 0;
  BASIC::VV3 points_world_v3_, points_body_v3_;
  alignas(64) bool effect_mask_[200000] = {false};
  alignas(64) bool effect_knn_mask_[200000] = {false};
  std::vector<int> effect_knn_idxs_;
  std::vector<std::pair<BASIC::M6, BASIC::V6>> H_R_;
  std::vector<std::array<double, 4>> abcd_vec_;
  int pcd_index_ = -1;

  std::unique_ptr<superm::FrameStore> frame_store_;
  rclcpp::Publisher<super_lio_interfaces::msg::FrameCommitted>::SharedPtr commit_pub_;
  rclcpp::Publisher<super_lio_interfaces::msg::MappingStatus>::SharedPtr store_status_pub_;
  uint64_t stored_frame_id_ = 0;
  Timer time_record_;
  std::size_t processing_samples_ = 0;
  double processing_mean_ms_ = 0.0;
  double processing_m2_ = 0.0;
};

} // namespace END.

#endif


