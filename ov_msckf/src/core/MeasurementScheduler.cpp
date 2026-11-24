/*
 * @Description: ROS-agnostic IMU/camera scheduling shared by ROS1Visualizer and ROS2Visualizer.
 */

#include "MeasurementScheduler.h"

#include <algorithm>
#include <chrono>
#include <thread>
#include <vector>

#include <boost/date_time/posix_time/posix_time.hpp>

#include "core/VioManager.h"
#include "state/State.h"
#include "types/Vec.h"
#include "utils/print.h"

using namespace ov_msckf;

MeasurementScheduler::MeasurementScheduler(std::shared_ptr<VioManager> app, std::function<void()> on_camera_update,
                                           std::function<void(double)> on_imu)
    : _app(app), _on_camera_update(std::move(on_camera_update)), _on_imu(std::move(on_imu)) {}

MeasurementScheduler::~MeasurementScheduler() {
  while (thread_update_running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void MeasurementScheduler::feed_imu(const ov_core::ImuData &message) {

  // Feed IMU to propagator and initializer.
  // NOTE: this must be serialized against the camera-update thread below; the
  // initializer/propagator keep IMU samples in std::vector, so a concurrent
  // tracking update that scans/prunes that vector while we push_back into it
  // is a use-after-free (observed as SIGSEGV during VIO initialization).
  // Visualization stays OUTSIDE the lock so ROS publish / TF cannot stall IMU.
  {
    std::lock_guard<std::mutex> lck(vio_mtx_);
    _app->feed_measurement_imu(message);
  }
  if (_on_imu)
    _on_imu(message.timestamp);

  // If the processing thread is already running, return so we keep buffering
  bool expected = false;
  if (!thread_update_running.compare_exchange_strong(expected, true))
    return;

  const double timestamp_imu = message.timestamp;
  std::thread thread([this, timestamp_imu] {
    process_camera_queue(timestamp_imu);
    thread_update_running = false;
  });

  // Single-thread or detach based on config
  if (!_app->get_params().use_multi_threading_subs) {
    thread.join();
  } else {
    thread.detach();
  }
}

void MeasurementScheduler::process_camera_queue(double timestamp_imu) {

  // Snapshot the images that are now safe to process (older than the latest IMU),
  // then release the queue lock so the camera callback is not blocked by tracking.
  std::vector<ov_core::CameraData> pending;
  double timestamp_imu_inC = -1.0;
  {
    std::lock_guard<std::mutex> lck(camera_queue_mtx);

    // Gather distinct camera streams already in queue
    std::map<int, bool> unique_cam_ids;
    for (const auto &cam_msg : camera_queue) {
      unique_cam_ids[cam_msg.sensor_ids.at(0)] = true;
    }

    // For stereo, both images arrive in one CameraData (sensor_ids.size()==2),
    // so we only need one unique stream to proceed.
    auto params = _app->get_params();
    size_t num_unique_cameras = (params.state_options.num_cameras == 2) ? 1 : params.state_options.num_cameras;
    if (unique_cam_ids.size() != num_unique_cameras)
      return;

    {
      std::lock_guard<std::mutex> vio_lck(vio_mtx_);
      auto state = _app->get_state();
      if (!state || !state->_calib_dt_CAMtoIMU)
        return;
      // Drain camera queue: process each CameraData whose timestamp is older
      // than the latest IMU (IMU timestamp adjusted by cam-imu time offset).
      timestamp_imu_inC = timestamp_imu - state->_calib_dt_CAMtoIMU->value()(0);
    }

    while (!camera_queue.empty() && camera_queue.at(0).timestamp < timestamp_imu_inC) {
      pending.push_back(camera_queue.front());
      camera_queue.pop_front();
    }
  }

  for (const auto &cam_msg : pending) {
    auto rT0_1 = boost::posix_time::microsec_clock::local_time();
    double update_dt = 100.0 * (timestamp_imu_inC - cam_msg.timestamp);

    // Only the estimator mutation needs vio_mtx_. Publishing pose/features/path
    // under the same lock made IMU wait on RViz/DDS and amplified tracking stalls.
    {
      std::lock_guard<std::mutex> vio_lck(vio_mtx_);
      _app->feed_measurement_camera(cam_msg);
    }
    if (_on_camera_update)
      _on_camera_update();

    auto rT0_2 = boost::posix_time::microsec_clock::local_time();
    double time_total = (rT0_2 - rT0_1).total_microseconds() * 1e-6;
    PRINT_INFO(BLUE "[TIME]: %.4f seconds total (%.1f hz, %.2f ms behind)\n" RESET, time_total, 1.0 / time_total, update_dt);
  }
}

void MeasurementScheduler::feed_camera(const ov_core::CameraData &message) {

  // Rate limit: drop frames faster than track_frequency
  double timestamp = message.timestamp;
  double time_delta = 1.0 / _app->get_params().track_frequency;
  int cam_id = message.sensor_ids.at(0);
  auto it = camera_last_timestamp.find(cam_id);
  if (it != camera_last_timestamp.end() && timestamp < it->second + time_delta) {
    return;
  }
  camera_last_timestamp[cam_id] = timestamp;

  // Enqueue (sorted by timestamp)
  std::lock_guard<std::mutex> lck(camera_queue_mtx);
  camera_queue.push_back(message);
  std::sort(camera_queue.begin(), camera_queue.end());
}
