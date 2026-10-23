/*
 * @Description: ROS-agnostic IMU/camera scheduling shared by ROS1Visualizer and ROS2Visualizer.
 *               Cameras are rate limited and queued; each IMU triggers draining of the camera
 *               queue for all images older than that IMU (time-offset corrected).
 */

#ifndef OV_MSCKF_MEASUREMENT_SCHEDULER_H
#define OV_MSCKF_MEASUREMENT_SCHEDULER_H

#include <atomic>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>

#include "utils/sensor_data.h"

namespace ov_msckf {

class VioManager;

class MeasurementScheduler {

public:
  /**
   * @param app Core estimator manager
   * @param on_camera_update Called after each processed camera measurement (e.g. visualize())
   * @param on_imu Called after each fed IMU measurement with its timestamp (e.g. visualize_odometry())
   */
  MeasurementScheduler(std::shared_ptr<VioManager> app, std::function<void()> on_camera_update, std::function<void(double)> on_imu);

  /// Waits for an in-flight (detached) update thread to finish
  ~MeasurementScheduler();

  /// Feed IMU, run odometry callback, then drain time-aligned camera measurements (possibly in a worker thread)
  void feed_imu(const ov_core::ImuData &message);

  /// Rate limit by track_frequency and enqueue (sorted by timestamp); processing is triggered by feed_imu()
  void feed_camera(const ov_core::CameraData &message);

private:
  void process_camera_queue(double timestamp_imu);

  std::shared_ptr<VioManager> _app;
  std::function<void()> _on_camera_update;
  std::function<void(double)> _on_imu;

  std::atomic<bool> thread_update_running{false};

  std::deque<ov_core::CameraData> camera_queue;
  std::mutex camera_queue_mtx;

  /// Last accepted camera timestamp per cam id (only touched from camera callbacks)
  std::map<int, double> camera_last_timestamp;
};

} // namespace ov_msckf

#endif // OV_MSCKF_MEASUREMENT_SCHEDULER_H
