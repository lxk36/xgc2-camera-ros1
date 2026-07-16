#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/error.h>
#include <libswscale/swscale.h>
}

#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>

#include <xgc2/camera/camera.hpp>

#include "h264_input_buffer.hpp"

namespace {

using xgc2::camera::Frame;
using xgc2::camera::PixelFormat;

std::uint32_t positiveDimension(const int value, const char* name)
{
  if (value <= 0) {
    throw std::invalid_argument(std::string(name) + " must be positive");
  }
  return static_cast<std::uint32_t>(value);
}

cv::Mat packedMat(const Frame& frame, const int type, const std::size_t bytes_per_pixel)
{
  const std::size_t minimum_row = static_cast<std::size_t>(frame.width()) * bytes_per_pixel;
  const std::size_t stride = frame.stride() == 0 ? minimum_row : frame.stride();
  const std::size_t required = stride * frame.height();
  if (!frame.data() || stride < minimum_row || frame.size() < required) {
    throw std::runtime_error("camera frame has an invalid packed buffer layout");
  }
  return cv::Mat(static_cast<int>(frame.height()), static_cast<int>(frame.width()), type,
                 const_cast<std::uint8_t*>(frame.data()), stride);
}

cv::Mat nv12Mat(const Frame& frame)
{
  const std::size_t width = frame.width();
  const std::size_t height = frame.height();
  if ((width % 2U) != 0U || (height % 2U) != 0U) {
    throw std::runtime_error("NV12 width and height must be even");
  }

  cv::Mat packed(static_cast<int>(height + height / 2U), static_cast<int>(width), CV_8UC1);
  const auto& planes = frame.planes();
  if (planes.size() >= 2U) {
    const auto& y_plane = planes[0];
    const auto& uv_plane = planes[1];
    const std::size_t y_stride = y_plane.stride == 0 ? width : y_plane.stride;
    const std::size_t uv_stride = uv_plane.stride == 0 ? width : uv_plane.stride;
    if (!y_plane.data || !uv_plane.data || y_stride < width || uv_stride < width ||
        y_plane.bytes_used < y_stride * height || uv_plane.bytes_used < uv_stride * (height / 2U)) {
      throw std::runtime_error("camera frame has an invalid multi-plane NV12 layout");
    }
    for (std::size_t row = 0; row < height; ++row) {
      std::memcpy(packed.ptr(static_cast<int>(row)), y_plane.data + row * y_stride, width);
    }
    for (std::size_t row = 0; row < height / 2U; ++row) {
      std::memcpy(packed.ptr(static_cast<int>(height + row)), uv_plane.data + row * uv_stride, width);
    }
    return packed;
  }

  const std::size_t required = width * height * 3U / 2U;
  if (!frame.data() || frame.size() < required) {
    throw std::runtime_error("camera frame has an invalid contiguous NV12 layout");
  }
  std::memcpy(packed.data, frame.data(), required);
  return packed;
}

cv::Mat decodeToBgr(const Frame& frame)
{
  cv::Mat bgr;
  switch (frame.pixel_format()) {
    case PixelFormat::MJPEG: {
      if (!frame.data() || frame.size() == 0U) {
        throw std::runtime_error("empty MJPEG frame");
      }
      const int encoded_size =
          xgc_camera_driver::detail::checkedCompressedPayloadSize(frame.size(), "MJPEG");
      const cv::Mat encoded(1, encoded_size, CV_8UC1,
                            const_cast<std::uint8_t*>(frame.data()));
      bgr = cv::imdecode(encoded, cv::IMREAD_COLOR);
      if (bgr.empty()) {
        throw std::runtime_error("OpenCV could not decode MJPEG frame");
      }
      break;
    }
    case PixelFormat::BGR24:
      bgr = packedMat(frame, CV_8UC3, 3U).clone();
      break;
    case PixelFormat::RGB24:
      cv::cvtColor(packedMat(frame, CV_8UC3, 3U), bgr, cv::COLOR_RGB2BGR);
      break;
    case PixelFormat::YUYV:
      cv::cvtColor(packedMat(frame, CV_8UC2, 2U), bgr, cv::COLOR_YUV2BGR_YUY2);
      break;
    case PixelFormat::UYVY:
      cv::cvtColor(packedMat(frame, CV_8UC2, 2U), bgr, cv::COLOR_YUV2BGR_UYVY);
      break;
    case PixelFormat::NV12:
      cv::cvtColor(nv12Mat(frame), bgr, cv::COLOR_YUV2BGR_NV12);
      break;
    case PixelFormat::GREY:
      cv::cvtColor(packedMat(frame, CV_8UC1, 1U), bgr, cv::COLOR_GRAY2BGR);
      break;
    case PixelFormat::H264:
      throw std::logic_error("H264 must be decoded by the persistent stream decoder");
    case PixelFormat::Unknown:
      throw std::runtime_error("camera returned an unknown pixel format");
  }
  if (bgr.cols != static_cast<int>(frame.width()) || bgr.rows != static_cast<int>(frame.height())) {
    throw std::runtime_error("decoded image dimensions do not match the negotiated camera dimensions");
  }
  return bgr;
}

std::string ffmpegError(const int code)
{
  char buffer[AV_ERROR_MAX_STRING_SIZE] = {};
  av_strerror(code, buffer, sizeof(buffer));
  return std::string(buffer);
}

struct DecodedFrame {
  cv::Mat bgr;
  xgc2::camera::Timestamp timestamp;
  std::uint64_t sequence{0};
};

class H264StreamDecoder {
public:
  H264StreamDecoder()
  {
    const AVCodec* codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!codec) {
      throw std::runtime_error("FFmpeg H264 decoder is unavailable");
    }
    parser_ = av_parser_init(codec->id);
    context_ = avcodec_alloc_context3(codec);
    frame_ = av_frame_alloc();
    packet_ = av_packet_alloc();
    if (!parser_ || !context_ || !frame_ || !packet_) {
      throw std::runtime_error("could not allocate FFmpeg H264 decoder state");
    }
    context_->thread_count = 0;
    context_->thread_type = FF_THREAD_FRAME | FF_THREAD_SLICE;
    const int result = avcodec_open2(context_, codec, nullptr);
    if (result < 0) {
      throw std::runtime_error("could not open FFmpeg H264 decoder: " + ffmpegError(result));
    }
  }

  ~H264StreamDecoder()
  {
    sws_freeContext(scaler_);
    av_packet_free(&packet_);
    av_frame_free(&frame_);
    avcodec_free_context(&context_);
    if (parser_) {
      av_parser_close(parser_);
    }
  }

  H264StreamDecoder(const H264StreamDecoder&) = delete;
  H264StreamDecoder& operator=(const H264StreamDecoder&) = delete;

  bool decode(const Frame& input_frame, DecodedFrame& output)
  {
    SourceMetadata current{input_frame.timestamp(), input_frame.sequence()};
    metadata_[static_cast<std::int64_t>(current.sequence)] = current;
    while (metadata_.size() > 512U) {
      metadata_.erase(metadata_.begin());
    }

    const xgc_camera_driver::detail::PaddedH264Input padded_input(
        input_frame.data(), input_frame.size());
    const std::uint8_t* input = padded_input.data();
    int remaining = padded_input.payloadSize();
    while (remaining > 0) {
      std::uint8_t* packet_data = nullptr;
      int packet_size = 0;
      const int consumed = av_parser_parse2(
          parser_, context_, &packet_data, &packet_size, input, remaining,
          static_cast<std::int64_t>(current.sequence), static_cast<std::int64_t>(current.sequence), 0);
      if (consumed < 0) {
        throw std::runtime_error("FFmpeg could not parse H264 stream: " + ffmpegError(consumed));
      }
      input += consumed;
      remaining -= consumed;
      if (packet_size > 0) {
        sendPacket(packet_data, packet_size, current);
      }
      if (consumed == 0 && packet_size == 0) {
        throw std::runtime_error("FFmpeg H264 parser made no progress");
      }
    }

    if (decoded_.empty()) {
      return false;
    }
    output = std::move(decoded_.front());
    decoded_.pop_front();
    return true;
  }

private:
  struct SourceMetadata {
    xgc2::camera::Timestamp timestamp;
    std::uint64_t sequence;
  };

  void sendPacket(std::uint8_t* data, const int size, const SourceMetadata& current)
  {
    av_packet_unref(packet_);
    packet_->data = data;
    packet_->size = size;
    packet_->pts = parser_->pts == AV_NOPTS_VALUE ? static_cast<std::int64_t>(current.sequence) : parser_->pts;
    packet_->dts = parser_->dts == AV_NOPTS_VALUE ? packet_->pts : parser_->dts;
    int result = avcodec_send_packet(context_, packet_);
    if (result == AVERROR(EAGAIN)) {
      drain(current);
      result = avcodec_send_packet(context_, packet_);
    }
    if (result < 0) {
      throw std::runtime_error("FFmpeg rejected H264 packet: " + ffmpegError(result));
    }
    drain(current);
  }

  void drain(const SourceMetadata& fallback)
  {
    for (;;) {
      const int result = avcodec_receive_frame(context_, frame_);
      if (result == AVERROR(EAGAIN) || result == AVERROR_EOF) {
        return;
      }
      if (result < 0) {
        throw std::runtime_error("FFmpeg failed to decode H264 frame: " + ffmpegError(result));
      }
      if (frame_->width <= 0 || frame_->height <= 0) {
        av_frame_unref(frame_);
        throw std::runtime_error("FFmpeg decoded an H264 frame with invalid dimensions");
      }

      scaler_ = sws_getCachedContext(
          scaler_, frame_->width, frame_->height, static_cast<AVPixelFormat>(frame_->format),
          frame_->width, frame_->height, AV_PIX_FMT_BGR24, SWS_BILINEAR, nullptr, nullptr, nullptr);
      if (!scaler_) {
        av_frame_unref(frame_);
        throw std::runtime_error("FFmpeg could not create H264 color conversion context");
      }
      DecodedFrame decoded;
      decoded.bgr.create(frame_->height, frame_->width, CV_8UC3);
      std::uint8_t* output_data[] = {decoded.bgr.data, nullptr, nullptr, nullptr};
      int output_stride[] = {static_cast<int>(decoded.bgr.step), 0, 0, 0};
      sws_scale(scaler_, frame_->data, frame_->linesize, 0, frame_->height, output_data, output_stride);

      const std::int64_t key = frame_->best_effort_timestamp == AV_NOPTS_VALUE ?
          (frame_->pts == AV_NOPTS_VALUE ? static_cast<std::int64_t>(fallback.sequence) : frame_->pts) :
          frame_->best_effort_timestamp;
      auto found = metadata_.find(key);
      const SourceMetadata& metadata = found == metadata_.end() ? fallback : found->second;
      decoded.timestamp = metadata.timestamp;
      decoded.sequence = metadata.sequence;
      if (found != metadata_.end()) {
        metadata_.erase(found);
      }
      decoded_.push_back(std::move(decoded));
      av_frame_unref(frame_);
    }
  }

  AVCodecParserContext* parser_{nullptr};
  AVCodecContext* context_{nullptr};
  AVFrame* frame_{nullptr};
  AVPacket* packet_{nullptr};
  SwsContext* scaler_{nullptr};
  std::map<std::int64_t, SourceMetadata> metadata_;
  std::deque<DecodedFrame> decoded_;
};

class ImageDecoder {
public:
  bool decode(const Frame& frame, DecodedFrame& decoded)
  {
    if (frame.pixel_format() == PixelFormat::H264) {
      if (!h264_) {
        h264_.reset(new H264StreamDecoder());
      }
      return h264_->decode(frame, decoded);
    }
    decoded.bgr = decodeToBgr(frame);
    decoded.timestamp = frame.timestamp();
    decoded.sequence = frame.sequence();
    return true;
  }

private:
  std::unique_ptr<H264StreamDecoder> h264_;
};

cv::Mat convertOutput(const cv::Mat& bgr, const std::string& encoding)
{
  if (encoding == sensor_msgs::image_encodings::BGR8) {
    return bgr;
  }
  cv::Mat output;
  if (encoding == sensor_msgs::image_encodings::RGB8) {
    cv::cvtColor(bgr, output, cv::COLOR_BGR2RGB);
  } else if (encoding == sensor_msgs::image_encodings::MONO8) {
    cv::cvtColor(bgr, output, cv::COLOR_BGR2GRAY);
  } else {
    throw std::invalid_argument("output_encoding must be bgr8, rgb8, or mono8");
  }
  return output;
}

class RosTimestampMapper {
public:
  ros::Time map(const xgc2::camera::Timestamp& timestamp)
  {
    if (timestamp.clock == xgc2::camera::TimestampClock::Realtime && timestamp.seconds >= 0) {
      ros::Time result;
      result.fromNSec(static_cast<std::uint64_t>(timestamp.to_nanoseconds()));
      return result;
    }

    const std::int64_t source_ns = timestamp.to_nanoseconds();
    if (!anchored_) {
      anchored_ = true;
      source_anchor_ns_ = source_ns;
      ros_anchor_ = ros::Time::now();
    }
    const double delta_seconds = static_cast<double>(source_ns - source_anchor_ns_) * 1e-9;
    return ros_anchor_ + ros::Duration(delta_seconds);
  }

private:
  bool anchored_{false};
  std::int64_t source_anchor_ns_{0};
  ros::Time ros_anchor_;
};

class CameraDriverNode {
public:
  CameraDriverNode()
    : nh_(), private_nh_("~"), image_transport_(nh_), start_wall_time_(ros::WallTime::now())
  {
    loadParameters();
    camera_info_manager_.reset(
        new camera_info_manager::CameraInfoManager(nh_, camera_name_, camera_info_url_));
    camera_publisher_ = image_transport_.advertiseCamera("image_raw", 1);

    updater_.setHardwareID(backend_ == "v4l2" ? video_device_ : "synthetic");
    updater_.add("camera stream", this, &CameraDriverNode::diagnose);

    xgc2::camera::CaptureConfig config;
    config.backend = xgc2::camera::backend_kind_from_string(backend_);
    config.device = video_device_;
    config.width = width_;
    config.height = height_;
    config.frame_rate = framerate_;
    config.pixel_format = xgc2::camera::pixel_format_from_string(pixel_format_);
    config.capture_mode = xgc2::camera::capture_mode_from_string(capture_mode_);
    config.buffer_count = buffer_count_;
    config.synthetic_seed = synthetic_seed_;
    camera_ = xgc2::camera::make_camera(config);
  }

  ~CameraDriverNode()
  {
    if (camera_) {
      camera_->stop();
    }
  }

  int run()
  {
    camera_->start();
    running_.store(true);
    ROS_INFO_STREAM("XGC2 camera started: backend=" << backend_ << " device=" << video_device_
                                                     << " " << width_ << "x" << height_ << "@"
                                                     << framerate_ << " format=" << pixel_format_
                                                     << " output=" << output_encoding_);

    while (ros::ok()) {
      try {
        const Frame frame = camera_->read(capture_timeout_ms_);
        capture_packet_count_.fetch_add(1U);
        publish(frame);
      } catch (const xgc2::camera::CameraError& error) {
        if (error.code() == xgc2::camera::ErrorCode::Timeout) {
          setError(error.what());
          updater_.force_update();
          continue;
        }
        throw;
      }
      updater_.update();
    }
    running_.store(false);
    camera_->stop();
    return 0;
  }

private:
  void loadParameters()
  {
    int width = 640;
    int height = 480;
    int buffer_count = 4;
    int synthetic_seed = 1;
    private_nh_.param<std::string>("backend", backend_, "v4l2");
    private_nh_.param<std::string>("video_device", video_device_, "/dev/video0");
    private_nh_.param("width", width, width);
    private_nh_.param("height", height, height);
    private_nh_.param("framerate", framerate_, 30.0);
    private_nh_.param<std::string>("pixel_format", pixel_format_, "mjpeg");
    private_nh_.param<std::string>("capture_mode", capture_mode_, "auto");
    private_nh_.param<std::string>("output_encoding", output_encoding_, sensor_msgs::image_encodings::BGR8);
    private_nh_.param<std::string>("camera_name", camera_name_, "usb_cam");
    private_nh_.param<std::string>("frame_id", frame_id_, "usb_cam_optical_frame");
    private_nh_.param<std::string>("camera_info_url", camera_info_url_, "");
    private_nh_.param("buffer_count", buffer_count, buffer_count);
    private_nh_.param("synthetic_seed", synthetic_seed, synthetic_seed);
    private_nh_.param("capture_timeout_ms", capture_timeout_ms_, 2000);

    width_ = positiveDimension(width, "width");
    height_ = positiveDimension(height, "height");
    buffer_count_ = positiveDimension(buffer_count, "buffer_count");
    if (buffer_count_ < 2U) {
      throw std::invalid_argument("buffer_count must be at least 2");
    }
    synthetic_seed_ = static_cast<std::uint32_t>(synthetic_seed < 0 ? 0 : synthetic_seed);
    if (framerate_ <= 0.0 || capture_timeout_ms_ <= 0 || camera_name_.empty() || frame_id_.empty()) {
      throw std::invalid_argument("framerate, capture_timeout_ms, camera_name, and frame_id must be valid");
    }
    // Validate strings before opening a device.
    (void)xgc2::camera::backend_kind_from_string(backend_);
    (void)xgc2::camera::pixel_format_from_string(pixel_format_);
    (void)xgc2::camera::capture_mode_from_string(capture_mode_);
    (void)convertOutput(cv::Mat(1, 1, CV_8UC3), output_encoding_);
  }

  void publish(const Frame& frame)
  {
    DecodedFrame decoded;
    if (!decoder_.decode(frame, decoded)) {
      return;
    }
    if (decoded.bgr.cols != static_cast<int>(frame.width()) ||
        decoded.bgr.rows != static_cast<int>(frame.height())) {
      throw std::runtime_error("decoded image dimensions do not match the negotiated camera dimensions");
    }
    const cv::Mat image = convertOutput(decoded.bgr, output_encoding_);
    std_msgs::Header header;
    header.seq = static_cast<std::uint32_t>(decoded.sequence);
    header.stamp = timestamp_mapper_.map(decoded.timestamp);
    header.frame_id = frame_id_;

    sensor_msgs::ImagePtr image_message = cv_bridge::CvImage(header, output_encoding_, image).toImageMsg();
    sensor_msgs::CameraInfo camera_info = camera_info_manager_->getCameraInfo();
    camera_info.header = header;
    camera_info.width = image_message->width;
    camera_info.height = image_message->height;
    camera_publisher_.publish(*image_message, camera_info);

    frame_count_.fetch_add(1U);
    last_sequence_.store(decoded.sequence);
    last_frame_wall_ns_.store(ros::WallTime::now().toNSec());
    clearError();
  }

  void diagnose(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    const std::uint64_t count = frame_count_.load();
    const double elapsed = (ros::WallTime::now() - start_wall_time_).toSec();
    const std::uint64_t last_ns = last_frame_wall_ns_.load();
    const double age = last_ns == 0U ? elapsed :
        (static_cast<double>(ros::WallTime::now().toNSec() - last_ns) * 1e-9);
    std::string error;
    {
      std::lock_guard<std::mutex> lock(error_mutex_);
      error = last_error_;
    }

    const bool calibrated = camera_info_manager_ && camera_info_manager_->isCalibrated();
    if (!running_.load()) {
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "camera is not running");
    } else if (!error.empty() || age > static_cast<double>(capture_timeout_ms_) * 2e-3) {
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                     error.empty() ? "camera frames are stale" : error);
    } else if (count == 0U) {
      status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "waiting for first camera frame");
    } else if (!calibrated) {
      status.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                     "camera is publishing but CameraInfo is not calibrated");
    } else {
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "camera is publishing");
    }
    status.add("backend", backend_);
    status.add("device", video_device_);
    status.add("pixel_format", pixel_format_);
    status.add("output_encoding", output_encoding_);
    status.add("frame_id", frame_id_);
    status.add("captured_packets", capture_packet_count_.load());
    status.add("published_frames", count);
    status.add("camera_info_calibrated", calibrated);
    status.add("camera_info_url", camera_info_url_);
    status.add("last_sequence", last_sequence_.load());
    status.add("measured_fps", elapsed > 0.0 ? static_cast<double>(count) / elapsed : 0.0);
    status.add("last_frame_age_seconds", age);
  }

  void setError(const std::string& message)
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = message;
  }

  void clearError()
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_.clear();
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  image_transport::ImageTransport image_transport_;
  image_transport::CameraPublisher camera_publisher_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  diagnostic_updater::Updater updater_;
  std::unique_ptr<xgc2::camera::Camera> camera_;
  RosTimestampMapper timestamp_mapper_;
  ImageDecoder decoder_;

  std::string backend_;
  std::string video_device_;
  std::string pixel_format_;
  std::string capture_mode_;
  std::string output_encoding_;
  std::string camera_name_;
  std::string frame_id_;
  std::string camera_info_url_;
  std::uint32_t width_{640};
  std::uint32_t height_{480};
  std::uint32_t buffer_count_{4};
  std::uint32_t synthetic_seed_{1};
  double framerate_{30.0};
  int capture_timeout_ms_{2000};

  std::atomic<bool> running_{false};
  std::atomic<std::uint64_t> frame_count_{0};
  std::atomic<std::uint64_t> capture_packet_count_{0};
  std::atomic<std::uint64_t> last_sequence_{0};
  std::atomic<std::uint64_t> last_frame_wall_ns_{0};
  ros::WallTime start_wall_time_;
  std::mutex error_mutex_;
  std::string last_error_;
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xgc_camera_driver");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  try {
    CameraDriverNode node;
    return node.run();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("XGC2 camera driver failed: " << error.what());
    return 1;
  }
}
