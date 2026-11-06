#include <chrono>
#include <cmath>
#include <cv_bridge/cv_bridge.hpp>
#include <numbers>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/face.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include "rumblex_interfaces/msg/face_target.hpp"
#include "rumblex_perception_face/tracking.hpp"

using rumblex_interfaces::msg::FaceTarget;
using rumblex_perception_face::Detection;

class FaceDetector : public rclcpp::Node {
   public:
    FaceDetector() : Node("node_face_detector") {
        const auto model = declare_parameter<std::string>("model_path", "");
        const double hz = declare_parameter("processing_hz", 12.0);
        width_ = declare_parameter("input_width", 320);
        const double confidence = declare_parameter("confidence_threshold", 0.85);
        minimum_ = declare_parameter("minimum_face_pixels", 24.0);
        max_age_ = declare_parameter("max_image_age_s", 0.3);
        approximate_ = declare_parameter("allow_approximate_intrinsics", false);
        fov_ = declare_parameter("desktop_horizontal_fov_deg", 65.0);
        debug_ = declare_parameter("publish_debug_image", false);
        const int acquisition = declare_parameter("acquisition_frames", 3);
        if (!std::isfinite(hz) || hz < 1 || hz > 60 || width_ < 64 || width_ > 1280 ||
            !std::isfinite(confidence) || confidence <= 0 || confidence > 1 || !std::isfinite(minimum_) ||
            minimum_ <= 0 || !std::isfinite(max_age_) || max_age_ <= 0 || max_age_ > 1 ||
            !std::isfinite(fov_) || fov_ < 10 || fov_ > 150 || acquisition < 1 || acquisition > 30)
            throw std::invalid_argument("Invalid face detector parameters");
        selector_ = rumblex_perception_face::TargetSelector(acquisition);
        cv::setNumThreads(1);
        detector_ = cv::FaceDetectorYN::create(model, "", {width_, width_}, static_cast<float>(confidence));
        target_pub_ = create_publisher<FaceTarget>("perception/face_target", rclcpp::QoS(1));
        status_pub_ = create_publisher<std_msgs::msg::String>("perception/face_status",
                                                              rclcpp::QoS(1).transient_local());
        image_pub_ = create_publisher<sensor_msgs::msg::Image>("perception/face_debug",
                                                               rclcpp::SensorDataQoS().keep_last(1));
        compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
            "perception/face_debug/compressed", rclcpp::SensorDataQoS().keep_last(1));
        if (declare_parameter("compressed_input", false)) {
            compressed_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
                "camera/image_raw/compressed", rclcpp::SensorDataQoS().keep_last(1),
                [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr image) {
                    try {
                        latest_ = cv_bridge::toCvCopy(image, "bgr8")->toImageMsg();
                    } catch (const std::exception& error) {
                        latest_.reset();
                        selector_.reset();
                        status(std::string("inhibited: invalid compressed image: ") + error.what());
                    }
                });
        } else {
            image_sub_ = create_subscription<sensor_msgs::msg::Image>(
                "camera/image_raw", rclcpp::SensorDataQoS().keep_last(1),
                [this](sensor_msgs::msg::Image::ConstSharedPtr image) { latest_ = image; });
        }
        info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera/camera_info", rclcpp::SensorDataQoS().keep_last(1),
            [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) { info_ = info; });
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / hz), [this] { process(); });
        status("waiting for camera");
    }

   private:
    void status(const std::string& text) {
        if (text == status_) return;
        status_ = text;
        std_msgs::msg::String message;
        message.data = text;
        status_pub_->publish(message);
        RCLCPP_INFO(get_logger(), "%s", text.c_str());
    }
    void process() {
        if (!latest_) return;
        const auto image = std::exchange(latest_, nullptr);
        FaceTarget target;
        target.header = image->header;
        const auto stamp = rclcpp::Time(image->header.stamp);
        const double age = (now() - stamp).seconds();
        if (age < -0.05 || age > max_age_ || stamp.nanoseconds() <= last_stamp_ ||
            image->header.frame_id.empty()) {
            selector_.reset();
            if (stamp.nanoseconds() < last_stamp_) last_stamp_ = 0;
            target_pub_->publish(target);
            status("inhibited: stale, out-of-order, or invalid image header");
            return;
        }
        last_stamp_ = stamp.nanoseconds();
        try {
            auto bgr = cv_bridge::toCvCopy(image, "bgr8")->image;
            if (bgr.cols < 1 || bgr.rows < 1) throw std::runtime_error("Empty image");
            if (size_ != bgr.size()) {
                selector_.reset();
                size_ = bgr.size();
            }
            const double scale = static_cast<double>(width_) / std::max(bgr.cols, bgr.rows);
            cv::Mat resized;
            cv::resize(bgr, resized, {}, scale, scale);
            detector_->setInputSize(resized.size());
            cv::Mat faces;
            detector_->detect(resized, faces);
            std::vector<Detection> detections;
            for (int row = 0; row < faces.rows; ++row) {
                Detection d{{faces.at<float>(row, 0), faces.at<float>(row, 1), faces.at<float>(row, 2),
                             faces.at<float>(row, 3)},
                            faces.at<float>(row, 14)};
                // Use actual rounded resize dimensions, not the nominal scale.
                d.box.x *= static_cast<float>(bgr.cols) / resized.cols;
                d.box.width *= static_cast<float>(bgr.cols) / resized.cols;
                d.box.y *= static_cast<float>(bgr.rows) / resized.rows;
                d.box.height *= static_cast<float>(bgr.rows) / resized.rows;
                if (rumblex_perception_face::validDetection(d) && d.box.width >= minimum_ &&
                    d.box.height >= minimum_)
                    detections.push_back(d);
            }
            const auto chosen = selector_.update(detections);
            if (chosen) {
                const cv::Point2f centre(chosen->box.x + chosen->box.width / 2,
                                         chosen->box.y + chosen->box.height / 2);
                const bool calibrated =
                    info_ && info_->width == image->width && info_->height == image->height &&
                    info_->header.frame_id == image->header.frame_id && info_->k[0] > 0 && info_->k[4] > 0 &&
                    std::all_of(info_->k.begin(), info_->k.end(),
                                [](double v) { return std::isfinite(v); }) &&
                    std::all_of(info_->d.begin(), info_->d.end(),
                                [](double v) { return std::isfinite(v); }) &&
                    (info_->distortion_model == "plumb_bob" ||
                     info_->distortion_model == "rational_polynomial") &&
                    (info_->d.size() == 5 || info_->d.size() == 8);
                if (calibrated) {
                    cv::Mat k(3, 3, CV_64F, const_cast<double*>(info_->k.data()));
                    std::vector<cv::Point2f> rays;
                    cv::undistortPoints(std::vector<cv::Point2f>{centre}, rays, k, info_->d);
                    target.x_error = rays[0].x;
                    target.y_error = rays[0].y;
                } else if (approximate_) {
                    const double focal = bgr.cols / (2 * std::tan(fov_ * std::numbers::pi / 360));
                    target.x_error = (centre.x - bgr.cols / 2.0) / focal;
                    target.y_error = (centre.y - bgr.rows / 2.0) / focal;
                }
                target.calibrated = calibrated;
                target.valid = (calibrated || approximate_) && std::isfinite(target.x_error) &&
                               std::isfinite(target.y_error);
                target.track_id = selector_.trackId();
                target.confidence = chosen->confidence;
                status(target.valid
                           ? (calibrated ? "tracking: calibrated" : "tracking: approximate desktop FOV")
                           : "inhibited: camera calibration required");
                if (debug_) cv::rectangle(bgr, chosen->box, {0, 255, 0}, 2);
            } else {
                status("acquiring: no unambiguous persistent face");
            }
            // Inference may itself exceed the freshness budget.
            if ((now() - stamp).seconds() > max_age_) {
                target.valid = false;
                selector_.reset();
                status("inhibited: inference exceeded image age limit");
            }
            target_pub_->publish(target);
            if (debug_) {
                cv::putText(bgr, status_, {12, 24}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 255}, 1);
                const cv_bridge::CvImage annotated(image->header, "bgr8", bgr);
                if (image_pub_->get_subscription_count()) image_pub_->publish(*annotated.toImageMsg());
                if (compressed_pub_->get_subscription_count())
                    compressed_pub_->publish(*annotated.toCompressedImageMsg());
            }
        } catch (const std::exception& error) {
            selector_.reset();
            target.valid = false;
            target_pub_->publish(target);
            status(std::string("inhibited: ") + error.what());
        }
    }
    int width_{};
    double minimum_{}, max_age_{}, fov_{};
    bool approximate_{}, debug_{};
    int64_t last_stamp_ = 0;
    cv::Size size_;
    std::string status_;
    cv::Ptr<cv::FaceDetectorYN> detector_;
    rumblex_perception_face::TargetSelector selector_;
    sensor_msgs::msg::Image::ConstSharedPtr latest_;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
    rclcpp::Publisher<FaceTarget>::SharedPtr target_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<FaceDetector>());
    } catch (const std::exception& error) {
        RCLCPP_ERROR(rclcpp::get_logger("node_face_detector"), "Face detector unavailable: %s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
}
