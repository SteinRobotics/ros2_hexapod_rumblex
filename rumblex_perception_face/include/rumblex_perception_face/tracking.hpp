#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <opencv2/core.hpp>
#include <optional>
#include <vector>

namespace rumblex_perception_face {
struct Detection {
    cv::Rect2f box;
    float confidence;
};

inline bool validDetection(const Detection& d) {
    return std::isfinite(d.box.x) && std::isfinite(d.box.y) && std::isfinite(d.box.width) &&
           std::isfinite(d.box.height) && d.box.width > 0 && d.box.height > 0 &&
           std::isfinite(d.confidence) && d.confidence >= 0 && d.confidence <= 1;
}

// Conservative association: ambiguous overlap loses the track instead of claiming identity.
class TargetSelector {
   public:
    explicit TargetSelector(int acquisition_frames = 3) : acquisition_frames_(acquisition_frames) {
    }
    std::optional<Detection> update(const std::vector<Detection>& detections) {
        std::vector<Detection> valid;
        for (const auto& detection : detections)
            if (validDetection(detection)) valid.push_back(detection);
        if (valid.empty()) {
            reset();
            return std::nullopt;
        }
        Detection chosen = *std::max_element(valid.begin(), valid.end(), [](const auto& a, const auto& b) {
            return a.box.area() < b.box.area();
        });
        if (previous_) {
            int matches = 0;
            for (const auto& candidate : valid) {
                const float intersection = (candidate.box & previous_->box).area();
                const float overlap =
                    intersection / (candidate.box.area() + previous_->box.area() - intersection);
                if (overlap >= 0.2F) {
                    chosen = candidate;
                    ++matches;
                }
            }
            if (matches != 1) {
                reset();
                return std::nullopt;
            }
        } else {
            ++track_id_;
        }
        previous_ = chosen;
        ++consecutive_;
        return consecutive_ >= acquisition_frames_ ? previous_ : std::nullopt;
    }
    void reset() {
        previous_.reset();
        consecutive_ = 0;
    }
    uint32_t trackId() const {
        return track_id_;
    }

   private:
    int acquisition_frames_;
    int consecutive_ = 0;
    uint32_t track_id_ = 0;
    std::optional<Detection> previous_;
};
}  // namespace rumblex_perception_face
