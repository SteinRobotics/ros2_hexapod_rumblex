#pragma once
#include "requester/types.hpp"
#include "rumblex_utils/body_model.hpp"
namespace rumblex_movement {
class CKinematics : public rumblex_geometry::CBodyModel {
   public:
    using CBodyModel::CBodyModel;
};
}  // namespace rumblex_movement
