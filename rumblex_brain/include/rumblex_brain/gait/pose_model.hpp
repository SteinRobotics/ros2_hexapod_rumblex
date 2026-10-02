#pragma once
#include "rumblex_utils/body_model.hpp"
namespace brain {
using namespace rumblex_geometry;
class CPoseModel : public rumblex_geometry::CBodyModel {
   public:
    using CBodyModel::CBodyModel;
};
}  // namespace brain
