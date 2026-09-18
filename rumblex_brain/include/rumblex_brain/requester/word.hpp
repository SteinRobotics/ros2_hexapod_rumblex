#pragma once

#include <cstdint>
#include <string>

namespace brain {

class CWord {
   public:
    CWord() {};
    virtual ~CWord() = default;

    std::string value = "";
    uint32_t index = 0;
    std::string type = "";
};

}  // namespace brain
