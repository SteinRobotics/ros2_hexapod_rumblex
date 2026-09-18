// auto code created by VocabularyCreator.py START

#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace brain {

class CInterpretation {
   public:
    CInterpretation() {};
    virtual ~CInterpretation() = default;

    std::unordered_map<std::string, std::vector<uint32_t>> wordType2WordIndices;
};

}  // namespace brain
