/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include <algorithm>
#include <map>
#include <ranges>
#include <string>
#include <unordered_map>
//
#include "rclcpp/rclcpp.hpp"
//
#include "requester/interpretation.hpp"
#include "requester/word.hpp"

namespace brain {

class CTextInterpreter {
   public:
    CTextInterpreter(std::shared_ptr<rclcpp::Node> node);
    virtual ~CTextInterpreter() = default;

    std::vector<CWord> parseText(std::string& text);
    std::string searchInterpretation(const std::vector<CWord>& identifiedWords);
    bool lettersIdentified(const std::string& letters, const std::vector<CWord>& words);

   private:
    CWord letters2Word(const std::string& letters);
    bool isWordIn(const CWord& word, const std::vector<CWord>& words);
    void readInterpretation();
    std::shared_ptr<rclcpp::Node> node_;
    std::unordered_map<std::string, CWord> vocabulary_;
    std::unordered_map<std::string, CInterpretation> interpretations_;
};

}  // namespace brain
