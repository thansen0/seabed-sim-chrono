#pragma once

#include <vector>
#include <string>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <random>
#include <unordered_map>
#include <utility>
#include <algorithm>

#include "AbstractNoduleGenerator.hpp"
#include "DynamicSystemMulticore.hpp"

class UniformNodules : public AbstractNoduleGenerator {
public:
    UniformNodules(const std::string& config_path, DynamicSystemMulticore *sys);

    std::vector<Nodule> generate_nodules() override;
};