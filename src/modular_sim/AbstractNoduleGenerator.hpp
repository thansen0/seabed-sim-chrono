#pragma once

#include <memory>
#include <vector>

#include "DynamicSystemMulticore.hpp"

#include "chrono/physics/ChBodyEasy.h"

struct Nodule {
    double x;   // meters
    double y;   // meters
    double d;   // diameter in meters
    std::shared_ptr<chrono::ChBody> nodule;
};

class AbstractNoduleGenerator {
protected:
    std::string config_path;
    DynamicSystemMulticore *sys;

public:
    AbstractNoduleGenerator(const std::string& config_path, DynamicSystemMulticore *sys) : config_path(config_path), sys(sys) {}

    virtual std::vector<Nodule> generate_nodules() {
        throw std::logic_error("AbstractNoduleGenerator::generate_nodules not implemented");
    }
};