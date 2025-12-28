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
    DynamicSystemMulticore *sys;

public:
    AbstractNoduleGenerator(const toml::table& config_tbl, DynamicSystemMulticore *sys) : sys(sys) {}

    virtual std::vector<Nodule> generate_nodules() {
        throw std::logic_error("AbstractNoduleGenerator::generate_nodules not implemented");
    }
};