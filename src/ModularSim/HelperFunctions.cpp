#include <algorithm>
#include <cctype>
#include <string>
#include <filesystem>
#include <iostream>

#include <toml++/toml.h>

#include "HelperFunctions.hpp"

void lower(std::string& s) {
    std::transform(
        s.begin(), s.end(), s.begin(),
        [](unsigned char c) { return std::tolower(c); }
    );
}

void trim_chars(std::string& s, std::string_view chars) {
    auto start = s.find_first_not_of(chars);
    if (start == std::string::npos)
        return;

    auto end = s.find_last_not_of(chars);
    s = s.substr(start, end - start + 1);
}

toml::table parse_toml_file(const std::string& filepath) {
    if (!std::filesystem::exists(filepath)) {
        // file doesn't exist
        std::cerr << "Config file \"" << filepath << "\" does not exist! Exiting." << std::endl;
        exit(2);
    }

    toml::table config_tbl = toml::parse_file(filepath);

    if (auto v = config_tbl["MASTER_CONFIG"]["sim_length"].value<double>()) {
        sim_length = *v;
    } else if (auto v = config_tbl["sim_length"].value<double>()) {
        sim_length = *v;
    } else {
        std::cerr << "Warning: sim_length not set in config, exiting" << std::endl;
        exit(2);
    }

    if (auto v = config_tbl["MASTER_CONFIG"]["sim_width"].value<double>()) {
        sim_width = *v;
    } else if (auto v = config_tbl["sim_width"].value<double>()) {
        sim_width = *v;
    } else {
        std::cerr << "Warning: sim_width not set in config, exiting" << std::endl;
        exit(2);
    }

    return config_tbl;
}