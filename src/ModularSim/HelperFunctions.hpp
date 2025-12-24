#pragma once

#include <string>

#include <toml++/toml.h>
// import tomlplusplus; // soon I will get this to work...

extern double sim_length;    // X size
extern double sim_width;     // Y size

void lower(std::string& s);

void trim_chars(std::string& s, std::string_view chars);

toml::table parse_toml_file(const std::string& filepath);