#include <algorithm>
#include <cctype>
#include <string>

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
