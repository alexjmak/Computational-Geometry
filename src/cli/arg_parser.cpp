#include "cli/arg_parser.hpp"
#include <cerrno>
#include <charconv>
#include <cstdlib>
#include <string>

void ArgParser::parse(const std::vector<std::string>& args) {
    size_t positional_index = 0;
    for (size_t i = 1; i < args.size(); ++i) {
        const std::string& arg = args[i];

        if (!arg.empty() && arg[0] == '-') {
            auto option_it = options.find(arg);
            if (option_it == options.end()) {
                throw ParseError("Unknown option: " + arg);
            }

            if (i + 1 >= args.size()) {
                throw ParseError("Missing value for option: " + arg);
            }

            option_it->second.set_value(args[++i]);
            option_it->second.parsed = true;
            continue;
        }

        if (positional_index < positional_args.size()) {
            positional_args[positional_index].set_value(arg);
            positional_args[positional_index].parsed = true;
            ++positional_index;
            continue;
        }

        if (remaining_args) {
            remaining_args->append_value(arg);
            ++remaining_args->parsed_count;
            continue;
        }

        throw ParseError("Too many arguments");
    }

    for (const auto& [name, option] : options) {
        if (!option.parsed && option.requirement == REQUIRED) {
            throw ParseError("Missing required option: " + name);
        }
    }

    for (size_t i = positional_index; i < positional_args.size(); ++i) {
        if (positional_args[i].requirement == REQUIRED) {
            throw ParseError("Too few arguments");
        }
    }

    if (remaining_args && remaining_args->requirement == REQUIRED &&
        remaining_args->parsed_count == 0) {
        throw ParseError("Too few arguments");
    }
}

void ArgParser::set_value(int& value, std::string_view text) {
    int tmp{};
    auto [ptr, ec] = std::from_chars(text.data(), text.data() + text.size(), tmp);

    if (ec != std::errc{} || ptr != text.data() + text.size()) {
        throw ParseError("Invalid integer");
    }

    value = tmp;
}

void ArgParser::set_value(double& value, std::string_view text) {
    std::string s(text); // strtod needs null-terminated string.

    char* end = nullptr;
    errno = 0;

    double tmp = std::strtod(s.c_str(), &end);

    if (errno == ERANGE || end != s.c_str() + s.size()) {
        throw ParseError("Invalid double");
    }

    value = tmp;
}

void ArgParser::set_value(std::string& value, std::string_view text) {
    value = std::string(text);
}

void ArgParser::set_value(std::filesystem::path& value, std::string_view text) {
    value = std::filesystem::path(text);
}
