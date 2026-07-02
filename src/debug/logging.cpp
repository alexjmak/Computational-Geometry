#include "debug/logging.hpp"
#include <cctype>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>

namespace debug {
namespace {

/// \brief Stream buffer that discards all bytes written to it.
class NullBuffer : public std::streambuf {
  public:
    int overflow(int character) override {
        return character;
    }
};

/// \brief Parsed set of categories enabled by `CGEOM_DEBUG`.
struct EnabledCategories {
#define X(category) bool category = false;
    DEBUG_CATEGORY_LIST(X)
#undef X
};

EnabledCategories parseEnabledCategories();

/// \brief Category state parsed once when the logging module is initialized.
EnabledCategories enabled_categories = parseEnabledCategories();
NullBuffer null_buffer;
std::ostream null_stream(&null_buffer);

/// \brief Return a copy of `value` without leading or trailing ASCII whitespace.
std::string trim(const std::string& value) {
    const std::size_t start = value.find_first_not_of(" \t\n\r");
    if (start == std::string::npos) {
        return "";
    }
    const std::size_t end = value.find_last_not_of(" \t\n\r");
    return value.substr(start, end - start + 1);
}

/// \brief Compare an environment token with a category name ignoring ASCII case.
bool equalsIgnoringCase(const std::string& lhs, const char* rhs) {
    std::size_t index = 0;
    while (index < lhs.size() && rhs[index] != '\0') {
        const auto lhs_char = static_cast<unsigned char>(lhs[index]);
        const auto rhs_char = static_cast<unsigned char>(rhs[index]);
        if (std::tolower(lhs_char) != std::tolower(rhs_char)) {
            return false;
        }
        ++index;
    }
    return index == lhs.size() && rhs[index] == '\0';
}

/// \brief Enable a single category token, ignoring unknown tokens.
void enableCategory(EnabledCategories& categories, const std::string& token) {
    if (equalsIgnoringCase(token, "all")) {
#define X(category) categories.category = true;
        DEBUG_CATEGORY_LIST(X)
#undef X
        return;
    }

#define X(category)                                                                                \
    if (equalsIgnoringCase(token, #category)) {                                                    \
        categories.category = true;                                                                \
        return;                                                                                    \
    }
    DEBUG_CATEGORY_LIST(X)
#undef X
}

/// \brief Parse `CGEOM_DEBUG` into enabled category flags.
///
/// Accepted forms include comma-separated category names such as `dcel,rayQuery` and `all`.
/// Unknown tokens are intentionally ignored so typos do not break normal execution.
EnabledCategories parseEnabledCategories() {
    EnabledCategories categories{};

    const char* debug_env = std::getenv("CGEOM_DEBUG");
    if (debug_env == nullptr || debug_env[0] == '\0') {
        return categories;
    }

    std::stringstream tokens(debug_env);
    std::string token;
    while (std::getline(tokens, token, ',')) {
        enableCategory(categories, trim(token));
    }
    return categories;
}

/// \brief Return the parsed enabled category flags.
const EnabledCategories& enabledCategories() {
    return enabled_categories;
}

} // namespace

#define X(category)                                                                                \
    /** \brief Return whether this category was enabled when `CGEOM_DEBUG` was parsed. */          \
    bool category##EnabledImpl() {                                                                 \
        return enabledCategories().category;                                                       \
    }                                                                                              \
    /** \brief Return `std::cerr` for enabled diagnostics, otherwise a sink stream. */             \
    std::ostream& category() {                                                                     \
        if (category##Enabled()) {                                                                 \
            return std::cerr;                                                                      \
        }                                                                                          \
        return null_stream;                                                                        \
    }
DEBUG_CATEGORY_LIST(X)
#undef X

} // namespace debug
