#ifndef ARG_PARSER_HPP
#define ARG_PARSER_HPP

#include <filesystem>
#include <functional>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

/// \brief Exception thrown when CLI argument parsing fails.
class ParseError : public std::runtime_error {
  public:
    /// \brief Construct a parse error without extra output context.
    /// \param message The error message.
    explicit ParseError(std::string message) : std::runtime_error(std::move(message)) {}

    /// \brief Construct a parse error with usage and optional script context.
    /// \param message The error message.
    /// \param usage The usage text for the failing operation.
    /// \param script_path The instruction script path, if the error came from a script.
    /// \param line_number The 1-based instruction script line number, if available.
    ParseError(std::string message, std::string usage, std::string script_path = "",
               size_t line_number = 0)
        : std::runtime_error(std::move(message)), usage_text(std::move(usage)),
          script_path(std::move(script_path)), line_number(line_number) {}

    std::string usage_text; ///< Usage text for the failing command, if available.
    std::string script_path; ///< Instruction script path, if available.
    size_t line_number = 0; ///< 1-based instruction script line number, or zero if unavailable.
};

/// \brief Whether a declared CLI argument is required.
enum class ArgRequirement { REQUIRED, OPTIONAL };

inline constexpr ArgRequirement REQUIRED = ArgRequirement::REQUIRED;
inline constexpr ArgRequirement OPTIONAL = ArgRequirement::OPTIONAL;

/// \brief A declared positional argument or option.
struct Arg {
    std::function<void(std::string_view)> set_value; ///< Assign parsed text to the bound value.
    ArgRequirement requirement; ///< Whether this argument must be present.
    bool parsed = false;        ///< Whether this argument was present.
};

/// \brief A declared variadic positional argument group.
struct RemainingArgs {
    std::function<void(std::string_view)> append_value; ///< Append parsed text to the bound vector.
    ArgRequirement requirement; ///< Whether at least one remaining argument is required.
    size_t parsed_count = 0;    ///< Number of remaining arguments parsed.
};

/// \brief Small parser for positional arguments and value-taking options.
class ArgParser {
  public:
    ArgParser() = default;

    /// \brief Declare a positional argument bound to a typed variable.
    /// \param variable The destination variable.
    /// \param requirement Whether the positional argument is required.
    template <typename T>
    void declare_positional_arg(T& variable, ArgRequirement requirement = REQUIRED) {
        if (remaining_args) {
            throw std::logic_error("Positional arguments cannot be declared after remaining args");
        }
        if (requirement == REQUIRED && !positional_args.empty() &&
            positional_args.back().requirement == OPTIONAL) {
            throw std::logic_error("Required positional arguments cannot follow optional ones");
        }
        positional_args.push_back({[&variable](std::string_view text) { set_value(variable, text); },
                                   requirement, false});
    }

    /// \brief Declare remaining positional arguments bound to a typed vector.
    /// \param variable The destination vector.
    /// \param requirement Whether at least one remaining argument is required.
    template <typename T>
    void declare_remaining_positional_args(std::vector<T>& variable,
                                           ArgRequirement requirement = OPTIONAL) {
        if (remaining_args) {
            throw std::logic_error("Remaining positional arguments already declared");
        }
        remaining_args = RemainingArgs{
            [&variable](std::string_view text) {
                T value{};
                set_value(value, text);
                variable.push_back(std::move(value));
            },
            requirement, 0};
    }

    /// \brief Declare a named option bound to a typed variable.
    /// \param name The option name, such as "-count".
    /// \param variable The destination variable.
    /// \param requirement Whether the option is required.
    template <typename T>
    void declare_option(std::string name, T& variable, ArgRequirement requirement = REQUIRED) {
        auto [it, inserted] = options.emplace(
            std::move(name),
            Arg{[&variable](std::string_view text) { set_value(variable, text); }, requirement,
                false});
        if (!inserted) {
            throw std::logic_error("Option already declared: " + it->first);
        }
    }

    /// \brief Parse command arguments into the declared variables.
    /// \param args The arguments to parse, including the operation name at index zero.
    void parse(const std::vector<std::string>& args);

  private:
    std::vector<Arg> positional_args; ///< Declared positional arguments in order.
    std::optional<RemainingArgs> remaining_args; ///< Optional variadic positional group.
    std::unordered_map<std::string, Arg> options; ///< Declared options by option name.

    /// \brief Parse an integer value.
    /// \param value The destination value.
    /// \param arg The text to parse.
    static void set_value(int& value, std::string_view arg);

    /// \brief Parse a floating-point value.
    /// \param value The destination value.
    /// \param arg The text to parse.
    static void set_value(double& value, std::string_view arg);

    /// \brief Parse a string value.
    /// \param value The destination value.
    /// \param arg The text to parse.
    static void set_value(std::string& value, std::string_view arg);

    /// \brief Parse a filesystem path value.
    /// \param value The destination value.
    /// \param arg The text to parse.
    static void set_value(std::filesystem::path& value, std::string_view arg);
};

#endif // ARG_PARSER_HPP
