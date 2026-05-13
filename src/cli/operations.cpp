#include "cli/operations.hpp"
#include "algorithms/convex_hull.hpp"
#include "algorithms/line_segment_intersection.hpp"
#include "cli/arg_parser.hpp"
#include "geometry/random.hpp"
#include "io/geometry_io.hpp"
#include <algorithm>
#include <cctype>
#include <cstddef>
#include <exception>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

class Operation {
  public:
    virtual ~Operation() = default;

    /// \brief Parse operation-specific command-line arguments into this operation.
    /// \param args The command-line arguments including the operation name at index zero.
    virtual void parse(const std::vector<std::string>& args) = 0;

    /// \brief Validate parsed operation arguments.
    virtual void validate() const {}

    /// \brief Execute this parsed operation.
    /// \returns The process exit code for the operation.
    virtual int run() const = 0;

    /// \brief Return operation-specific usage text.
    /// \returns The usage string for this operation.
    virtual std::string usage() const = 0;

    /// \brief Check whether this operation may be called from an instruction script.
    /// \returns True if the operation is allowed in instruction scripts.
    virtual bool allowedInScript() const {
        return true;
    }
};

using OperationFactory = std::function<std::unique_ptr<Operation>()>;

struct ParseContext {
    std::string script_path; ///< Instruction script path, if parsing a script line.
    size_t line_number = 0;  ///< 1-based script line number, or zero if unavailable.
};

struct RandomOptions {
    int seed = 42;        ///< Seed used for deterministic random generation.
    int count = 100;      ///< Number of random primitives to generate.
    int min_coord = 1;    ///< Minimum generated coordinate.
    int max_coord = 1000; ///< Maximum generated coordinate.
    int min_length = 1;   ///< Minimum generated segment length.
    int max_length = 40;  ///< Maximum generated segment length.
};

std::string trimLeadingWhitespace(const std::string& line) {
    size_t first_non_space = 0;
    while (first_non_space < line.size() &&
           std::isspace(static_cast<unsigned char>(line[first_non_space])) != 0) {
        ++first_non_space;
    }
    return line.substr(first_non_space);
}

std::vector<std::string> tokenize(const std::string& line) {
    std::istringstream stream(line);
    std::vector<std::string> tokens;
    std::string token;
    while (stream >> token) {
        tokens.push_back(token);
    }
    return tokens;
}

std::unique_ptr<Operation> parseOperation(const std::vector<std::string>& args,
                                          const ParseContext& context = {});

class ConvexHullOperation : public Operation {
  public:
    void parse(const std::vector<std::string>& args) override {
        ArgParser parser;
        parser.declare_positional_arg(input_path, REQUIRED);
        parser.declare_positional_arg(output_path, REQUIRED);
        parser.parse(args);
    }

    int run() const override {
        Document document = loadYaml(input_path);
        std::vector<Point> points = document.collectPoints();
        if (points.empty()) {
            throw std::runtime_error("convex_hull requires at least one input point");
        }

        Layer output_layer;
        output_layer.name = "convex_hull";
        output_layer.polygons.push_back(Polygon({convexHull(points)}));
        document.layers.push_back(std::move(output_layer));

        saveYaml(document, output_path);
        std::cout << "Wrote " << output_path << std::endl;
        return 0;
    }

    std::string usage() const override {
        return "input.yaml output.yaml\n";
    }

  private:
    std::string input_path;
    std::string output_path;
};

class IntersectionOperation : public Operation {
  public:
    void parse(const std::vector<std::string>& args) override {
        ArgParser parser;
        parser.declare_positional_arg(input_path, REQUIRED);
        parser.declare_positional_arg(output_path, REQUIRED);
        parser.declare_option("-algorithm", algorithm, OPTIONAL);
        parser.parse(args);
    }

    void validate() const override {
        if (algorithm != "line-sweep" && algorithm != "brute-force") {
            throw ParseError("Algorithm must be 'line-sweep' or 'brute-force'");
        }
    }

    int run() const override {
        Document document = loadYaml(input_path);
        std::vector<Segment> segments = document.collectSegments();
        std::unordered_set<Point> intersections = algorithm == "brute-force"
                                                      ? bruteForceLineSegmentIntersection(segments)
                                                      : lineSegmentIntersection(segments);

        Layer output_layer;
        output_layer.name = "intersections";
        output_layer.points.insert(output_layer.points.end(), intersections.begin(),
                                   intersections.end());
        document.layers.push_back(std::move(output_layer));

        saveYaml(document, output_path);
        std::cout << "Found " << intersections.size() << " intersections" << std::endl;
        std::cout << "Algorithm: " << algorithm << std::endl;
        std::cout << "Wrote " << output_path << std::endl;
        return 0;
    }

    std::string usage() const override {
        return "input.yaml output.yaml "
               "[-algorithm {line-sweep|brute-force}]\n";
    }

  private:
    std::string input_path;
    std::string output_path;
    std::string algorithm = "line-sweep";
};

class CompareOperation : public Operation {
  public:
    void parse(const std::vector<std::string>& args) override {
        ArgParser parser;
        parser.declare_positional_arg(first_input_path, REQUIRED);
        parser.declare_remaining_positional_args(other_input_paths, REQUIRED);
        parser.parse(args);
    }

    int run() const override {
        Document expected = loadYaml(first_input_path);
        bool all_equal = true;
        for (const std::string& input_path : other_input_paths) {
            Document actual = loadYaml(input_path);
            if (!(expected == actual)) {
                std::cerr << "Mismatch: " << first_input_path << " != " << input_path << std::endl;
                all_equal = false;
            }
        }

        if (!all_equal) {
            return 1;
        }

        std::cout << "All documents match" << std::endl;
        return 0;
    }

    std::string usage() const override {
        return "input.yaml input.yaml [input.yaml ...]\n";
    }

  private:
    std::string first_input_path;
    std::vector<std::string> other_input_paths;
};

class FilterOperation : public Operation {
  public:
    void parse(const std::vector<std::string>& args) override {
        ArgParser parser;
        parser.declare_positional_arg(input_path, REQUIRED);
        parser.declare_positional_arg(output_path, REQUIRED);
        parser.declare_option("-min-x", min_x, REQUIRED);
        parser.declare_option("-min-y", min_y, REQUIRED);
        parser.declare_option("-max-x", max_x, REQUIRED);
        parser.declare_option("-max-y", max_y, REQUIRED);
        parser.declare_option("-layer", layer_name, OPTIONAL);
        parser.parse(args);
    }

    void validate() const override {
        if (min_x > max_x || min_y > max_y) {
            throw ParseError("Minimum bounds must be less than or equal to maximum bounds");
        }
    }

    int run() const override {
        Document document = loadYaml(input_path);
        Rectangle rectangle(Point(min_x, min_y), Point(max_x, max_y));

        bool filtered_layer = false;
        for (Layer& layer : document.layers) {
            if (layer_name.empty() || layer.name == layer_name) {
                layer.filter(rectangle);
                filtered_layer = true;
            }
        }

        if (!layer_name.empty() && !filtered_layer) {
            throw std::runtime_error("Layer not found: " + layer_name);
        }

        saveYaml(document, output_path);
        std::cout << "Wrote " << output_path << std::endl;
        return 0;
    }

    std::string usage() const override {
        return "input.yaml output.yaml -min-x X -min-y Y -max-x X "
               "-max-y Y [-layer name]\n";
    }

  private:
    std::string input_path;
    std::string output_path;
    int min_x = 0;
    int min_y = 0;
    int max_x = 0;
    int max_y = 0;
    std::string layer_name;
};

class RandomOperation : public Operation {
  public:
    void parse(const std::vector<std::string>& args) override {
        ArgParser parser;
        parser.declare_positional_arg(kind, REQUIRED);
        parser.declare_positional_arg(output_path, REQUIRED);
        parser.declare_option("-seed", options.seed, OPTIONAL);
        parser.declare_option("-count", options.count, OPTIONAL);
        parser.declare_option("-min-coord", options.min_coord, OPTIONAL);
        parser.declare_option("-max-coord", options.max_coord, OPTIONAL);
        parser.declare_option("-min-length", options.min_length, OPTIONAL);
        parser.declare_option("-max-length", options.max_length, OPTIONAL);
        parser.parse(args);
    }

    void validate() const override {
        if (kind != "points" && kind != "segments") {
            throw ParseError("Random kind must be 'points' or 'segments'");
        }
        if (options.count < 0) {
            throw ParseError("count must be non-negative");
        }
        if (options.min_coord > options.max_coord) {
            throw ParseError("min-coord must be less than or equal to max-coord");
        }
        if (options.min_length < 0 || options.max_length < 0) {
            throw ParseError("segment lengths must be non-negative");
        }
        if (options.min_length > options.max_length) {
            throw ParseError("min-length must be less than or equal to max-length");
        }
    }

    int run() const override {
        std::mt19937 rng(options.seed);

        Document document;
        Layer layer;

        if (kind == "points") {
            layer.name = "random_points";
            layer.points = randomPoints(options.count, options.min_coord, options.max_coord, &rng);
        } else {
            layer.name = "random_segments";
            layer.segments = randomSegments(options.count, options.min_length, options.max_length,
                                            options.min_coord, options.max_coord, &rng);
        }

        document.layers.push_back(std::move(layer));
        saveYaml(document, output_path);
        std::cout << "Wrote " << output_path << std::endl;
        return 0;
    }

    std::string usage() const override {
        return "points output.yaml [-seed N] [-count N] [-min-coord N] "
               "[-max-coord N]\n"
               "segments output.yaml [-seed N] [-count N] [-min-coord N] "
               "[-max-coord N] [-min-length N] [-max-length N]\n";
    }

  private:
    std::string kind;
    std::string output_path;
    RandomOptions options;
};

class ScriptOperation : public Operation {
  public:
    bool allowedInScript() const override {
        return false;
    }

    void parse(const std::vector<std::string>& args) override {
        ArgParser parser;
        parser.declare_positional_arg(script_path, REQUIRED);
        parser.parse(args);

        std::ifstream script(script_path);
        if (!script) {
            throw std::runtime_error("Could not open instruction script: " + script_path);
        }

        std::string line;
        size_t line_number = 0;
        while (std::getline(script, line)) {
            ++line_number;
            const std::string trimmed = trimLeadingWhitespace(line);
            if (trimmed.empty() || trimmed[0] == '#') {
                continue;
            }

            const std::vector<std::string> sub_args = tokenize(trimmed);
            ParseContext parse_context{script_path, line_number};
            std::unique_ptr<Operation> sub_operation = parseOperation(sub_args, parse_context);

            operations.push_back(std::move(sub_operation));
        }
    }

    int run() const override {
        for (const auto& operation : operations) {
            int result = operation->run();
            if (result != 0) {
                return result;
            }
        }
        return 0;
    }

    std::string usage() const override {
        return "instructions.cgeom\n";
    }

  private:
    std::string script_path;
    std::vector<std::unique_ptr<Operation>> operations;
};

const std::unordered_map<std::string, OperationFactory>& operationFactories() {
    static const std::unordered_map<std::string, OperationFactory> factories = {
        {"compare", [] { return std::make_unique<CompareOperation>(); }},
        {"convex_hull", [] { return std::make_unique<ConvexHullOperation>(); }},
        {"filter", [] { return std::make_unique<FilterOperation>(); }},
        {"intersection", [] { return std::make_unique<IntersectionOperation>(); }},
        {"random", [] { return std::make_unique<RandomOperation>(); }},
        {"script", [] { return std::make_unique<ScriptOperation>(); }},
    };
    return factories;
}

void printGeneralUsage() {
    std::vector<std::string> names;
    names.reserve(operationFactories().size());
    for (const auto& [name, factory] : operationFactories()) {
        names.push_back(name);
    }
    std::sort(names.begin(), names.end());

    std::cerr << "Usage:\n  <subcommand> ...\n\nSubcommands:\n";
    for (const std::string& name : names) {
        std::cerr << "  " << name << '\n';
    }
}

std::string usageForCommand(const std::string& command, const std::string& usage) {
    std::istringstream lines(usage);
    std::string formatted = "Usage:\n";
    std::string line;
    while (std::getline(lines, line)) {
        if (line.empty()) {
            continue;
        }
        formatted += "  " + command + ' ' + line + '\n';
    }
    return formatted;
}

void printParseError(const ParseError& error) {
    std::cerr << "Error: ";
    if (!error.script_path.empty() && error.line_number != 0) {
        std::cerr << error.script_path << ':' << error.line_number << ": ";
    }
    std::cerr << error.what() << std::endl;

    if (error.usage_text.empty()) {
        printGeneralUsage();
    } else {
        std::cerr << error.usage_text;
    }
}

std::unique_ptr<Operation> parseOperation(const std::vector<std::string>& args,
                                          const ParseContext& context) {
    if (args.empty()) {
        throw ParseError("Missing subcommand", "", context.script_path, context.line_number);
    }

    const auto factory_it = operationFactories().find(args[0]);
    if (factory_it == operationFactories().end()) {
        throw ParseError("Unknown subcommand: " + args[0], "", context.script_path,
                         context.line_number);
    }

    std::unique_ptr<Operation> operation = factory_it->second();

    if (!context.script_path.empty() && !operation->allowedInScript()) {
        throw ParseError("operation cannot be called from an instruction script",
                         usageForCommand(args[0], operation->usage()), context.script_path,
                         context.line_number);
    }

    try {
        operation->parse(args);
        operation->validate();
    } catch (const ParseError& error) {
        if (!error.usage_text.empty() || !error.script_path.empty() || error.line_number != 0) {
            throw;
        }
        throw ParseError(error.what(), usageForCommand(args[0], operation->usage()),
                         context.script_path, context.line_number);
    }

    return operation;
}

} // namespace

int dispatchOperation(const std::vector<std::string>& args) {
    std::unique_ptr<Operation> operation;
    try {
        operation = parseOperation(args);
    } catch (const ParseError& error) {
        printParseError(error);
        return 1;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << std::endl;
        return 1;
    }

    try {
        return operation->run();
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << std::endl;
        return 1;
    }
}
