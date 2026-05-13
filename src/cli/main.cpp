#include "cli/operations.hpp"
#include <string>
#include <vector>

int main(int argc, char* argv[]) {
    return dispatchOperation(std::vector<std::string>(argv + 1, argv + argc));
}
