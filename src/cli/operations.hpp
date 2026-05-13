#ifndef OPERATIONS_HPP
#define OPERATIONS_HPP

#include <string>
#include <vector>

/// \brief Parse and execute a CLI operation.
/// \param args The command-line arguments excluding argv[0].
/// \returns The process exit code for the selected operation.
int dispatchOperation(const std::vector<std::string>& args);

#endif // OPERATIONS_HPP
