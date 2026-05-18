#ifndef GEOMETRY_IO_HPP
#define GEOMETRY_IO_HPP

#include <io/document.hpp>
#include <string>

Document loadYaml(const std::string& path);
void saveYaml(const Document& document, const std::string& path);

#endif
