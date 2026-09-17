#include "Logger.h"
#include <iostream>

void Logger::info(const std::string& message) {
	std::cout << "[INFO] " << message << std::endl;
}

void Logger::warn(const std::string& message) {
	std::cout << "[WARN] " << message << std::endl;
}

void Logger::error(const std::string& message) {
	std::cout << "[ERROR] " << message << std::endl;
}