#ifndef __CPP_LOGGER_H
#define __CPP_LOGGER_H

#include <memory>
#include <string>

// 配置常量
constexpr const char* CONFIG_FILE_PATH = "/home/forlinx/.logconfig/agv_control_config.yaml";
constexpr const char* DEFAULT_LOGGER_NAME = "agv_control_logger";  // 统一日志名称

// SPDLOG配置
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"

std::shared_ptr<spdlog::logger> get_logger();
void set_log_level(const std::string& level);

// 安全日志宏
#define LOG_DEBUG(fmt, ...)do {                                            \
SPDLOG_LOGGER_DEBUG (::spdlog::get(DEFAULT_LOGGER_NAME), fmt, ##__VA_ARGS__);       \
} while (0)

#define LOG_INFO(fmt, ...) do {                                            \
SPDLOG_LOGGER_INFO (::spdlog::get(DEFAULT_LOGGER_NAME), fmt, ##__VA_ARGS__);       \
} while (0)

#define LOG_WARN(fmt, ...)  do {                                            \
SPDLOG_LOGGER_WARN (::spdlog::get(DEFAULT_LOGGER_NAME), fmt, ##__VA_ARGS__);       \
} while (0)

#define LOG_ERROR(fmt, ...) do {                                            \
SPDLOG_LOGGER_ERROR(::spdlog::get(DEFAULT_LOGGER_NAME), fmt, ##__VA_ARGS__);       \
} while (0)

#endif // __CPP_LOGGER_H
