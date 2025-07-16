#include "Logger.h"
#include "spdlog/sinks/daily_file_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;

// 日志级别映射表
static const std::map<std::string, spdlog::level::level_enum> LEVEL_MAP = {
    {"TRACE", spdlog::level::trace}, {"DEBUG", spdlog::level::debug},
    {"INFO", spdlog::level::info}, {"WARN", spdlog::level::warn},
    {"ERROR", spdlog::level::err}, {"CRITICAL", spdlog::level::critical}
};

std::shared_ptr<spdlog::logger> init_logger_from_config(const std::string& config_path)
{
    static std::shared_ptr<spdlog::logger> logger(nullptr);
    static std::mutex mutex;

    if (logger == nullptr)
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (logger == nullptr)
        {
            std::cout << "[Logger Init] Initializing logger from config: "
                << config_path << std::endl;

            try
            {
                auto config = YAML::LoadFile(config_path);
                auto log_config = config["logging"];

                std::vector<spdlog::sink_ptr> sinks;

                // 控制台sink
                if (log_config["console"]["enabled"].as<bool>())
                {
                    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
                    console_sink->set_pattern(log_config["console"]["pattern"].as<std::string>());
                    console_sink->set_level(LEVEL_MAP.at(log_config["console"]["level"].as<std::string>()));
                    sinks.push_back(console_sink);
                }

                // 文件sink
                if (log_config["file"]["enabled"].as<bool>())
                {
                    auto log_dir = log_config["file"]["absolute_path"].as<std::string>();
                    auto log_filename = log_config["file"]["filename"].as<std::string>();
                    auto full_path = fs::path(log_dir) / log_filename;

                    fs::create_directories(log_dir);

                    if (log_config["file"]["rotation"]["type"].as<std::string>() == "daily")
                    {
                        auto file_sink = std::make_shared<spdlog::sinks::daily_file_sink_mt>(
                            full_path.string(), 0, 0, false,
                            log_config["file"]["rotation"]["max_files"].as<int>());
                        file_sink->set_pattern(log_config["file"]["pattern"].as<std::string>());
                        sinks.push_back(file_sink);
                    }
                    else
                    {
                        // 基于大小的轮转
                        auto max_size = log_config["file"]["rotation"]["max_size"].as<size_t>() * 1024 * 1024;
                        // MB转换为字节
                        auto max_files = log_config["file"]["rotation"]["max_files"].as<int>();

                        auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
                            full_path.string(), max_size, max_files);
                        file_sink->set_pattern(log_config["file"]["pattern"].as<std::string>());
                        sinks.push_back(file_sink);
                    }
                }

                // 使用DEFAULT_LOGGER_NAME创建logger
                logger = std::make_shared<spdlog::logger>(DEFAULT_LOGGER_NAME, sinks.begin(), sinks.end());

                // 设置日志级别
                logger->set_level(LEVEL_MAP.at(log_config["global_level"].as<std::string>()));
                logger->flush_on(LEVEL_MAP.at(log_config["flush_level"].as<std::string>()));

                // 注册logger
                spdlog::register_logger(logger);
                std::cout << "[Logger Init] Logger initialized successfully with name: "
                    << DEFAULT_LOGGER_NAME << std::endl;
            }
            catch (const std::exception& e)
            {
                std::cerr << "[Logger Error] Initialization failed: " << e.what() << std::endl;
                // 创建应急logger
                logger = spdlog::stdout_color_mt(DEFAULT_LOGGER_NAME);
                logger->set_level(spdlog::level::info);
                logger->error("Failed to initialize logger from config: {}", e.what());
            }
        }
    }
    return logger;
}

std::shared_ptr<spdlog::logger> get_logger()
{
    static auto logger = init_logger_from_config(CONFIG_FILE_PATH);
    return logger;
}

void set_log_level(const std::string& level)
{
    if (auto logger = spdlog::get(DEFAULT_LOGGER_NAME))
    {
        logger->set_level(LEVEL_MAP.at(level));
    }
}
