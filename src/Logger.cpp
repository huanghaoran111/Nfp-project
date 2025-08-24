#include <Logger.h>
#include <UI.h>
UnifiedLogger::UnifiedLogger(const std::string& name, const std::string& filename) 
    : spd_logger_(name, { 
        std::make_shared<spdlog::sinks::stderr_sink_st>(),
        std::make_shared<spdlog::sinks::basic_file_sink_st>(filename) 
    })
    , log_window_(nullptr) 
{
    spd_logger_.set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] %v");
}

std::string UnifiedLogger::FormatForGUI(LogLevel level, const std::string& msg) {
    const char* level_names[] = {"TRACE", "DEBUG", "INFO", "WARN", "ERROR", "CRITICAL"};
    return fmt::format("[{}] {}", level_names[static_cast<int>(level)], msg);
}

void UnifiedLogger::Log(LogLevel level, const std::string& message) {
    // 写入spdlog
    spd_logger_.log(static_cast<spdlog::level::level_enum>(level), message);
    
    // 写入GUI窗口（带颜色标签）
    if(log_window_)
        log_window_->AddLog(FormatForGUI(level, message));
}

void UnifiedLogger::RenderGUI() { log_window_->Render(); }