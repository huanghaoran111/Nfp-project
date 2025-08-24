#pragma once
#include <string>
#include <memory>
#include <spdlog/spdlog.h>
#include "spdlog/sinks/stdout_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"

#include <UI.h>

// 日志级别枚举（兼容spdlog）
enum class LogLevel {
    Trace, Debug, Info, Warn, Error, Critical
};
// 统一日志接口类
class UnifiedLogger {
public:
    UnifiedLogger(const std::string& name, const std::string& filename);

    // 统一写入接口
    void Log(LogLevel level, const std::string& message);

    // 渲染GUI窗口
    void RenderGUI();

    void setWindow(std::shared_ptr<LogWindow> window) {
        log_window_ = window;
    }
private:
    spdlog::logger spd_logger_;
    std::shared_ptr<LogWindow> log_window_;
    std::string FormatForGUI(LogLevel level, const std::string& msg);
};

static UnifiedLogger logger("App", "logs/app.log");