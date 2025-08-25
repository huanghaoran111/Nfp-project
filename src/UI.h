#pragma once
#include <glad/glad.h>
#include <glfw/glfw3.h>

#include <iostream>
#include <vector>
#include <string>
#include <memory>

#define IMGUI_DEFINE_MATH_OPERATORS
#include <imgui_internal.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <draw_warp.h>

static void GLFWInit();
static void InitImGui(GLFWwindow* window);
static GLFWwindow* Create_glfw_Window();

class WindowComponent {
public:
    WindowComponent(const std::string& name);
    virtual ~WindowComponent() = default;

    // 主渲染函数
    virtual void Render();

    // 设置窗口标志
    void SetFlags(ImGuiWindowFlags flags) { flags_ |= flags; }
    
    // 可见性控制
    void Show() { visible_ = true; }
    void Hide() { visible_ = false; }
    bool IsVisible() const { return visible_; }

protected:
    // 子类需要实现的内容
    virtual void Content() = 0;
    
    // 可选的预处理和后处理
    virtual void PreRender() {}
    virtual void PostRender() {}

    std::string name_;
    bool visible_;
    ImGuiWindowFlags flags_ = 0;
    
    // 窗口位置和大小缓存
    ImVec2 position_ = {0, 0};
    ImVec2 size_ = {0, 0};
};

class CanvasWindow : public WindowComponent {
public:
    explicit CanvasWindow(const std::string& name);

protected:
    virtual void PreRender();

    void Content() override;

private:
    float zoom_ = 1.f;
    ImVec2 canvas_origin_;  // canvas_origin_为画布原点（逻辑坐标 (0,0) 点）在屏幕空间的位置​​（屏幕坐标）
    bool is_panning_ = false;
    ImVec2 last_mouse_pos_;
    const ImVec2 DEFAULT_ORIGIN_ = ImVec2(1920 * 0.7f / 2, 1080 * 0.6f / 2);

    void ResetView() {canvas_origin_ = DEFAULT_ORIGIN_;}

    // 坐标转换函数
    ImVec2 TransformPoint(const ImVec2& logical_pos) const {
        return ImVec2(
            canvas_origin_.x + logical_pos.x * zoom_,
            canvas_origin_.y + logical_pos.y * zoom_
        );
    }

    void HandleShortcuts() {
        ImGuiIO& io = ImGui::GetIO();
        if (ImGui::IsKeyPressed(ImGuiKey_R) && io.KeyCtrl) {  // 检测 Home 键
            ResetView();
        }
    }
    
    ImVec2 GetLogicalMousePos(const ImVec2& screen_pos) const {
        return ImVec2(
            (screen_pos.x - canvas_origin_.x) / zoom_,
            (screen_pos.y - canvas_origin_.y) / zoom_
        );
    }
    
    void HandleZoom() {
        ImGuiIO& io = ImGui::GetIO();

        // 左键拖动平移
        if (ImGui::IsWindowHovered() && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            canvas_origin_.x += io.MouseDelta.x * zoom_;
            canvas_origin_.y += io.MouseDelta.y * zoom_;
            // printf("Canvas Origin: (%.1f, %.1f)\n", canvas_origin_.x, canvas_origin_.y);
        }
    }

    void DrawGrid(ImDrawList* draw_list, const ImVec2& canvas_screen_pos, const ImVec2& canvas_size);
    
    // 处理画布交互
    void HandleCanvasInteraction(ImDrawList* draw_list, const ImVec2& canvas_pos, const ImVec2& canvas_size) {}
};

class LogWindow : public WindowComponent {
public:
    explicit LogWindow(const std::string& name);

protected:
    void PreRender() override;

    void Content() override;

    void PostRender();

public:
    void AddLog(const std::string& message);
private:
    std::vector<std::string> logs_;
    bool auto_scroll_ = true;
    void CopyLogsToClipboard();
};

class WindowManager {
public:
    template<typename T, typename... Args>
    std::shared_ptr<T> CreateWindows(Args&&... args) {
        auto window = std::make_shared<T>(std::forward<Args>(args)...);
        windows_.push_back(window);
        return window;
    }

    template<typename T, typename... Args>
    static std::shared_ptr<T> CreateWindows_without_list(Args&&... args) {
        auto window = std::make_shared<T>(std::forward<Args>(args)...);
        return window;
    }

    void RenderAll();
private:
    std::vector<std::shared_ptr<WindowComponent>> windows_;
};

struct Window{
    ImGuiWindowFlags window_flags = 0;
    GLFWwindow* glfwWindow;
    std::unique_ptr<WindowManager> windowManager;
    Window();
    void run();
    ~Window();
};

