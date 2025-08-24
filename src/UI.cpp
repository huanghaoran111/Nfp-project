#include <UI.h>
#include <Logger.h>
WindowComponent::WindowComponent(const std::string& name) : name_(name), visible_(true) {}

static void GLFWInit()
{
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
}

/**
 * @brief 创建GLFW窗口的函数
 * @return 返回创建的GLFW窗口指针，如果创建失败则返回NULL
 */
static GLFWwindow* Create_glfw_Window()
{
    // 创建一个1920x1080分辨率的窗口，标题为"NFP"
    GLFWwindow* window = glfwCreateWindow(1920, 1080, "NFP", NULL, NULL);
    if (window == NULL)  // 检查窗口是否创建成功
    {
        std::cout << "Failed to create GLFW window" << std::endl;  // 输出错误信息
        glfwTerminate();  // 终止GLFW
        return nullptr;
    }
    glfwMakeContextCurrent(window);  // 将窗口的上下设置为当前上下文
    glfwSwapInterval(1);  // 设置交换间隔为1，启用垂直同步
    return window;  // 返回创建的窗口对象
}

/**
 * @brief 初始化ImGui库
 * @param window GLFW窗口指针，用于绑定ImGui上下文
 */
static void InitImGui(GLFWwindow* window)
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    // 获取窗口用户指针，转换为Window类型
    // auto win = static_cast<Window*>(glfwGetWindowUserPointer(window));
    // 检查ImGui版本并创建ImGui上下文
    // 获取ImGui IO配置并启用键盘和游戏手柄控制
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    // Load Fonts
    io.Fonts->AddFontFromFileTTF("c:/windows/fonts/simhei.ttf", 13.0f, NULL, io.Fonts->GetGlyphRangesChineseSimplifiedCommon());

    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");
    ImGui::StyleColorsLight();
}

Window::Window(){
    GLFWInit();
    glfwWindow = Create_glfw_Window();
    if (glfwWindow == nullptr)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        exit(-1);
    }
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        exit(-1);
    }
    glfwSetWindowUserPointer(glfwWindow, this);
    InitImGui(glfwWindow);
    glfwSetScrollCallback(glfwWindow, ImGui_ImplGlfw_ScrollCallback);
    windowManager = std::make_unique<WindowManager>();
    windowManager->CreateWindows<CanvasWindow>(std::string("Main Canvas"));
    logger.setWindow(windowManager->CreateWindows<LogWindow>(std::string("Log")));
}

void Window::run(){
    while (!glfwWindowShouldClose(glfwWindow))
    {
        glfwPollEvents();
        // 添加：清除颜色缓冲区
        glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        windowManager->RenderAll();
        // logger.RenderGUI();
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());  // 添加：渲染 ImGui 绘制数据
        // 交换缓冲区
        glfwSwapBuffers(glfwWindow);
    }
}

Window::~Window(){
    // 添加：清理 ImGui 资源
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    
    glfwDestroyWindow(glfwWindow);
    glfwTerminate();
}

void WindowManager::RenderAll() {
    for (auto& window : windows_) {
        window->Render();
    }
}

void LogWindow::CopyLogsToClipboard() {
    std::string all_logs;
    for (const auto& log : logs_) {
        all_logs += log + "\n";
    }
    ImGui::SetClipboardText(all_logs.c_str());
}
void LogWindow::AddLog(const std::string& message) {
    logs_.push_back(message);
    if (logs_.size() > 1000) logs_.erase(logs_.begin()); // 限制日志数量
}

void LogWindow::PostRender() {
    // 恢复样式
    // ImGui::PopStyleVar(4);
    ImGui::PopStyleColor(2);
}

/**
 * @brief 日志窗口的内容显示函数
 * 该函数负责渲染日志显示区域和控制栏，包括日志列表、自动滚动功能以及控制按钮
 */
void LogWindow::Content() {
    // 定义控制栏的高度为30像素
    const float control_bar_height = 30.0f;
    // 开始创建日志内容显示区域，设置高度为剩余空间减去控制栏高度
    // 启用水平滚动条
    ImGui::BeginChild("LogContent", ImVec2(0, -control_bar_height), true, ImGuiWindowFlags_HorizontalScrollbar);
    // 显示日志内容
    for (const auto& log : logs_) {
        // ImGui::TextUnformatted(log.c_str());
        ImGui::TextWrapped("%s", log.c_str());
        ImGui::Separator(); // 每条日志间加分隔线
    }
    
    // 自动滚动到底部
    if (auto_scroll_ && ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 5.0f) {
        ImGui::SetScrollHereY(1.0f);
    }
    ImGui::EndChild();
    ImGui::PopStyleVar(2);
    ImGui::PopStyleColor(2);
    ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(0, 0, 0, 255));
    ImGui::PushStyleColor(ImGuiCol_ChildBg, IM_COL32(255, 255, 255, 255));
    ImGui::BeginChild("ControlBar", ImVec2(0, 0), false);
    {
        ImGui::Separator();
        ImGui::Checkbox("Auto Scroll", &auto_scroll_);
        ImGui::SameLine();
        if (ImGui::Button("Clear")) logs_.clear();
        ImGui::SameLine();
        if (ImGui::Button("Copy All")) CopyLogsToClipboard();
    }
    ImGui::EndChild();
    
}

void LogWindow::PreRender() {
    flags_ |= ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove;
    ImVec2 viewport_size = ImGui::GetMainViewport()->Size;
    ImGui::SetNextWindowPos(ImVec2(0, viewport_size.y * 0.6f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(viewport_size.x * 0.7f, viewport_size.y * 0.4f));
    ImGui::PushStyleColor(ImGuiCol_ChildBg, IM_COL32(25, 25, 25, 255));
    ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(240, 240, 240, 255));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0)); // 移除窗口内边距
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4, 2));   // 紧凑行间距
}

LogWindow::LogWindow(const std::string& name) : WindowComponent(name) {
    SetFlags(ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove);
}

void WindowComponent::Render() {
    if (!visible_) return;
    
    PreRender();
    if (ImGui::Begin(name_.c_str(), &visible_, flags_)) {
        Content();
    }
    ImGui::End();
    PostRender();
}

CanvasWindow::CanvasWindow(const std::string& name) : WindowComponent(name) {
    SetFlags(ImGuiWindowFlags_NoTitleBar);
    canvas_origin_ = {1920 * 0.7f / 2, 1080 * 0.6f / 2};
}

void CanvasWindow::PreRender() {
    flags_ |= ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove;
    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
    ImVec2 viewport_size = ImGui::GetMainViewport()->Size;
    ImGui::SetNextWindowSize({viewport_size.x * 0.7f, viewport_size.y * 0.6f});
}

void CanvasWindow::Content() {
    // 获取ImGui的IO接口，包含输入状态和配置信息
    ImGuiIO& io = ImGui::GetIO();
    ImVec2 viewport_size = ImGui::GetMainViewport()->Size; // 屏幕分辨率
    // ImVec2 canvas_size = {viewport_size.x * 0.7f, viewport_size.y * 0.6f};
    ImVec2 canvas_size = ImGui::GetContentRegionAvail();
    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
    ImGui::Text("Mouse Pos: (%.1f, %.1f)", io.MousePos.x, io.MousePos.y);
    ImGui::Text("Mouse Down: %d", ImGui::IsMouseDown(ImGuiMouseButton_Left));
    ImGui::Text("zoom: %.3f", zoom_);
    auto m = GetLogicalMousePos(ImGui::GetMousePos());
    ImGui::Text("mouse_logical pos: %.3f, %.3f", m.x, m.y);
    ImGui::Text("Frame %d: origin=(%.1f,%.1f)", 
        ImGui::GetFrameCount(), canvas_origin_.x, canvas_origin_.y);
    // ImGui::Text("origin: %.3f, %.3f", origin.x, origin.y);
    HandleShortcuts();
    // 创建画布子窗口
    ImGui::BeginChild("Canvas", canvas_size);
    {
        // 处理缩放
        HandleZoom();
        // ImDrawList* draw_list = ImGui::GetWindowDrawList();
        
        // 处理画布绘制和交互
        // HandleCanvasInteraction(draw_list, canvas_pos, canvas_size);
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        ImVec2 canvas_pos = ImGui::GetMousePos(); // 子窗口内容区起点
        DrawGrid(draw_list, canvas_origin_, canvas_size);
        // 绘制示例：红色矩形标记左上角
        draw_list->AddRectFilled(
            canvas_pos, 
            canvas_pos + ImVec2(50, 50), 
            IM_COL32(255, 0, 0, 255)
        );

        draw_list->AddLine(
            ImVec2(0, 0),
            ImGui::GetWindowSize(),
            IM_COL32(255, 0, 0, 255)
        );
    }
    ImGui::EndChild();
}

void CanvasWindow::DrawGrid(ImDrawList* draw_list, const ImVec2& canvas_screen_pos, const ImVec2& canvas_size) {
    const float grid_step_pixels = 50.0f;  // 固定网格步长（屏幕像素）
    const ImU32 grid_color = IM_COL32(150, 150, 150, 50); // 半透明灰色

    // 计算网格线的起始偏移（考虑画布平移）
    float offset_x = fmodf(canvas_origin_.x, grid_step_pixels);
    float offset_y = fmodf(canvas_origin_.y, grid_step_pixels);

    // 绘制水平网格线
    for (float y = offset_y; y < canvas_size.y; y += grid_step_pixels) {
        ImVec2 start = ImVec2(0, y);
        ImVec2 end = ImVec2(canvas_size.x, y);
        draw_list->AddLine(start, end, grid_color, 1.0f);
    }

    // 绘制垂直网格线
    for (float x = offset_x; x < canvas_size.x; x += grid_step_pixels) {
        ImVec2 start = ImVec2(x, 0);
        ImVec2 end = ImVec2(x, canvas_size.y);
        draw_list->AddLine(start, end, grid_color, 1.0f);
    }
}