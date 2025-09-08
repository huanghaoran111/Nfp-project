#include <UI.h>
#include <Logger.h>
#include <data_warp.h>
#include <FileFinder.hpp>

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
    io.Fonts->AddFontFromFileTTF("c:/windows/fonts/simhei.ttf", 20.0f, NULL, io.Fonts->GetGlyphRangesChineseSimplifiedCommon());

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
    // 子窗体创建
    windowManager = std::make_unique<WindowManager>();
    windowManager->CreateWindows<CanvasWindow>(std::string("Main Canvas"));
    logger.setWindow(windowManager->CreateWindows<LogWindow>(std::string("Log")));
    windowManager->CreateWindows<OptionWindow>(std::string("option"));
}

/**
 * @brief 主循环函数，持续运行直到窗口关闭
 * 该函数处理窗口事件、渲染场景和管理GUI界面
 */
void Window::run(){
    // 主循环，持续运行直到GLFW窗口应该关闭
    while (!glfwWindowShouldClose(glfwWindow))
    {
        // 处理所有待处理的GLFW事件
        glfwPollEvents();
        // 添加：清除颜色缓冲区
        // 设置清除颜色为浅灰色（RGBA格式）
        glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
        // 执行清除操作，只清除颜色缓冲区
        glClear(GL_COLOR_BUFFER_BIT);
        // 开始新的ImGui帧
        ImGui_ImplOpenGL3_NewFrame();  // OpenGL3后端新帧准备
        ImGui_ImplGlfw_NewFrame();     // GLFW后端新帧准备
        ImGui::NewFrame();             // ImGui上下文新帧准备
        // 窗体渲染
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
    // 调整画布原点初始位置 就改这里 无效的话 把imgui.ini删了
    canvas_origin_ = {1920 * 0.7f / 2, 1080 * 0.6f / 2};
    EventActivator::GetInstance().RegisterEvent("updateData", [this](){this->updateData();});
    EventActivator::GetInstance().RegisterEvent("getData", 
        std::function<void(std::vector<std::shared_ptr<Shape>>)>
        (
            [this](std::vector<std::shared_ptr<Shape>> a){
                data.emplace_back(a);
                this->updateData();
            }
        )
    );
    EventActivator::GetInstance().RegisterEvent("clearData", [this](){data.clear();this->updateData();});
}

void CanvasWindow::PreRender() {
    flags_ |= ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove;
    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
    ImVec2 viewport_size = ImGui::GetMainViewport()->Size;
    ImGui::SetNextWindowSize({viewport_size.x * 0.7f, viewport_size.y * 0.6f});
    if(need_update){
        need_update = false;
        DrawWarp::GetInstance().clearShapes();
        // std::cout << data.size() << std::endl;
        // TODO: 写入算法接口
        auto a = std::make_shared<xdn_test>();
        a->apply();
        auto b = std::make_shared<xdn_test>();
        b->apply();
        a->apply();
        auto c = std::make_shared<TestCases>();
        //std::cout << "Test result: " << std::endl;
        c->apply();
        // auto shape = DWCreateShape<Point>(0.f,0.f);
        // shape->setIdx(0);
    }
}

void CanvasWindow::Content() {
    // 获取ImGui的IO接口，包含输入状态和配置信息
    ImGuiIO& io = ImGui::GetIO();
    ImVec2 viewport_size = ImGui::GetMainViewport()->Size; // 屏幕分辨率
    // ImVec2 canvas_size = {viewport_size.x * 0.7f, viewport_size.y * 0.6f};
    ImVec2 canvas_size = ImGui::GetContentRegionAvail();
    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
    // ImGui::Text("Mouse Pos: (%.1f, %.1f)", io.MousePos.x, io.MousePos.y);
    // ImGui::Text("Mouse Down: %d", ImGui::IsMouseDown(ImGuiMouseButton_Left));
    // ImGui::Text("zoom: %.3f", zoom_);
    // auto m = GetLogicalMousePos(ImGui::GetMousePos());
    // ImGui::Text("mouse_logical pos: %.3f, %.3f", m.x, m.y);
    // ImGui::Text("Frame %d: origin=(%.1f,%.1f)", 
    //     ImGui::GetFrameCount(), canvas_origin_.x, canvas_origin_.y);
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
        auto transform = [this](Vec2 p){
            return this->TransformPoint(ImVec2(p.x, p.y));
        };
        DrawGrid(draw_list, canvas_origin_, canvas_size);
        DrawWarp::GetInstance().drawShapes(draw_list, transform);
    }
    ImGui::EndChild();
    // 解决每次更新的问题
    std::vector<std::string> memlist = {
        std::string("ShowPointTag"),
        std::string("ShowPointPos")
    };
    for(auto s : memlist){
        if(EventActivator::GetInstance().HasEvent(s)){
            EventActivator::GetInstance().RemoveEvent(s);
        }
    }
    
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

OptionWindow::OptionWindow(const std::string& name) : WindowComponent(name) {
    SetFlags(ImGuiWindowFlags_NoTitleBar);
    selectedFileIndex = -1;
    oldselectedFileIndex = -1;
    jsonName = RefreshJsonFileList("./data");
}

void OptionWindow::PreRender() {
    flags_ |= ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove;
    ImVec2 viewport_size = ImGui::GetMainViewport()->Size;
    ImGui::SetNextWindowPos(ImVec2(viewport_size.x * 0.7f, 0), ImGuiCond_Always);
    ImGui::SetNextWindowSize({viewport_size.x * 0.3f, viewport_size.y * 1.f});
}

void OptionWindow::Content(){
    static bool show_point_tag = false;
    static bool show_point_pos = false;
    if(ImGui::Checkbox("show point tag", &show_point_tag)){
        EventActivator::GetInstance().RegisterEvent("ShowPointTag", std::function<void(bool*)>([this](bool* is_show){*is_show=show_point_tag;}));
    }
    ImGui::SameLine();
    if(ImGui::Checkbox("show point pos", &show_point_pos)){
        EventActivator::GetInstance().RegisterEvent("ShowPointPos", std::function<void(bool*)>([this](bool* is_show){*is_show=show_point_pos;}));
    }
    if(ImGui::Button("Refresh")) {
        jsonName = RefreshJsonFileList("./data");
    }
    ImGui::SameLine();
    ImGui::Text("Folder: %s", "./data");
    ImGui::Separator();
    if (jsonName.empty()) {
        ImGui::TextColored(ImVec4(1, 0.5f, 0, 1), "No JSON files found in directory");
    } else {
        for (int i = 0; i < jsonName.size(); i++) {
            // 高亮选中的文件
            // if (i == selectedFileIndex) {
            //     ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0, 1, 0, 1));
            // }
            
            bool isSelected = (i == selectedFileIndex);
            if (ImGui::Selectable(jsonName[i].c_str(), isSelected)) {
                selectedFileIndex = i;
            }

            // 双击检测放在Selectable块外
            if (isSelected && ImGui::IsMouseDoubleClicked(0)) {
                if (oldselectedFileIndex != selectedFileIndex) {
                    // 加载数据
                    auto m_data = std::vector<std::shared_ptr<Shape>>();
                    m_data.push_back(DrawWarp::GetInstance().CreateShape<Line>(0, 0, 1, 2, Colors::BLACK));
                    m_data.push_back(DrawWarp::GetInstance().CreateShape<Line>(0, 0, 1, 2, Colors::BLACK));
                    m_data.push_back(DrawWarp::GetInstance().CreateShape<Line>(0, 0, 1, 2, Colors::BLACK));
                    EventActivator::GetInstance().ActivateEvent("clearData");
                    EventActivator::GetInstance().ActivateEvent("getData", m_data);
                    oldselectedFileIndex = selectedFileIndex;
                }
            }
            
            // if (i == selectedFileIndex) {
            //     ImGui::PopStyleColor();
            // }
        }
    }
}