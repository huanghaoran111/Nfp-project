#include <UI.h>
#include <Logger.h>
#include <data_warp.h>
#include <FileFinder.hpp>
#include <chrono>

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
    GLFWwindow* window = glfwCreateWindow(1920, 1080, "NFP Solver System", NULL, NULL);
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
    // io.Fonts->AddFontFromFileTTF("C:/windows/fonts/times.ttf", 20.0f, NULL, io.Fonts->GetGlyphRangesDefault());
    io.Fonts->AddFontFromFileTTF("C:/windows/fonts/STXIHEI.TTF", 20.0f, NULL, io.Fonts->GetGlyphRangesChineseSimplifiedCommon());

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
    windowManager->CreateWindows<OptionWindow>(std::string("option"));
    windowManager->CreateWindows<CanvasWindow>(std::string("Main Canvas"));
    logger.setWindow(windowManager->CreateWindows<LogWindow>(std::string("Log")));
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
        std::function<void(std::vector<std::vector<std::shared_ptr<NFP::Point>>>)>
        (
            [this](std::vector<std::vector<std::shared_ptr<NFP::Point>>> a){
                this->data = a;
                this->updateData();
            }
        )
    );
    EventActivator::GetInstance().RegisterEvent("clearData", [this](){data.clear();this->updateData();});
    EventActivator::GetInstance().RegisterEvent("parseOption", std::function<void(unsigned int)>
        (
            [this](unsigned int count){
                configureOptions(this, count);
            }
        )
    );
}

void ShowMessageBox(const char* title, const char* message) {
    ImGui::OpenPopup(title); // 触发弹窗
    if (ImGui::BeginPopupModal(title, nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("%s", message);
        ImGui::Separator();
        if (ImGui::Button("OK", ImVec2(120, 0))) {
            ImGui::CloseCurrentPopup(); // 关闭弹窗
        }
        ImGui::EndPopup();
    }
}

void configureOptions(CanvasWindow* canvas_window, unsigned int options){
    if (canvas_window->data.size() == 0 && options != (1 << 6)) {
        EventActivator::GetInstance().RegisterEvent("resetOption", std::function<void()>());
        return;
    }
    if(canvas_window->data.size() == 0){
        return;
    }
    // std::cout << (options & ~(1 << 6)) << std::endl;
    DrawWarp::GetInstance().clearShapes();
    if(options & 0x01){
        DWAddShape<NFP::Polygon>(std::make_shared<NFP::Polygon>(canvas_window->data[0]));
    }
    if(options & 0x02){
        DWAddShape<NFP::ConvexityPolygon>(std::make_shared<NFP::ConvexityPolygon>(canvas_window->data[0]));
    }
    if(options & 0x04){
        DWAddShape<NFP::TriangulatedPolygon>(std::make_shared<NFP::TriangulatedPolygon>(canvas_window->data[0]));
    }
    
    if(options & 0x08){
        DWAddShape<NFP::Polygon>(std::make_shared<NFP::Polygon>(canvas_window->data[1]));
    }
    if(options & 0x10){
        DWAddShape<NFP::ConvexityPolygon>(std::make_shared<NFP::ConvexityPolygon>(canvas_window->data[1]));
    }
    if(options & 0x20){
        DWAddShape<NFP::TriangulatedPolygon>(std::make_shared<NFP::TriangulatedPolygon>(canvas_window->data[1]));
    }
    if(options & 0x80){
        auto algo = std::make_shared<NFP::TrajectoryNFPAlgorithm>(canvas_window->data);
        TimingExp(algo->apply())
    }
    if(options & 0x100){
        auto algo = std::make_shared<NFP::LocalContourNFPAlgorithm>(canvas_window->data);
        TimingExp(algo->apply())
    }
    if(options & 0x200){
        auto algo = std::make_shared<NFP::TwoLocalContourNFPAlgorithm>(canvas_window->data);
        TimingExp(algo->apply())
    }
    
    if(options & 0x400){
        auto algo = std::make_shared<NFP::DelaunayTriangulationNFPAlgorithm>(canvas_window->data);
        TimingExp(algo->apply())
    }
    if(options & 0x800){
        auto algo = std::make_shared<NFP::MinkowskiSumNFPAlgorithm>(canvas_window->data);
        TimingExp(algo->apply())
    }
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
        auto a = std::make_shared<NFP::xdn_test>();
        a->apply();
        auto b = std::make_shared<NFP::xdn_test>();
        b->apply();
        a->apply();
        // auto c = std::make_shared<TestCases>();
        //std::cout << "Test result: " << std::endl;
        // c->apply();
        // auto shape = DWCreateShape<Point>(0.f,0.f);
        // shape->setIdx(0);
        std::vector<std::shared_ptr<NFP::Point>> ps;
        #define Add_Shape(s, e, i) \
        auto k##i = DrawWarp::GetInstance().CreateShape<NFP::Point>(s, e); \
        k##i->setIdx(i);\
        ps.push_back(k##i);

        Add_Shape(0.f, 0.f, 0)
        Add_Shape(200.f, 0.f, 1)
        Add_Shape(200.f, 100.f, 2)
        Add_Shape(125.f, 125.f, 3)
        Add_Shape(100.f, 200.f, 4)
        Add_Shape(0.f, 200.f, 5)
        #undef Add_Shape
        
        // std::shared_ptr<NFP::Shape> pp = DWCreateShape<NFP::TriangulatedPolygon>(ps, "A");

        // std::vector<std::shared_ptr<Shape>> vecp = {pp};
        // std::vector<std::vector<std::shared_ptr<NFP::Point>>> vecp = {ps, ps};
        // auto test = std::make_shared<NFP::DelaunayTriangulationNFPAlgorithm>(vecp);
        // test->apply();
        //std::cout << "size: " << aa->getLines().size() << std::endl;
        //auto km = DWCreateShape<Point>(0.f, 0.f);
    }
}

void CanvasWindow::HandleZoom() {
    ImGuiIO& io = ImGui::GetIO();

    // 左键拖动平移
    if (ImGui::IsWindowHovered() && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        // 只在鼠标刚按下时记录初始位置
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            drag_start_mouse_pos_ = ImGui::GetMousePos();
            drag_start_canvas_origin_ = canvas_origin_;
        }
        
        // 计算鼠标移动后的期望逻辑位置
        ImVec2 current_mouse_pos = ImGui::GetMousePos();
        ImVec2 mouse_delta = ImVec2(
            (current_mouse_pos.x - drag_start_mouse_pos_.x) / zoom_,
            (current_mouse_pos.y - drag_start_mouse_pos_.y) / zoom_
        );
        
        // 更新画布原点，保持鼠标下的逻辑位置不变
        canvas_origin_.x = drag_start_canvas_origin_.x + mouse_delta.x * zoom_;
        canvas_origin_.y = drag_start_canvas_origin_.y + mouse_delta.y * zoom_;
    }
    if (ImGui::IsWindowHovered() && io.MouseWheel != 0.f){
        auto mouse_pos = ImGui::GetMousePos();
        auto logical_mouse_pos = GetLogicalPos(mouse_pos);
        const float zoomSpeed = 0.15f; // 缩放速度系数
        float new_zoom = zoom_ * (1.0f + io.MouseWheel * zoomSpeed);
        const float minZoom = 0.05f;  // 最小缩放
        const float maxZoom = 20.0f;  // 最大缩放
        new_zoom = ImClamp(new_zoom, minZoom, maxZoom);
        if (new_zoom == zoom_) {
            return;
        }
        const ImVec2 mouse_offset_from_origin = ImVec2(
            mouse_pos.x - canvas_origin_.x,
            mouse_pos.y - canvas_origin_.y
        );
        
        // 计算新的原点位置
        canvas_origin_.x = mouse_pos.x - mouse_offset_from_origin.x * (new_zoom / zoom_);
        canvas_origin_.y = mouse_pos.y - mouse_offset_from_origin.y * (new_zoom / zoom_);
        
        zoom_ = new_zoom;
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
        auto transform = [this](NFP::Vec2 p){
            return this->TransformPoint(ImVec2(p.x, p.y));
        };
        if(EventActivator::GetInstance().HasEvent("ShowGrid")){
            DrawGrid(draw_list, canvas_origin_, canvas_size);
        }
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
    const float grid_step_pixels = 50.0f * zoom_;  // 固定网格步长（屏幕像素）
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
static void CenterNextText(const char* text) {
    float columnWidth = ImGui::GetColumnWidth();
    float textWidth = ImGui::CalcTextSize(text).x;
    ImGui::SetCursorPosX(ImGui::GetColumnIndex() * columnWidth + (columnWidth - textWidth) * 0.5f);
    ImGui::Text(text);
}

void OptionWindow::Content(){
    static bool show_point_tag = false;
    static bool show_point_pos = false;
    static char folderPath[256] = "./data"; // 默认路径，可编辑
    if(ImGui::Checkbox("Show Point Tag", &show_point_tag)){
        EventActivator::GetInstance().RegisterEvent("ShowPointTag", std::function<void(bool*)>([this](bool* is_show){*is_show=show_point_tag;}));
    }
    ImGui::SameLine();
    if(ImGui::Checkbox("Show Point Pos", &show_point_pos)){
        EventActivator::GetInstance().RegisterEvent("ShowPointPos", std::function<void(bool*)>([this](bool* is_show){*is_show=show_point_pos;}));
    }
    ImGui::Dummy(ImVec2(0.0f, 10.0f));
    ImGui::Text("JSON DIR: ");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(300.0f);
    ImGui::InputText(" ", folderPath, sizeof(folderPath));
    ImGui::SameLine();
    if(ImGui::Button("Refresh")) {
        jsonName = RefreshJsonFileList(folderPath);
    }
    ImGui::Dummy(ImVec2(0.0f, 10.0f));
    // ImGui::SameLine();
    ImGui::BeginChild("FileListScrolling", ImVec2(0, 200), true, ImGuiWindowFlags_HorizontalScrollbar);
    {
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
                        auto m_data = getDataFromJson(std::string(folderPath) + std::string("/") + jsonName[i]);
                        EventActivator::GetInstance().ActivateEvent("clearData");
                        EventActivator::GetInstance().ActivateEvent("getData", m_data);
                        oldselectedFileIndex = selectedFileIndex;
                    }
                }
            }
        }
        
     }
    ImGui::EndChild(); // 结束滚动区域
    ImGui::Dummy(ImVec2(0.0f, 10.0f));
    static unsigned int count = 0;
    static bool PolygonA = false;
    static bool ConvexityPolygonA = false;
    static bool TriangulatedPolygonA = false;
    static bool PolygonB = false;
    static bool ConvexityPolygonB = false;
    static bool TriangulatedPolygonB = false;
    static bool TrajectoryLines = false;
    static int selectedMode = 0;
    static int NoneAlgoorithm = 0;
    static int TrajectoryNFPAlgorithm = 1;
    static int LocalContourNFPAlgorithm = 2;
    static int TwoLocalContourNFPAlgorithm = 3;
    static int DelaunayTriangulationNFPAlgorithm = 4;
    static int MinkowskiSumNFPAlgorithm = 5;

    static int colWidth = ImGui::GetColumnWidth();
    ImGui::Columns(2, "Modules", true);
    ImGui::SetColumnWidth(ImGui::GetColumnIndex(), colWidth / 2);
    // ImGui::SetColumnWidth(1, ImGui::GetColumnWidth() / 2);
    ImGui::Separator();
    CenterNextText("Polygon A");
    ImGui::PushID("GroupA");
    ImGui::Checkbox("Polygon", &PolygonA);
    ImGui::Checkbox("Convexity Polygon", &ConvexityPolygonA);
    ImGui::Checkbox("Triangulated Polygon", &TriangulatedPolygonA);
    ImGui::PopID();
    ImGui::NextColumn();
    ImGui::SetColumnWidth(ImGui::GetColumnIndex(), colWidth / 2);
    ImGui::PushID("GroupB");
    CenterNextText("Polygon B");
    ImGui::Checkbox("Polygon", &PolygonB);
    ImGui::Checkbox("Convexity Polygon", &ConvexityPolygonB);
    ImGui::Checkbox("Triangulated Polygon", &TriangulatedPolygonB);
    ImGui::PopID();
    ImGui::Columns(1); // 恢复单列
    ImGui::Separator();
    // ImGui::Checkbox("TrajectoryLines", &TrajectoryLines);
    ImGui::Checkbox("Show The TrajectoryLines", &TrajectoryLines);
    if(TrajectoryLines){
        EventActivator::GetInstance().RegisterEvent("ShowTrajectoryLines", std::function<void(bool*)>([this](bool* ShowTrajectoryLines){*ShowTrajectoryLines=TrajectoryLines;}));
    }
    ImGui::Separator();
    CenterNextText("NFP Algorithm");
    ImGui::RadioButton("NoneAlgoorithm", &selectedMode, NoneAlgoorithm);
    ImGui::RadioButton("TrajectoryNFPAlgorithm", &selectedMode, TrajectoryNFPAlgorithm);
    ImGui::RadioButton("LocalContourNFPAlgorithm", &selectedMode, LocalContourNFPAlgorithm);
    ImGui::RadioButton("TwoLocalContourNFPAlgorithm", &selectedMode, TwoLocalContourNFPAlgorithm);
    ImGui::RadioButton("DelaunayTriangulationNFPAlgorithm", &selectedMode, DelaunayTriangulationNFPAlgorithm);
    ImGui::RadioButton("MinkowskiSumNFPAlgorithm", &selectedMode, MinkowskiSumNFPAlgorithm);
    ImGui::Separator();
    if (EventActivator::GetInstance().HasEvent("resetOption")) {
        count = 1 << 6;
        PolygonA = false;
        ConvexityPolygonA = false;
        TriangulatedPolygonA = false;
        PolygonB = false;
        ConvexityPolygonB = false;
        TriangulatedPolygonB = false;
        selectedMode = 0;
        EventActivator::GetInstance().RemoveEvent("resetOption");
    }
    if(PolygonA) count |= 1 << 0; else count &= ~(1 << 0);
    if(ConvexityPolygonA) count |= 1 << 1; else count &= ~(1 << 1);
    if(TriangulatedPolygonA) count |= 1 << 2; else count &= ~(1 << 2);
    if(PolygonB) count |= 1 << 3; else count &= ~(1 << 3);
    if(ConvexityPolygonB) count |= 1 << 4; else count &= ~(1 << 4);
    if(TriangulatedPolygonB) count |= 1 << 5; else count &= ~(1 << 5);
    if(selectedMode == NoneAlgoorithm) count |= 1 << 6; else count &= ~(1 << 6);
    if(selectedMode == TrajectoryNFPAlgorithm) count |= 1 << 7; else count &= ~(1 << 7);
    if(selectedMode == LocalContourNFPAlgorithm) count |= 1 << 8; else count &= ~(1 << 8);
    if(selectedMode == TwoLocalContourNFPAlgorithm) count |= 1 << 9; else count &= ~(1 << 9);
    if(selectedMode == DelaunayTriangulationNFPAlgorithm) count |= 1 << 10; else count &= ~(1 << 10);
    if(selectedMode == MinkowskiSumNFPAlgorithm) count |= 1 << 11; else count &= ~(1 << 11);
    if(TrajectoryLines) count |= 1 << 12; else count &= ~(1 << 12);
    EventActivator::GetInstance().ActivateEvent("parseOption", count);

    CenterNextText("Other Options");
    static bool showGrid = true;
    ImGui::Checkbox("网格线", &showGrid);
    if(showGrid){
        EventActivator::GetInstance().RegisterEvent("ShowGrid", std::function<void()>());
    }else{
        if(EventActivator::GetInstance().HasEvent("ShowGrid")){
            EventActivator::GetInstance().RemoveEvent("ShowGrid");
        }
    }
    static bool TimingAlgo = false;
    ImGui::Checkbox("对算法计时", &TimingAlgo);
    if(TimingAlgo){
        EventActivator::GetInstance().RegisterEvent("TimingAlgo", std::function<void()>());
    }else{
        if(EventActivator::GetInstance().HasEvent("TimingAlgo")){
            EventActivator::GetInstance().RemoveEvent("TimingAlgo");
        }
    }
}