add_library(imgui)

SET(imgui_inc
    .
    backends
    misc
    ../glfw/include
)

SET(imgui
    imgui_demo.cpp
    imgui_draw.cpp
    imgui_tables.cpp
    imgui_widgets.cpp
    imgui.cpp
    backends/imgui_impl_glfw.cpp
    backends/imgui_impl_opengl3.cpp
)

target_sources(imgui PRIVATE ${imgui})
target_include_directories(imgui PUBLIC ${imgui_inc})   
