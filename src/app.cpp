#define IMGUI_DEFINE_MATH_OPERATORS
#include <UI.h>

int main(int argc, char const *argv[])
{
    auto window = std::make_unique<Window>();
    window->run();
    return 0;
}
