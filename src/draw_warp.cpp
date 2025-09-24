#include <draw_warp.h>
#include <functional>

void DrawWarp::drawShapes(ImDrawList* draw_list, std::function<ImVec2(NFP::Vec2)> func){
    for(auto shape : shapes){
        shape->draw(draw_list, func);
    }
}

