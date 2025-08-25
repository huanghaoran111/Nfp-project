#include <draw_warp.h>
#include <functional>

void DrawWarp::addShape(std::shared_ptr<Shape> shape) {
    shapes.push_back(shape);
}

void DrawWarp::drawShapes(ImDrawList* draw_list, std::function<ImVec2(Vec2)> func){
    for(auto shape : shapes){
        shape->draw(draw_list, func);
    }
}

void DWAddShape(std::shared_ptr<Shape> shape) {
    DrawWarp::GetInstance().addShape(shape);
}