#pragma once
#include <Shape.hpp>
#include <imgui.h>
#include <util.hpp>

using Shape = NFP::Shape;

struct DrawWarp {
    template<typename T, typename... Args>
    std::shared_ptr<T> CreateShape(Args&&... args){
        static_assert(std::is_base_of<NFP::Shape, T>::value, 
                     "T must be a derived class of Shape");
        return std::make_shared<T>(std::forward<Args>(args)...);
    }

    template<typename T>
    void addShape(std::shared_ptr<T> shape){ // Add a shape to the list of shapes to be drawn
        shapes.push_back(std::dynamic_pointer_cast<NFP::Shape>(shape));
    } 

    void drawShapes(ImDrawList* draw_list, std::function<ImVec2(Vec2)> func); // Draw all shapes in the list

    ~DrawWarp() = default; // Destructor

    static DrawWarp& GetInstance(){
        static DrawWarp instance;
        return instance;
    }

    int getShapeCount(){
        return shapes.size();
    }

    void clearShapes(){
        shapes.clear();
    }
private:
    std::vector<std::shared_ptr<NFP::Shape>> shapes;
};

template<typename T, typename... Args>
inline std::shared_ptr<T> DWCreateShape(Args&&... args){
    auto shape = DrawWarp::GetInstance().CreateShape<T>(std::forward<Args>(args)...);
    DrawWarp::GetInstance().addShape(shape);
    return shape;
}
template<typename T>
void DWAddShape(std::shared_ptr<T> shape) {
    static_assert(std::is_base_of<Shape, T>::value, 
        "T must be a derived class of Shape"
    );
    DrawWarp::GetInstance().addShape(shape);
}