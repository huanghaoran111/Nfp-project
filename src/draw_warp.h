#pragma once
#include <Shape.hpp>
#include <imgui.h>

struct DrawWarp {
    template<typename T, typename... Args>
    std::shared_ptr<T> CreateShape(Args&&... args){
        static_assert(std::is_base_of<Shape, T>::value, 
                     "T must be a derived class of Shape");
        return std::make_unique<T>(std::forward<Args>(args)...);
    }

    void addShape(std::shared_ptr<Shape> shape); // Add a shape to the list of shapes to be drawn

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
    std::vector<std::shared_ptr<Shape>> shapes;
};

template<typename T, typename... Args>
inline std::shared_ptr<T> DWCreateShape(Args&&... args){
    auto shape = DrawWarp::GetInstance().CreateShape<T>(std::forward<Args>(args)...);
    DrawWarp::GetInstance().addShape(shape);
    return shape;
}

void DWAddShape(std::shared_ptr<Shape> shape);