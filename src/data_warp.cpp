#include <data_warp.h>
#include <UI.h>         // for EventActivator
// ======= common ========
static std::shared_ptr<Line> getMinRightAngleIntersection(std::shared_ptr<Line> current_line, std::vector<std::shared_ptr<Line>> trajectory_lines){
    // TODO:
    return nullptr;
    // return DrawWarp::GetInstance().CreateShape<Line>(0, 0, 1000, 1000, Colors::BLACK);
}

// ===== test ======
void xdn_test::apply() override {
    auto a = DrawWarp::GetInstance().CreateShape<Line>(0, 0, 200, 0, Colors::BLACK);
    auto b = DrawWarp::GetInstance().CreateShape<Line>(100, 0, 300, 0, Colors::RED);
    DrawWarp::GetInstance().addShape(a);
    DrawWarp::GetInstance().addShape(b);
    auto [_, p] = a->findIntersection(*b);
    std::cout << "Line: " << p.getPoint().x << ", " << p.getPoint().y << std::endl;
}

// ===== Algorithm1:2006 =====
Algorithms1::Algorithms1(std::vector<std::shared_ptr<Shape>> polygon_data){
    this->polygon_data = polygon_data;
}
void Algorithms1::apply(){
    // TODO:
    
}

// ===== Algorithm2:2024 =====
Algorithms2::Algorithms2(std::vector<std::shared_ptr<Shape>> polygon_data){
    this->polygon_data = polygon_data;
}
void Algorithms1::apply(){
    // TODO:
    
}