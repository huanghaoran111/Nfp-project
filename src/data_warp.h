#pragma once

#include <draw_warp.h>

class Algorithms{

public:
    virtual void apply() = 0;
};


class Algorithms1 : public Algorithms{
public:
    virtual void apply() override{
        DWCreateShape<Line>(0, 0, 1000, 1000, Colors::BLACK);
        DWCreateShape<Line>(0, 0, 400, 400, Colors::RED);
        DWCreateShape<Line>(0, 0, 100, 400, Colors::GREEN);
        DWCreateShape<Line>(0, 0, 200, 400, Colors::BLUE);
        DWCreateShape<Line>(0, 0, 300, 400, Colors::BLACK);
        // DrawWarp::CreateShape<Line>(0, 0, )

        auto a = DrawWarp::GetInstance().CreateShape<Line>(0, 0, 1000, 1000, Colors::BLACK);
        auto b = DrawWarp::GetInstance().CreateShape<Line>(0, 0, -100, 50, Colors::BLACK);
        DrawWarp::GetInstance().addShape(b);
        auto [_, p] = a->findIntersection(*b);
        std::cout << p.getPoint().x << ", " << p.getPoint().y << std::endl;
    }
};


class test : public Algorithms{
public:
    virtual void apply() override{
        auto a = DrawWarp::GetInstance().CreateShape<Line>(0, 0, 1000, 1000, Colors::BLACK);
        auto b = DrawWarp::GetInstance().CreateShape<Line>(0, 0, -100, 50, Colors::RED);
        DrawWarp::GetInstance().addShape(b);
        DrawWarp::GetInstance().addShape(a);
        auto [_, p] = a->findIntersection(*b);
        std::cout << p.getPoint().x << ", " << p.getPoint().y << std::endl;
    }
};
