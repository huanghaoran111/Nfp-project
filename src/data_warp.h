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
    }
};

