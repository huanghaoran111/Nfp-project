#pragma once

#include <draw_warp.h>

class Algorithms{
public:
    virtual void apply() = 0;
};

class xdn_test : public Algorithms{
public:
    virtual void apply() override;
};

class TestCases : public Algorithms {
public:
    virtual void apply() override;
};

// ===== Algorithm1:2006 =====
class Algorithms1 : public Algorithms{
public:
    Algorithms1(std::vector<std::shared_ptr<Shape>> polygon_data);
    virtual void apply();
private:
    std::vector<std::shared_ptr<Shape>> polygon_data;
};


// ===== Algorithm2:2024 =====
class Algorithms2 : public Algorithms{
public:
    Algorithms2(std::vector<std::shared_ptr<Shape>> polygon_data);
    virtual void apply();
private:
    std::vector<std::shared_ptr<Shape>> polygon_data;
};
