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
class GridNFPAlgorithm : public Algorithms{
public:
    GridNFPAlgorithm(std::vector<std::shared_ptr<Point>> polygon_data);
    int step1();
    void step2();
    virtual void apply();
private:
    std::vector<std::shared_ptr<Point>> polygon_data;
};


// ===== Algorithm2:2024 =====
class LocalContourNFPAlgorithm : public Algorithms{
public:
    LocalContourNFPAlgorithm(std::vector<std::shared_ptr<Point>> polygon_data);
    void step1();
    void step2();
    virtual void apply();
private:
    std::vector<std::shared_ptr<Point>> polygon_data;
};

// ===== Algorithm3:TwoLocalContour 2024o =====
class TwoLocalContourNFPAlgorithm : public Algorithms {
public:
    TwoLocalContourNFPAlgorithm(std::vector<std::shared_ptr<Point>> polygon_data);
    void step1();
    void step2();
    virtual void apply();
private:
    std::vector<std::shared_ptr<Point>> polygon_data;
};

// ===== Algorithm4: DelaunayTriangulationNFP =====
class DelaunayTriangulationNFPAlgorithm : public Algorithms {
public:
    DelaunayTriangulationNFPAlgorithm(std::vector<std::vector<std::shared_ptr<Point>>> polygon_data);

    virtual void apply();
private:
    std::vector<std::vector<std::shared_ptr<Point>>> polygon_data;
};