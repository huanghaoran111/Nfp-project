#pragma once

#include <draw_warp.h>


namespace NFP{
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

class MovingCollisionNFPAlgorithm : public Algorithms {
public:
    MovingCollisionNFPAlgorithm(std::vector<std::vector<std::shared_ptr<Point>>> polygon_data);
    
    virtual void apply();
private:
    std::vector<std::vector<std::shared_ptr<NFP::Point>>> polygon_data;
};

// ===== Algorithm1:2006 =====
class TrajectoryNFPAlgorithm : public Algorithms{
public:
    TrajectoryNFPAlgorithm(std::vector<std::vector<std::shared_ptr<Point>>> polygon_data);
    
    std::vector<std::shared_ptr<Line>> GenerateTrajectoryLinesSet(std::shared_ptr<Polygon> polygonA, std::shared_ptr<Polygon> polygonB);
    virtual void apply();
private:
    std::vector<std::vector<std::shared_ptr<Point>>> polygon_data;
};


// ===== Algorithm2:2024 =====
class LocalContourNFPAlgorithm : public Algorithms{
public:
    LocalContourNFPAlgorithm(std::vector<std::vector<std::shared_ptr<Point>>> polygon_data);
    void step1();
    void step2();
    virtual void apply();
private:
    std::vector<std::vector<std::shared_ptr<Point>>> polygon_data;
};

// ===== Algorithm3:TwoLocalContour 2024o =====
class TwoLocalContourNFPAlgorithm : public Algorithms {
public:
    TwoLocalContourNFPAlgorithm(std::vector<std::vector<std::shared_ptr<Point>>> polygon_data);
    void step1();
    void step2();
    virtual void apply();
private:
    std::vector<std::vector<std::shared_ptr<Point>>> polygon_data;
};

class MinkowskiSumNFPAlgorithm : public Algorithms {
public:
    MinkowskiSumNFPAlgorithm(std::vector<std::vector<std::shared_ptr<Point>>> polygon_data);
    virtual void apply();
private:
    std::vector<std::vector<std::shared_ptr<Point>>> polygon_data;
};

// ===== Algorithm4: DelaunayTriangulationNFP =====
class DelaunayTriangulationNFPAlgorithm : public Algorithms {
public:
    
    DelaunayTriangulationNFPAlgorithm(std::vector<std::vector<std::shared_ptr<Point>>> polygon_data);

    virtual void apply();
private:
    std::vector<std::vector<std::shared_ptr<Point>>> polygon_data;
};
}