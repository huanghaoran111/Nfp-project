/**
 * @file Shape.h
 * @brief 定义了形状相关的类和接口
 */
#pragma once
#include <math.h>
#include <iostream>
#include <string>
#include <memory>
#include <regex>
#include <mutex>
#include <unordered_map>
#include <iomanip>
#include <sstream>

#include <MathCore.h>
#include <Color.hpp>

class Shape;
class ShapeID;
class Point;
class Line;
class Polygon;
class ConvexityPolygon;

struct CollectionMap{
    static CollectionMap& getInstance();
    static void addShape(std::shared_ptr<Shape> shape);
    std::unordered_map<std::string, std::shared_ptr<Shape>> shapeMap;
};

/**
 * @class ShapeID
 * @brief 用于生成形状唯一ID的类，采用单例模式
 */
class ShapeID {
private:
    const int width;            // 数字部分宽度
    std::unordered_map<std::string, int> counterMap; // 用于存储不同前缀的计数器
    // 私有构造函数
    ShapeID();
    // 防止复制和赋值
    ShapeID(const ShapeID&) = delete;
    ShapeID& operator=(const ShapeID&) = delete;
public:
    static ShapeID& getInstance();  // 获取单例实例
    // 生成形状ID
    std::string generate(const std::string& prefix = "SHAPE");
    // 重置计数器
    void reset(unsigned int startValue = 0);
};

/**
 * @class Shape
 * @brief 抽象基类，定义了所有形状的基本接口
 */
class Shape {
protected:
    const std::string SHAPE_NAME;   // 形状名，对不同的类不同
    std::string id;                 // 形状唯一ID
public:
    Shape(const std::string& shape_name = "Shape", Color color_ = 0x000000ff);
    virtual ~Shape() {} // 虚析构函数，确保派生类对象能正确析构
    std::string getId() const;  // 获取形状ID
    std::string getType() const; // 获取形状类型
    virtual void draw(ImDrawList* draw_list, std::function<ImVec2(Vec2)>&) const = 0; // 纯虚函数，绘制形状，由派生类实现
    Color color;                    // 形状颜色
    Color getColor() const;
};

class Point : public Shape {
public:
    Point();
    Point(float x, float y);
    Point(const Vec2 p_);
    Point(const Point& p_);
    void setComeFrom(const std::string& str);
    const std::vector<std::string>& getComeFrom() const;
    void draw(ImDrawList* draw_list, std::function<ImVec2(Vec2)>&) const override;
    Vec2 getPoint() const;
private:
    Vec2 p;
    std::vector<std::string> come_from; // 用于记录点的来源
};

class Line : public Shape {
public:
    Line();
    explicit Line(Vec2 x, Vec2 y);
    explicit Line(Point p1, Point p2);
    explicit Line(float xx, float xy, float yx, float yy);
    explicit Line(Vec2 x, Vec2 y, uint32_t color);
    explicit Line(Point p1, Point p2, uint32_t color);
    explicit Line(float xx, float xy, float yx, float yy, uint32_t color);
    void draw(ImDrawList* draw_list, std::function<ImVec2(Vec2)>&) const override;
    void setComeFrom(const std::string& str);
    const char* getComeFrom();
    /**
     * @brief 获取线段的起点坐标
     * @return 返回一个Vec2类型的坐标点，表示线段的起点
     */
    Vec2 getStartPoint() const;

    /**
     * @brief 获取线段的终点坐标
     * @return 返回一个Vec2类型的坐标点，表示线段的终点
     */
    Vec2 getEndPoint() const;

    /**
     * @brief 获取线段的长度
     * @return 返回一个浮点数，表示线段的长度
     */
    float getLength() const;
    
    /**
     * @brief 获取线段与X轴的夹角
     * @return 返回一个浮点数，表示线段与X轴的夹角（弧度制）
     */
    float getXangle() const;

    int whereIsPointOnLine(const Vec2 p) const;
    int whereIsPointOnLine(const Point p) const;

    bool arePointsOnSameSide(const Point p1, const Point p2) const;
    bool arePointsOnSameSide(const Vec2 p1, const Point p2) const;

    std::pair<bool, Point> findIntersection(const Line& l) const;

private:
    Vec2 p[2];
    std::string come_from = "None"; // 用于记录线的来源
};

class Polygon : public Shape{
public:
    Polygon();
    Polygon(const Polygon& p) = delete;
    Polygon(const std::vector<std::shared_ptr<Point>>& Points);
    Polygon(const std::vector<std::shared_ptr<Line>>& Lines);
    void draw(ImDrawList* draw_list, std::function<ImVec2(Vec2)>&) const override;
    const std::vector<Line>& getLines() const;

    Polygon operator=(const std::vector<std::shared_ptr<Line>>& Lines);
    Polygon operator=(const Polygon& Lines) = delete;
    Polygon operator=(Polygon&& Lines);
private:
    std::vector<std::shared_ptr<Line>> Lines;
    std::vector<std::shared_ptr<Point>> Points;
};

class ConvexityPolygon : public Shape {
public:
    enum class LineType
    {
        RAW,
        CONVEX,
    };
    ConvexityPolygon(const std::vector<std::shared_ptr<Line>>& Lines_);
    void draw(ImDrawList* draw_list, std::function<ImVec2(Vec2)>&) const override;
    const std::vector<Line>& getLines() const;
    std::pair<Line, Line> getStartAndEndLines() const;

private:
    std::shared_ptr<Shape> raw_polygon;
    std::vector<std::shared_ptr<Line>> Lines;
    std::vector<LineType> LinesType;
    std::shared_ptr<Line> startLine, endLine;
};