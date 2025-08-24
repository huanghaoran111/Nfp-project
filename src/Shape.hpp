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

/**
 * @class ShapeID
 * @brief 用于生成形状唯一ID的类，采用单例模式
 */
class ShapeID {
private:
    static std::mutex mutex;  // 互斥锁，用于保证线程安全
    const int width;            // 数字部分宽度
    std::unordered_map<std::string, int> counterMap; // 用于存储不同前缀的计数器
    // 私有构造函数
    ShapeID();
    // 防止复制和赋值
    ShapeID(const ShapeID&) = delete;
    ShapeID& operator=(const ShapeID&) = delete;
public:
    static ShapeID* getInstance();  // 获取单例实例
    // 生成形状ID
    std::string generate(const std::string& prefix = "SHAPE");
    // 重置计数器
    void reset(unsigned int startValue = 0);
};

// 初始化静态成员变量
std::mutex ShapeID::mutex = std::mutex(); 

/**
 * @class Shape
 * @brief 抽象基类，定义了所有形状的基本接口
 */
class Shape {
protected:
    const std::string SHAPE_NAME;   // 形状名，对不同的类不同
    std::string id;                 // 形状唯一ID
    Color color;                    // 形状颜色
public:
    Shape(const std::string& shape_name = "Shape");
    virtual ~Shape() {} // 虚析构函数，确保派生类对象能正确析构
    std::string getId() const;  // 获取形状ID
    std::string getType() const; // 获取形状类型
    virtual void draw() const = 0; // 纯虚函数，绘制形状，由派生类实现
    // 静态成员初始化
    ShapeID* Shape::idGenerator = ShapeID::getInstance();
    void setColor(const Color& color);
};

class Point : public Shape {
public:
    Point();
    Point(float x, float y);
    Point(const Vec2 p_);
    void draw() const override;
    Vec2 getPoint() const;
private:
    Vec2 p;
};

class Line : public Shape {
public:
    Line();
    explicit Line(Vec2 x, Vec2 y);
    explicit Line(Point p1, Point p2);
    explicit Line(float x1, float x2, float y1, float y2);
    void draw() const override;

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
};

class Polygon : public Shape{
public:
    Polygon();
    Polygon(const Polygon& p);
    Polygon(const std::vector<Vec2>& Points);
    Polygon(const std::vector<Line>& Lines);
    void draw() const override;
    const std::vector<Line>& getLines() const;

    Polygon operator=(const std::vector<Line>& Lines);
    Polygon operator=(const Polygon& Lines);
private:
    std::vector<Line> Lines;
};

class ConvexityPolygon : public Shape {
    enum class LineType
    {
        RAW,
        GENERATED,
    };
    ConvexityPolygon(const std::vector<Line>& Lines_);
    void draw() const override;
    const std::vector<Line>& getLines() const;
    std::pair<Line, Line> getStartAndEndLines() const;

private:
    Polygon raw_polygon;
    std::vector<Line> Lines;
    std::vector<LineType> LinesType;
    Line startLine, endLine;
};