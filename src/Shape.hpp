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

class ShapeID {
private:
    static std::mutex mutex;
    const int width;            // 数字部分宽度
    std::unordered_map<std::string, int> counterMap; // 用于存储不同前缀的计数器
    // 私有构造函数
    ShapeID();
    // 防止复制和赋值
    ShapeID(const ShapeID&) = delete;
    ShapeID& operator=(const ShapeID&) = delete;
public:
    static ShapeID* getInstance();
    // 生成形状ID
    std::string generate(const std::string& prefix = "SHAPE");
    // 重置计数器
    void reset(unsigned int startValue = 0);
};

std::mutex ShapeID::mutex = std::mutex(); 

class Shape {
protected:
    const std::string SHAPE_NAME;   // 形状名，对不同的类不同
    std::string id;                     // 形状唯一ID
    static ShapeID* idGenerator;        // ID生成器
public:
    Shape(const std::string& shape_name = "Shape");
    virtual ~Shape() {} // 虚析构函数
    std::string getId() const;
    std::string getType() const;
    virtual void draw() const = 0;
};

ShapeID* Shape::idGenerator = ShapeID::getInstance();

class Line : public Shape {
    
public:
    Line();
    void draw() const override;
    
private:
    
};