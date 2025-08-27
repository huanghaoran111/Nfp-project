#include <Shape.hpp>
#include <imgui.h>

CollectionMap& CollectionMap::getInstance(){
    static CollectionMap instance;
    return instance;
}

void CollectionMap::addShape(std::shared_ptr<Shape> shape){
    CollectionMap::getInstance().shapeMap.insert({shape->getId(), shape});
}

ShapeID::ShapeID()
    :width(std::to_string(std::numeric_limits<unsigned int>::max()).length()) {}

ShapeID& ShapeID::getInstance(){
    static ShapeID instance;
    return instance;
}

std::string ShapeID::generate(const std::string& prefix) {
    std::ostringstream oss;
    auto& count = counterMap[prefix];
    oss << prefix 
        << "-" 
        << std::setw(this->width) 
        << std::setfill('0') 
        << count++;
    return oss.str();
}

void ShapeID::reset(unsigned int startValue) {
    counterMap.clear();
}

Shape::Shape(const std::string& shape_name, Color color_) : SHAPE_NAME{shape_name}, color(color_) {
    id = ShapeID::getInstance().generate(SHAPE_NAME);
}

std::string Shape::getType() const {
    std::string pattern = "^([^0-9]*)(\\d{" + std::to_string(std::numeric_limits<unsigned int>::max()) + "})$";
    std::regex sepRegex(pattern);
    std::smatch matches;
    if (std::regex_match(id, matches, sepRegex)){
        return matches[1];
    }
    return "Unknown";
}

std::string Shape::getId() const { return id; }

Color Shape::getColor() const{
    return color;
}

Point::Point() : Shape("Point") {
    p.x = 0;
    p.y = 0;
}

Point::Point(float x, float y) : Shape("Point") {
    p.x = x;
    p.y = y;
}
Point::Point(const Vec2 p_) : Shape("Point") {
    this->p = p;
}

Point::Point(const Point& p_) : Shape("Point") {
    this->p = p_.getPoint();
    for(const auto& str : p_.getComeFrom()){
        this->come_from.push_back(str);
    }
}

void Point::draw(ImDrawList* draw_list, std::function<ImVec2(Vec2)>& trans) const{
    // std::cout << "Drawing a point" << std::endl;
}

Vec2 Point::getPoint() const{
    return p;
}

void Point::setComeFrom(const std::string& str){
    this->come_from.push_back(str);
}

const std::vector<std::string>& Point::getComeFrom() const {
    return this->come_from;
}

Line::Line() : Shape("Line"){
    p[0] = Vec2(0,0);
    p[1] = Vec2(0,0);
}

Line::Line(Point p1, Point p2) : Shape("Line"){
    p[0] = p1.getPoint();
    p[1] = p2.getPoint();
}

Line::Line(Vec2 x, Vec2 y) : Shape("Line"){
    p[0] = x;
    p[1] = y;
}

Line::Line(float x1, float x2, float y1, float y2) : Shape("Line") {
    p[0].x = x1;
    p[0].y = x2;
    p[1].x = y1;
    p[1].y = y2;
}

Line::Line(Vec2 x, Vec2 y, uint32_t color) : Shape("Line", color){
    p[0] = x;
    p[1] = y;
}
Line::Line(Point p1, Point p2, uint32_t color) : Shape("Line", color){
    p[0] = p1.getPoint();
    p[1] = p2.getPoint();
}

Line::Line(float x1, float x2, float y1, float y2, uint32_t color): Shape("Line", color){
    p[0].x = x1;
    p[0].y = x2;
    p[1].x = y1;
    p[1].y = y2;
}

void Line::setComeFrom(const std::string& str){
    this->come_from = str;
}

const char* Line::getComeFrom(){
    return this->come_from.c_str();
}

void Line::draw(ImDrawList* draw_list, std::function<ImVec2(Vec2)>& trans) const{
    // std::cout << "Drawing a line " << this->getId() << std::endl;
    draw_list->AddLine(
        trans(p[0]),
        trans(p[1]),
        Color::toImU32(getColor())
    );
}

Vec2 Line::getStartPoint() const{
    return p[0];
}

Vec2 Line::getEndPoint() const{
    return p[1];
}

float Line::getLength() const{
    return sqrt(p[0] * p[1]);
}

float Line::getXangle() const{
    return atan2(p[1].y - p[0].y, p[1].x - p[0].x);
}

int Line::whereIsPointOnLine(const Vec2 p) const{
    // 这里写入判断点与直线位置关系的代码
    
    Vec2 lineVec = this->p[1] - this->p[0];  // 直线向量
    Vec2 pointVec = p - this->p[0];   // 从直线起点到点的向量
    
    // 计算叉积
    double crossProduct = lineVec.x * pointVec.y - lineVec.y * pointVec.x;
    
    if (abs(crossProduct) < EPSILON) {
        return 0;  // 点在直线上
    } else if (crossProduct > 0) {
        return 1;  // 点在直线左侧
    } else {
        return -1; // 点在直线右侧
    }
    return 0;
}

int Line::whereIsPointOnLine(const Point p) const{
    return whereIsPointOnLine(p.getPoint());
}

bool Line::arePointsOnSameSide(const Point p1, const Point p2) const{
    return whereIsPointOnLine(p1.getPoint()) * whereIsPointOnLine(p2.getPoint()) >= 0;
}

bool Line::arePointsOnSameSide(const Vec2 p1, const Point p2) const{
    return whereIsPointOnLine(p[0]) * whereIsPointOnLine(p[1]) >= 0;
}

std::pair<bool, Point> Line::findIntersection(const Line& line) const{
    // TODO: 这里写入求交点的代码
    // Point的come_from是一个列表
    // 注意 返回的Point需要指明come_from 即调用setComeFrom 两条线都要setComeFrom
    return std::make_pair(false, Point());
}

Polygon::Polygon() : Shape("Polygon"){}


Polygon::Polygon(const std::vector<std::shared_ptr<Point>>& Points): Shape("Polygon") {
    for(int i = 1; i < Points.size(); i++){
        Lines.push_back(std::make_shared<Line>(*(Points[i-1]), *(Points[i])));
        Lines.back()->setComeFrom(this->getId());
        this->Points.push_back(std::make_shared<Point>(Points[i]->getPoint()));
        this->Points.back()->setComeFrom(this->getId());
    }
    Lines.push_back(std::make_shared<Line>(*(Points[Points.size()-1]), *(Points[0])));
    Lines.back()->setComeFrom(this->getId());
    this->Points.push_back(std::make_shared<Point>(Points[0]->getPoint()));
    this->Points.back()->setComeFrom(this->getId());
}

Polygon::Polygon(const std::vector<std::shared_ptr<Line>>& Lines_) : Shape("Polygon") {
    for (auto line : Lines_) {
        Lines.push_back(line);
        if(!Points.empty() && Points.back()->getPoint() == line->getStartPoint()){
            Points.push_back(std::make_shared<Point>(line->getEndPoint()));
            Points.back()->setComeFrom(this->getId());
        }else if(Points.back()->getPoint() == line->getStartPoint()){
            Points.push_back(std::make_shared<Point>(line->getStartPoint()));
            Points.back()->setComeFrom(this->getId());
            Points.push_back(std::make_shared<Point>(line->getEndPoint()));
            Points.back()->setComeFrom(this->getId());
        }else{
            throw std::runtime_error("Lines are not connected");
        }
    }
}

void Polygon::draw(ImDrawList* draw_list, std::function<ImVec2(Vec2)>& trans) const{
    for (auto line : Lines){
        line->draw(draw_list, trans);
    }
}

Polygon Polygon::operator=(const std::vector<std::shared_ptr<Line>>& Lines){
    return Polygon(Lines);
}

ConvexityPolygon::ConvexityPolygon(const std::vector<std::shared_ptr<Line>>& Lines_) 
    : Shape("ConvexityPolygon")
    , raw_polygon{std::make_shared<Polygon>(Lines_)}
{
    // TODO: 这里写入凸化过程，并将线的属性存入LinesType
    // 创建线需要将come_from属性设置为 this->getId() 这样在处理线的时候可以知道该线属于哪个多边形
    // 替换边的属性由LinesType指明 LineType::RAW 表示原始边 LineType::CONVEX 表示凸化后的边
    // 创建线可使用 DrawWarp::CreateShape<Line>() 不要使用DWCreateShape，因为它会直接将Line放入渲染队列中
    // 如果有其他的要求 在数据结构里面添加数据并在这里实现即可

}

std::pair<Line, Line> ConvexityPolygon::getStartAndEndLines() const{
    return std::make_pair(*startLine, *endLine);
}

void ConvexityPolygon::draw(ImDrawList* draw_list, std::function<ImVec2(Vec2)>& trans) const{
    for (auto line : Lines){
        line->draw(draw_list, trans);
    }
}