#include <Shape.hpp>
#include <imgui.h>

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

void Point::draw(ImDrawList* draw_list, std::function<ImVec2(Vec2)>& trans) const{
    std::cout << "Drawing a point" << std::endl;
}

Vec2 Point::getPoint() const{
    return p;
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

void Line::draw(ImDrawList* draw_list, std::function<ImVec2(Vec2)>& trans) const{
    std::cout << "Drawing a line " << trans(p[0]).x << ", " << trans(p[0]).y << "->" << trans(p[1]).x << ", " << trans(p[1]).y << "   " << Color::toImU32(getColor()) << std::endl;

    // draw_list->AddLine(trans(p[0]), trans(p[1]), Color::toImU32(getColor()), 20);
    draw_list->AddLine(
        trans(p[0]),
        trans(p[1]),
        // IM_COL32(255, 0, 0, 255)
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
    return std::make_pair(false, Point());
}

Polygon::Polygon() : Shape("Polygon"){}


Polygon::Polygon(const std::vector<std::shared_ptr<Point>>& Points): Shape("Polygon") {
    for(int i = 1; i < Points.size(); i++){
        Lines.push_back(std::make_shared<Line>(*(Points[i-1]), *(Points[i])));
    }
    Lines.push_back(std::make_shared<Line>(*(Points[Points.size()-1]), *(Points[0])));
}

Polygon::Polygon(const std::vector<std::shared_ptr<Line>>& Lines_) : Shape("Polygon") {
    for (auto line : Lines_) {
        Lines.push_back(line);
    }
}

void Polygon::draw(ImDrawList* draw_list, std::function<ImVec2(Vec2)>& trans) const{
    std::cout << "Drawing a polygon" << std::endl;
}

Polygon::Polygon(const Polygon& p): Shape("Polygon") {
    for (auto line : p.Lines) {
        Lines.push_back(line);
    }
}

Polygon Polygon::operator=(const std::vector<std::shared_ptr<Line>>& Lines){
    return Polygon(Lines);
}

Polygon Polygon::operator=(const Polygon& Lines){
    return Polygon(Lines);
}

ConvexityPolygon::ConvexityPolygon(const std::vector<std::shared_ptr<Line>>& Lines_) 
    : Shape("ConvexityPolygon")
    , raw_polygon{std::make_shared<Polygon>(Lines_)}
{
    // TODO: 这里写入凸化过程，并将线的属性存入LinesType

}

std::pair<Line, Line> ConvexityPolygon::getStartAndEndLines() const{
    return std::make_pair(*startLine, *endLine);
}

void ConvexityPolygon::draw(ImDrawList* draw_list, std::function<ImVec2(Vec2)>& trans) const{
    std::cout << "Drawing a convexity polygon" << std::endl;
}