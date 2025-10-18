#include <Shape.hpp>
#include <imgui.h>
#include <draw_warp.h>

#include <map>
#include <set>


using Polygon = NFP::Polygon;

CollectionMap& CollectionMap::getInstance(){
    static CollectionMap instance;
    return instance;
}

void CollectionMap::addShape(std::shared_ptr<NFP::Shape> shape){
    CollectionMap::getInstance().shapeMap.insert({shape->getId(), shape});
}
namespace NFP{
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

Shape::Shape(const char* shape_name, Color color_) : SHAPE_NAME{shape_name}, color(color_) {
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
    come_from = {};
}

Point::Point(float x, float y) : Shape("Point") {
    p.x = x;
    p.y = y;
    come_from = {};
}
Point::Point(const Vec2 p_) : Shape("Point") {
    this->p = p_;
    come_from = {};
}

void Point::draw(ImDrawList* draw_list, std::function<ImVec2(Vec2)>& trans) const{
    // std::cout << "Drawing a point" << std::endl;
    static bool is_reread = true;
    static bool show_point_tag;
    static bool show_point_pos;
    if(EventActivator::GetInstance().HasEvent("ShowPointTag")){
        EventActivator::GetInstance().ActivateEvent("ShowPointTag", &show_point_tag);
    }
    if(EventActivator::GetInstance().HasEvent("ShowPointPos")){
        EventActivator::GetInstance().ActivateEvent("ShowPointPos", &show_point_pos);
    }
    std::ostringstream oss;
    if(show_point_tag){
        oss << "p" << std::to_string(this->idx);
    }
    if(show_point_pos){
        oss << "(" << std::setprecision(3) << p.x << ", " << p.y << ")";
    }
    draw_list->AddCircleFilled(trans(Vec2(p.x, p.y)), 1, Colors::BLACK);
    draw_list->AddText(
        nullptr, 18, trans(Vec2(p.x, p.y)), IM_COL32(0, 0, 0, 255), oss.str().c_str()
    );
}

int Point::getIdx()const{
    return this->idx;
}

void Point::setIdx(int id){
    this->idx = id;
}

Vec2 Point::getPoint() const{
    return p;
}

void Point::setComeFrom(std::string str){
    this->come_from = str;
}

std::string Point::getComeFrom() const {
    return this->come_from;
}

void Point::Move(float x, float y){
    p.x += x;
    p.y += y;
}

void Point::MoveTo(float x, float y){
    p.x = x;
    p.y = y;
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
        Colors::BLACK,
        3.f
    );
    
}

Vec2 Line::getStartPoint() const{
    return p[0];
}

Vec2 Line::getEndPoint() const{
    return p[1];
}

float Line::getLength() const{
    return sqrt((p[0].x - p[1].x) * (p[0].x - p[1].x) + (p[0].y - p[1].y) * (p[0].y - p[1].y));
}

float Line::getXangle() const{
    return atan2(p[1].y - p[0].y, p[1].x - p[0].x);
}

void Line::Move(float x, float y){
    p[0].x += x;
    p[0].y += y;
    p[1].x += x;
    p[1].y += y;
}

void Line::MoveTo(int idx, float x, float y){
    assert(idx == 0 || idx == 1);
    auto a = (idx + 1) % 2;
    auto b = idx % 2;
    Vec2 vec = p[a] - p[b];
    p[b].x = x;
    p[b].y = y;
    p[a].x = x + vec.x;
    p[a].y = y + vec.y;
}

Line& Line::operator=(const Line& l){
    if(this == &l) return *this;
    Shape::operator=(l);
    this->p[0] = l.getStartPoint();
    this->p[1] = l.getEndPoint();
    this->come_from = l.come_from;
    return *this;
}

int Line::whereIsPointOnLine(const Vec2 p) const{
    // 这里写入判断点与直线位置关系的代码
    
    Vec2 lineVec = this->p[1] - this->p[0];  // 直线向量
    Vec2 pointVec = p - this->p[0];   // 从直线起点到点的向量
    
    // 计算叉积
    float crossProduct = lineVec.x * pointVec.y - lineVec.y * pointVec.x;
    
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

int Line::whereIsPointOnLineSegment(const Vec2 p) const{
    // 这里写入判断点与线段位置关系的代码
    if (whereIsPointOnLine(p) == 0) {
        Vec2 pVec1 = p - this->p[0];
        Vec2 pVec2 = p - this->p[1];
        if (pVec1 * pVec2 <= 0) {
            return 0;  // 点在线段上
        }
        else{
            return -2;
        }
    }
    else
    {
        return whereIsPointOnLine(p);
    }
}

int Line::whereIsPointOnLineSegment(const Point p) const {
    return whereIsPointOnLineSegment(p.getPoint());
}

bool Line::arePointsOnSameSide(const Point p1, const Point p2) const{
    return whereIsPointOnLine(p1.getPoint()) * whereIsPointOnLine(p2.getPoint()) >= 0;
}

bool Line::arePointsOnSameSide(const Vec2 p1, const Point p2) const{
    return whereIsPointOnLine(p[0]) * whereIsPointOnLine(p[1]) >= 0;
}

std::pair<Line::LineRelationship, std::shared_ptr<Point>> 
Line::findIntersection(const Line& line) const{
    using LineRelationship = Line::LineRelationship;
    // TODO: 这里写入求交点的代码
    // Point的come_from是一个列表
    // 注意 返回的Point需要指明come_from 即调用setComeFrom 两条线都要setComeFrom
    // 部分重叠的线段不在这里做判断（在“查找右侧夹角最小的线段”中做预处理）
    // 部分重叠：q1 在 p 内部 + q2 在 p 的右边延长线 → t0 ∈[0, 1]，t1 > 1
    
    // 获取两条线段的端点
    Vec2 p1 = this->p[0], p2 = this->p[1]; // 当前线段的两个端点
    Vec2 q1 = line.p[0], q2 = line.p[1];   // 参数线段的两个端点

    // 方向向量
    Vec2 r = p2 - p1;
    Vec2 s = q2 - q1;

    // 向量叉乘
    float rxs = r.Cross(s);
    Vec2 pq = q1 - p1;

    // r × s = 0 → 平行或共线 
    if (std::fabs(rxs) < EPSILON) {
        // r × s = 0 && pq × r = 0 → 共线
        std::cout << std::fabs(pq.Cross(r)) << std::endl;
        if (std::fabs(pq.Cross(r)) < EPSILON) {
            // 检查重叠情况
            float t0 = (pq * r) / (r * r);
            float t1 = (((q1 + s) - p1) * r) / (r * r);
            if (t0 > t1) std::swap(t0, t1);

            // [t0, t1] 与 [0,1] 区间有交集 → 有重叠部分
            // 因为是找右侧夹角最小的线段，因此：
            // 要的情况: q 在 p 内部 + 右边延长线 → t0 ∈[0, 1]，t1 > 1
            // 不要的情况（q 在 p 内部 + 左边延长线） → t0 < 0，t1 ∈[0, 1]
            if (t0 >= -EPSILON && t0 <= 1 + EPSILON && t1 > 1 + EPSILON) {
                // 部分重叠不返回交点
                return std::pair(LineRelationship::PARTOVERLAP, nullptr);
            }
        }
        // else: 平行但不共线 → 没有交点
    }
    else {
        // r × s ≠ 0 && 0 ≤ t ≤ 1 && 0 ≤ u ≤ 1 → 相交
        // 计算交点参数
        double t = pq.Cross(s) / rxs;
        double u = pq.Cross(r) / rxs;

        if (t >= -EPSILON && t <= 1 + EPSILON &&
            u >= -EPSILON && u <= 1 + EPSILON) {
            Vec2 intersection = p1 + t * r; // or：q1 + u * s
            if (std::abs(r.x) > std::abs(r.y)) {
                intersection = p1 + r * t;
            }
            else {
                intersection = q1 + s * u;
            }
            auto pt = DrawWarp::GetInstance().CreateShape<Point>(intersection.x, intersection.y);  // 用交点坐标构造一个 Point
            pt->setComeFrom(this->getId());    // 标记来源：本线段
            pt->setComeFrom(line.getId());      // 标记来源：另一条线段
            return std::pair(LineRelationship::INTERSECT, pt);
        }
    }
    // 没交点
    return std::pair(LineRelationship::NOTINTERSECT, nullptr);
}

Polygon::Polygon() : Shape("Polygon"){}

static void polygon_create_helper(
    const std::vector<std::shared_ptr<Point>>& Points, 
    std::vector<std::shared_ptr<Point>>& ps, 
    std::vector<std::shared_ptr<Line>>& ls,
    std::string shapeid
) {
    // 最左下角的点为起点
    std::vector<int> y_min_p = { 0 };
    for (int i = 1; i < Points.size(); i++) {
        if (Points[i]->getPoint().y < Points[y_min_p[0]]->getPoint().y) {
            y_min_p.clear();
            y_min_p.push_back(i);
        }
        else if (Points[i]->getPoint().y == Points[y_min_p[0]]->getPoint().y) {
            y_min_p.push_back(i);
        }
    }
    auto x_min_p = y_min_p[0];
    if (y_min_p.size() > 1) {
        for (int i = 1; i < y_min_p.size(); i++) {
            if (Points[y_min_p[i]]->getPoint().x < Points[x_min_p]->getPoint().x) {
                x_min_p = y_min_p[i];
            }
        }
    }
    //return; x_min_p;
    // 按顺序创建边和点
    auto n = Points.size();
    for (int i = (x_min_p + 1) % n; i != x_min_p; i = (i + 1) % n) {
        if (i != 0) {
            ls.push_back(std::make_shared<Line>(*(Points[i - 1]), *(Points[i])));
            ls.back()->setComeFrom(shapeid + std::to_string(ls.size() - 1));
            ps.push_back(std::make_shared<Point>(Points[i - 1]->getPoint()));
            ps.back()->setComeFrom(shapeid + std::to_string(ps.size() - 1));
            ps.back()->setIdx(ps.size() - 1);
        }
        else {
            ls.push_back(std::make_shared<Line>(*(Points[Points.size() - 1]), *(Points[0])));
            ls.back()->setComeFrom(shapeid + std::to_string(ls.size() - 1));
            ps.push_back(std::make_shared<Point>(Points[Points.size() - 1]->getPoint()));
            ps.back()->setComeFrom(shapeid + std::to_string(ps.size() - 1));
            ps.back()->setIdx(ps.size() - 1);
        }
    }
    ls.push_back(std::make_shared<Line>(*(Points[(x_min_p - 1 + Points.size()) % Points.size()]), *(Points[x_min_p])));
    ls.back()->setComeFrom(shapeid + std::to_string(ls.size() - 1));
    ps.push_back(std::make_shared<Point>(Points[(x_min_p - 1 + Points.size()) % Points.size()]->getPoint()));
    ps.back()->setComeFrom(shapeid + std::to_string(ps.size() - 1));
    ps.back()->setIdx(ps.size() - 1);
}

Polygon::Polygon(const std::vector<std::shared_ptr<Point>>& Points): Shape("Polygon"), Lines(), Points() {
    polygon_create_helper(Points, this->Points, this->Lines, this->getId());
}

Polygon::Polygon(const std::vector<std::shared_ptr<Point>>& Points, std::string id) : Shape("Polygon"), Lines(), Points() {
    polygon_create_helper(Points, this->Points, this->Lines, id);
}

Polygon::Polygon(const std::vector<std::shared_ptr<Line>>& Lines, std::string id) : Shape("Polygon"), Lines(), Points() {
    std::vector<std::shared_ptr<Point>> p;
    if (Lines.size() == 0) {
        throw std::runtime_error("Lines number is zero");
    }
    p.push_back(std::make_shared<Point>(Lines[0]->getStartPoint()));
    p.push_back(std::make_shared<Point>(Lines[0]->getEndPoint()));
    // 输入的最后一根线的终点应该是所有点的起点
    for (int i = 1; i < Lines.size(); i++) {
        if (p.back()->getPoint() == Lines[i]->getStartPoint()) {
            p.push_back(std::make_shared<Point>(Lines[i]->getEndPoint()));
        }
        else {
            throw std::runtime_error("Lines are not connected");
        }
    }
    if (p.back()->getPoint() != p.front()->getPoint()) {
        throw std::runtime_error("Lines are not connected in start and end");
    }
    else
    {
        p.pop_back();
    }
    polygon_create_helper(p, this->Points, this->Lines, id);
}

Polygon::Polygon(const std::vector<std::shared_ptr<Line>>& Lines_) : Shape("Polygon") {
    std::vector<std::shared_ptr<Point>> p;
    if (Lines_.size() == 0) {
        throw std::runtime_error("Lines number is zero");
    }
    p.push_back(std::make_shared<Point>(Lines_[0]->getStartPoint()));
    p.push_back(std::make_shared<Point>(Lines_[0]->getEndPoint()));
    // 输入的最后一根线的终点应该是所有点的起点
    for (int i = 1; i < Lines_.size(); i++) {
        if(p.back()->getPoint() == Lines_[i]->getStartPoint()){
            p.push_back(std::make_shared<Point>(Lines_[i]->getEndPoint()));
        }else{
            throw std::runtime_error("Lines are not connected");
        }
    }
    if (p.back()->getPoint() != p.front()->getPoint()) {
        throw std::runtime_error("Lines are not connected in start and end");
    }
    else
    {
        p.pop_back();
    }
    polygon_create_helper(p, this->Points, this->Lines, this->getId());
}

const std::vector<std::shared_ptr<Line>>& Polygon::getLines() const {
    return this->Lines;
}

const std::vector<std::shared_ptr<Point>>& Polygon::getPoints() const {
    return this->Points;
}

std::vector<std::shared_ptr<Point>> Polygon::reversePoints() const {
    std::vector<std::shared_ptr<Point>> res;
    res.push_back(this->Points[0]);
    for (int i = this->Points.size() - 1; i > 0; i--) {
        res.push_back(this->Points[i]);
    }
    return res;
}

std::vector<std::shared_ptr<Line>> Polygon::sortLineByAngle() const {
    std::vector<std::shared_ptr<Line>> res = this->Lines;
    std::sort(res.begin(), res.end(), [](auto line1, auto line2) {
        return (line1->getEndPoint() - line1->getStartPoint()).angle() < (line2->getEndPoint() - line2->getStartPoint()).angle();
    });
    return res;
}


bool Vec2Key::operator()(const Vec2& a, const Vec2& b) const{
    return a.x < b.x || (a.x == b.x && a.y < b.y);
}

std::map<Vec2, int, Vec2Key> Polygon::getVec2ToIndex() const{
    std::map<Vec2, int, Vec2Key> res;
    for (int i = 0; i < this->Points.size(); i++) {
        res[this->Points[i]->getPoint()] = i;
    }
    return res;
}

int Polygon::GetLowestPointIdx() const {
    std::vector<int> y_min_p = {0};
    for (int i = 1; i < this->Points.size(); i++) {
        if (this->Points[i]->getPoint().y < this->Points[y_min_p[0]]->getPoint().y) {
            y_min_p.clear();
            y_min_p.push_back(i);
        }
        else if (this->Points[i]->getPoint().y == this->Points[y_min_p[0]]->getPoint().y) {
            y_min_p.push_back(i);
        }
    }
    auto x_min_p = y_min_p[0];
    if (y_min_p.size() > 1) {
        for (int i = 1; i < y_min_p.size(); i++) {
            if (this->Points[y_min_p[i]]->getPoint().x < this->Points[x_min_p]->getPoint().x) {
                x_min_p = y_min_p[i];
            }
        }
    }
    return x_min_p;
}

void Polygon::draw(ImDrawList* draw_list, std::function<ImVec2(Vec2)>& trans) const {
    for (auto line : Lines) {
        draw_list->AddLine(
            trans(line->getStartPoint()),
            trans(line->getEndPoint()),
            Colors::BLACK,
            2.f
        );
    }
}

Polygon Polygon::operator=(const std::vector<std::shared_ptr<Line>>& Lines){
    return Polygon(Lines);
}

//static std::vector<int> CheckPointConvexity(std::shared_ptr<Shape> _polygon) {
//    auto polygon = std::static_pointer_cast<Polygon>(_polygon);
//    size_t n = polygon->getLines().size();
//
//    // 默认所有点为凸点（0）
//    std::vector<int> convexity(n, 0);
//
//    // ---------- 计算正方向 ----------
//    int minIdx = polygon->GetLowestPointIdx();
//    size_t prevIdx = (minIdx == 0) ? n - 1 : static_cast<size_t>(minIdx) - 1;
//    size_t nextIdx = (static_cast<size_t>(minIdx) + 1) % n;
//
//    Vec2 edge1 = polygon->getPoints()[minIdx]->getPoint() - polygon->getPoints()[prevIdx]->getPoint();
//    Vec2 edge2 = polygon->getPoints()[nextIdx]->getPoint() - polygon->getPoints()[minIdx]->getPoint();
//    float positiveDirection = edge1.Cross(edge2);
//
//    // ---------- 遍历每个点判断凸/凹 ----------
//    for (size_t i = 0; i < n; ++i) {
//        int prev = (i == 0) ? n - 1 : static_cast<int>(i) - 1;
//        int next = (i + 1) % n;
//
//        Vec2 e1 = polygon->getPoints()[i]->getPoint() - polygon->getPoints()[prev]->getPoint();
//        Vec2 e2 = polygon->getPoints()[next]->getPoint() - polygon->getPoints()[i]->getPoint();
//
//        float crossZ = e1.Cross(e2);
//        if (crossZ * positiveDirection < 0) {
//            convexity[i] = 1; // 凹点
//        }
//        else {
//            convexity[i] = 0; // 凸点或共线
//        }
//    }
//    return convexity;
//}

// 凸包构建
static void convexity_polygon_create_helper(std::vector<std::shared_ptr<Point>>& points, std::vector<std::shared_ptr<Point>>& ConvexityPoints, std::vector<ConvexityPolygon::PointType>& PointsType) {
    auto pivot = points[0];
    std::sort(points.begin(), points.end(),
        [pivot](const std::shared_ptr<Point>& a, const std::shared_ptr<Point>& b) {
            auto vecA = a->getPoint() - pivot->getPoint();
            auto vecB = b->getPoint() - pivot->getPoint();

            double crossVal = vecA ^ vecB;

            if (std::abs(crossVal) > 1e-10) {
                return crossVal > 0; // 逆时针方向
            }

            // 共线时，距离近的排在前面
            return Line(*a, *pivot).getLength() < Line(*b, *pivot).getLength();
        });
    ConvexityPoints.push_back(points[0]);
    ConvexityPoints.push_back(points[1]);

    for (size_t i = 2; i < points.size(); i++) {
        while (ConvexityPoints.size() >= 2) {
            auto a = ConvexityPoints[ConvexityPoints.size() - 2];
            auto b = ConvexityPoints[ConvexityPoints.size() - 1];
            auto c = points[i];

            auto ab = b->getPoint() - a->getPoint();
            auto bc = c->getPoint() - b->getPoint();

            double crossVal = ab ^ bc;

            if (crossVal < -1e-10) { // 顺时针方向，需要移除
                ConvexityPoints.pop_back();
            }
            else {
                break;
            }
        }
        ConvexityPoints.push_back(points[i]);
    }
    PointsType.resize(points.size());
    int hullidx = 0, pidx = 0;
    while (pidx < points.size()) {
        if (points[pidx]->getPoint() != ConvexityPoints[hullidx]->getPoint()) {
            PointsType[pidx] = ConvexityPolygon::PointType::CONCAVE;
        }
        else
        {
            PointsType[pidx] = ConvexityPolygon::PointType::CONVEX;
            hullidx++;
        }
        pidx++;
    }
}

ConvexityPolygon::ConvexityPolygon(const std::vector<std::shared_ptr<Line>>& Lines_) 
    : Shape("ConvexityPolygon")
    , raw_polygon{std::make_shared<Polygon>(Lines_)}
{
    std::vector<std::shared_ptr<Point>> points = std::static_pointer_cast<Polygon>(raw_polygon)->getPoints();
    convexity_polygon_create_helper(points, this->ConvexityPoints, this->PointsType);
}

ConvexityPolygon::ConvexityPolygon(const std::vector<std::shared_ptr<Point>>& Points_)
    : Shape("ConvexityPolygon")
    , raw_polygon{ std::make_shared<Polygon>(Points_)}
{
    std::vector<std::shared_ptr<Point>> points = Points_;
    convexity_polygon_create_helper(points, this->ConvexityPoints, this->PointsType);
}

std::vector<std::shared_ptr<Point>> ConvexityPolygon::getConvexityPoints() const {
    return this->ConvexityPoints;
}

//std::pair<Line, Line> ConvexityPolygon::getStartAndEndLines() const{
//    return std::make_pair(*startLine, *endLine);
//}

void ConvexityPolygon::draw(ImDrawList* draw_list, std::function<ImVec2(Vec2)>& trans) const{
    /*for (auto line : Lines){
        line->draw(draw_list, trans);
    }*/
}

const std::vector<ConvexityPolygon::PointType>& ConvexityPolygon::getPointsType() const {
    return this->PointsType;
}

static int getPointIdxInPolygonPoints(Vec2 p, std::vector<std::shared_ptr<Point>>& points) {
    int n = points.size();
    int idx = -1;
    for (int i = 0; i < n; i++) {
        if (points[i]->getPoint() == p) {
            idx = i;
            break;
        }
    }
    return idx;
}

struct Vec2Compare{
    bool operator()(const Vec2& a, const Vec2& b) const{
        return a.x < b.x || (a.x == b.x && a.y < b.y);
    }
};

TriangulatedPolygon::TriangulatedPolygon(const std::vector<std::shared_ptr<Point>>& points, const std::string& id) : Shape("TriangulatedPolygon"){
    this->raw_polygon = DrawWarp::GetInstance().CreateShape<NFP::Polygon>(points, id);
    this->triangulates = delaunay_triangulation(this->raw_polygon->getPoints());
    this->lines = std::static_pointer_cast<Polygon>(this->raw_polygon)->getLines();
    this->lineTypes = std::vector<LineType>(this->lines.size(), LineType::Regular);
    auto _ps = std::static_pointer_cast<Polygon>(this->raw_polygon)->getPoints();
    int num_line = this->lines.size();
    std::map<Vec2, int, Vec2Compare> pointMap;
    std::set<std::pair<int, int>> lineset;
    for(auto [p1, p2, p3]: this->triangulates){
        if(pointMap.find(p1) == pointMap.end()){
            pointMap.emplace(p1, getPointIdxInPolygonPoints(p1, _ps));
        }
        if(pointMap.find(p2) == pointMap.end()){
            pointMap.emplace(p2, getPointIdxInPolygonPoints(p2, _ps));
        }
        if(pointMap.find(p3) == pointMap.end()){
            pointMap.emplace(p3, getPointIdxInPolygonPoints(p3, _ps));
        }
        int idx1 = pointMap[p1];
        int idx2 = pointMap[p2];
        int idx3 = pointMap[p3];
        #define ADDLINE(idx1, idx2) \
        if((idx1 + 1) % num_line != idx2 && (idx2 + 1) % num_line != idx1){ \
            if(idx1 < idx2 && lineset.count(std::make_pair(idx1, idx2)) == 0){ \
                this->lines.push_back(std::make_shared<Line>(_ps[idx1]->getPoint(), _ps[idx2]->getPoint())); \
                this->lineTypes.push_back(LineType::Generated); \
                lineset.emplace(idx1, idx2); \
                lineset.emplace(idx2, idx1); \
            } \
            else if(lineset.count(std::make_pair(idx2, idx1)) == 0){ \
                this->lines.push_back(std::make_shared<Line>(_ps[idx2]->getPoint(), _ps[idx1]->getPoint())); \
                this->lineTypes.push_back(LineType::Generated); \
                lineset.emplace(idx1, idx2); \
                lineset.emplace(idx2, idx1); \
            } \
        }
        ADDLINE(idx1, idx2)
        ADDLINE(idx1, idx3)
        ADDLINE(idx2, idx3)
        #undef ADDLINE
    }
    auto cp = ConvexityPolygon(_ps);
    for(int i = 0; i < num_line; i++){
        if(cp.getPointsType()[i] == ConvexityPolygon::PointType::CONVEX &&
           cp.getPointsType()[(i + 1) % _ps.size()] == ConvexityPolygon::PointType::CONVEX){
            this->lineTypes[i] = LineType::AbsolutelyConvexLine;
        }
    }
}

void TriangulatedPolygon::draw(ImDrawList* draw_list, std::function<ImVec2(Vec2)>& trans) const{
    for (auto& line : this->lines){
        line->draw(draw_list, trans);
    }
    auto& ps = std::static_pointer_cast<Polygon>(this->raw_polygon)->getPoints();
    for (const auto& p : ps) {
        p->draw(draw_list, trans);
    }
}

const std::vector<std::tuple<Vec2, Vec2, Vec2>>& TriangulatedPolygon::getTriangulates() const {
    return this->triangulates;
}

bool doubleVec2Key::operator()(const std::pair<Vec2, Vec2>& a, const std::pair<Vec2, Vec2>& b) const{
    auto [a1, a2] = std::minmax(a.first, a.second, [](const Vec2& u, const Vec2& v) {
        return u.x < v.x || (u.x == v.x && u.y < v.y);
    });
    auto [b1, b2] = std::minmax(b.first, b.second, [](const Vec2& u, const Vec2& v) {
        return u.x < v.x || (u.x == v.x && u.y < v.y);
    });
    if (a1.x != b1.x) return a1.x < b1.x;
    if (a1.y != b1.y) return a1.y < b1.y;
    if (a2.x != b2.x) return a2.x < b2.x;
    return a2.y < b2.y;
}


std::map<std::pair<Vec2, Vec2>, int, doubleVec2Key> TriangulatedPolygon::getLineMapToIndex() const{
    auto res = std::map<std::pair<Vec2, Vec2>, int, doubleVec2Key>();
    for(int i = 0; i < this->lines.size(); i++){
        res[std::make_pair(this->lines[i]->getStartPoint(), this->lines[i]->getEndPoint())] = i;
    }
    return res;
}

const std::vector<std::shared_ptr<Line>>& TriangulatedPolygon::getLines() const{
    return this->lines;
}
}