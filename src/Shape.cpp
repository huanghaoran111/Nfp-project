#include <Shape.hpp>
#include <imgui.h>
#include <draw_warp.h>

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
    this->p = p_;
}

Point::Point(const Point& p_) : Shape("Point") {
    this->p = p_.getPoint();
    for(const auto& str : p_.getComeFrom()){
        this->come_from.push_back(str);
    }
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
    return sqrt((p[0].x - p[1].x) * (p[0].x - p[1].x) + (p[0].y - p[1].y) * (p[0].y - p[1].y));
}

float Line::getXangle() const{
    return atan2(p[1].y - p[0].y, p[1].x - p[0].x);
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
        float t = pq.Cross(s) / rxs;
        float u = pq.Cross(r) / rxs;

        if (t >= -EPSILON && t <= 1 + EPSILON &&
            u >= -EPSILON && u <= 1 + EPSILON) {
            Vec2 intersection = p1 + t * r; // or：q1 + u * s
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


Polygon::Polygon(const std::vector<std::shared_ptr<Point>>& Points): Shape("Polygon") {
    for(int i = 1; i < Points.size(); i++){
        Lines.push_back(std::make_shared<Line>(*(Points[i-1]), *(Points[i])));
        Lines.back()->setComeFrom(this->getId());
        this->Points.push_back(std::make_shared<Point>(Points[i - 1]->getPoint()));
        this->Points.back()->setComeFrom(this->getId());
    }
    Lines.push_back(std::make_shared<Line>(*(Points[Points.size()-1]), *(Points[0])));
    Lines.back()->setComeFrom(this->getId());
    this->Points.push_back(std::make_shared<Point>(Points[Points.size() - 1]->getPoint()));
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

const std::vector<std::shared_ptr<Line>>& Polygon::getLines() const {
    return this->Lines;
}

const std::vector<std::shared_ptr<Point>>& Polygon::getPoints() const {
    return this->Points;
}

int Polygon::GetLowestPointIdx() const {
    std::cout << "All points:" << std::endl;
    for (int i = 0; i < this->Points.size(); i++) {
        std::cout << "P" << i << ": (" << this->Points[i]->getPoint().x
            << ", " << this->Points[i]->getPoint().y << ")" << std::endl;
    }
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
    /*
    // ========= 1. 提取原始点 =========
    std::vector<Vec2> convexifiedPolygon;
    for (auto& p : raw_polygon->getPoints()) {
        convexifiedPolygon.push_back(p->getPoint());
    }

    // ========= 2. 执行凸化过程 =========
    std::vector<int> convexity = CheckPointConvexity(convexifiedPolygon);

    while (std::find(convexity.begin(), convexity.end(), 1) != convexity.end()) {
        size_t pa = std::numeric_limits<size_t>::max();
        size_t pb = std::numeric_limits<size_t>::max();

        // 2.1 找到一对合适的凸点
        for (size_t i = 0; i < convexity.size(); ++i) {
            if (convexity[i] == 0) { // 凸点
                pa = i;
                if (convexity[(i + 1) % convexity.size()] == 1) { // 凹点紧随其后
                    for (size_t j = (pa + 2) % convexity.size(); j != pa; j = (j + 1) % convexity.size()) {
                        if (convexity[j] == 0) { // 另一凸点
                            pb = j;
                            break;
                        }
                    }
                }
            }
            if (pb != std::numeric_limits<size_t>::max()) break;
        }

        size_t ini_p = pa;
        size_t last_p = pb;
        size_t start_p = ini_p;
        size_t end_p = last_p;

        // 2.2 同侧性检查
        auto checkSameSide = [&](int idx1, int idx2, int idx3) {
            return Line::arePointsOnSameSide(
                convexifiedPolygon[(idx1 + convexifiedPolygon.size()) % convexifiedPolygon.size()],
                convexifiedPolygon[(idx2 + convexifiedPolygon.size()) % convexifiedPolygon.size()],
                convexifiedPolygon[(idx3 + convexifiedPolygon.size()) % convexifiedPolygon.size()],
                convexifiedPolygon[ini_p],
                convexifiedPolygon[last_p]);
        };

        while (!checkSameSide(pa + 1, pa - 1, pb - 1) && pa != start_p) {
            pa = (pa - 1 + convexifiedPolygon.size()) % convexifiedPolygon.size();
            ini_p = pa;
        }

        while (!checkSameSide(pa - 1, pb + 1, pb - 1) && pb != end_p) {
            pb = (pb + 1) % convexifiedPolygon.size();
            last_p = pb;
        }

        // 2.3 删除凹点（保留 ini_p 与 last_p）
        std::vector<Vec2> updatedPoints;

        size_t ao_Start = (ini_p < last_p) ? ini_p : last_p;
        size_t ao_end = (ini_p < last_p) ? last_p : ini_p;

        int tag = (ao_Start + 1) % convexifiedPolygon.size();
        float A = convexifiedPolygon[ao_end].y - convexifiedPolygon[ao_Start].y;
        float B = convexifiedPolygon[ao_Start].x - convexifiedPolygon[ao_end].x;
        float C = convexifiedPolygon[ao_end].x * convexifiedPolygon[ao_Start].y
            - convexifiedPolygon[ao_Start].x * convexifiedPolygon[ao_end].y;

        if (A * convexifiedPolygon[tag].x + B * convexifiedPolygon[tag].y + C > 0)
            std::swap(ao_Start, ao_end);

        if (ao_Start > ao_end) {
            ao_end += convexifiedPolygon.size();
        }

        for (size_t i = 0; i < convexifiedPolygon.size(); ++i) {
            if ((i > ao_Start && i < ao_end) ||
                (ao_end >= convexifiedPolygon.size() && i < ao_end - convexifiedPolygon.size())) {
                continue;
            }
            updatedPoints.push_back(convexifiedPolygon[i]);
        }

        convexifiedPolygon = updatedPoints;
        convexity = CheckConvexity(convexifiedPolygon);
    }

    // ========= 3. 构造最终的边并存入 LinesType =========
    LinesType.clear();
    for (size_t i = 0; i < convexifiedPolygon.size(); ++i) {
        size_t j = (i + 1) % convexifiedPolygon.size();

        auto line = DrawWarp::GetInstance().CreateShape<Line>(
            Point(convexifiedPolygon[i].x, convexifiedPolygon[i].y),
            Point(convexifiedPolygon[j].x, convexifiedPolygon[j].y),
            Colors::BLACK
            );

        // 标记 come_from
        line->setComeFrom(this->getId());

        // 判断是否为原始边
        bool isRaw = false;
        for (auto& rawLine : Lines_) {
            if (line->isSame(rawLine)) {
                isRaw = true;
                break;
            }
        }

        // 分类存入
        if (isRaw) {
            LinesType[line] = LineType::RAW;
        }
        else {
            LinesType[line] = LineType::CONVEX;
        }
    }
    */
}

std::pair<Line, Line> ConvexityPolygon::getStartAndEndLines() const{
    return std::make_pair(*startLine, *endLine);
}

void ConvexityPolygon::draw(ImDrawList* draw_list, std::function<ImVec2(Vec2)>& trans) const{
    for (auto line : Lines){
        line->draw(draw_list, trans);
    }
}
