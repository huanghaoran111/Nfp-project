#include <data_warp.h>
#include <UI.h>         // for EventActivator

/* 
 * common functions:
 * IsNormalInRange - 
 * IsPossibleContact - 
 * getMinRightAngleIntersection - 
 * 
 * algo xxxx
 * xxxxx - xxxx
 */

// ======= common ========
static bool IsNormalInRange(const Vec2& VecStart, const Vec2& VecEnd, const Vec2& normal) {

    float crossStart_normal = VecStart.Cross(normal);
    float crossStart_end = VecStart.Cross(VecEnd);
    float crossNormal_end = normal.Cross(VecEnd);

    bool condition1 = !(std::signbit(crossStart_end) ^ std::signbit(crossStart_normal));
    bool condition2 = !(std::signbit(crossNormal_end) ^ std::signbit(crossStart_end));

    return crossStart_end * crossStart_normal >= 0 && crossNormal_end * crossStart_end >= 0;
}

static bool IsPossibleContact(const Vec2& P1, const Vec2& P2, const Vec2& P3, const Vec2& Pt1, const Vec2& Pt2) {
    // Step 1: 计算角的起始边矢量和终止边矢量
    Vec2 angvec1 = P1 - P2;
    Vec2 angvec2 = P3 - P2;

    // Step 2: 顺时针和逆时针旋转90°得到法向量
    Vec2 Vs = angvec1.RotateClockwise90();    // 起始法向量
    Vec2 Ve = angvec2.RotateCounterClockwise90(); // 终止法向量

    // Step 3: 计算边的矢量和边的法向量
    Vec2 edgevec = Pt2 - Pt1;
    Vec2 edgeNormal = edgevec.RotateClockwise90();

    // 使用叉乘判断法向量是否在范围内
    return IsNormalInRange(Vs, Ve, edgeNormal);
}

static std::shared_ptr<Point> getMinRightAngleIntersection(std::shared_ptr<Line> current_line, std::vector<std::shared_ptr<Line>> trajectory_lines){
    // TODO:求右侧夹角最小的线段的交点

    float minRightAngle = std::numeric_limits<float>::max();
    std::shared_ptr<Point> finalIntersection = nullptr;
    // 当前线的方向向量
    float baseAngle = current_line->getXangle();

    // 如果右侧夹角最小的线是部分重叠的线 => 合并两线段 并替换原来的两条线

    for (auto& trajectory_line : trajectory_lines) {
        auto res = current_line->findIntersection(*trajectory_line);
        auto lineRelationship = res.first;
        auto intersection = res.second;

        // 1.不相交线 => 跳过
        // 2.部分重叠 => 合并两线段 => 起点是当前线的起点，终点是待测线在当前线之外的点
        // 3.相交线 => 求与当前线的右侧夹角+交点
        if (lineRelationship == Line::LineRelationship::NOTINTERSECT) {
            continue;
        }
        else if (lineRelationship == Line::LineRelationship::PARTOVERLAP) {
            // 当前线的起点更新为x最小的点；当前线的终点更新为x最大的点，x相等则：
            // 当前线的起点更新为y最小的点；当前线的终点更新为y最大的点
            // 收集两个线段的端点
            std::vector<std::shared_ptr<Point>> pts = {
                std::make_shared<Point>(current_line->getStartPoint()),
                std::make_shared<Point>(current_line->getEndPoint()),
                std::make_shared<Point>(trajectory_line->getStartPoint()),
                std::make_shared<Point>(trajectory_line->getEndPoint())
            };

            // 排序规则：先按 x 升序，如果 x 相等再按 y 升序
            auto cmp = [](const std::shared_ptr<Point>& a, const std::shared_ptr<Point>& b) {
                if (a->getPoint().x != b->getPoint().x) return a->getPoint().x < b->getPoint().x;
                return a->getPoint().y < b->getPoint().y;
            };

            auto new_start = *std::min_element(pts.begin(), pts.end(), cmp);
            auto new_end = *std::max_element(pts.begin(), pts.end(), cmp);

            // 生成新的合并线段
            auto merged_line = std::make_shared<Line>(
                new_start->getPoint().x, new_start->getPoint().y,
                new_end->getPoint().x, new_end->getPoint().y
                );

            return getMinRightAngleIntersection(merged_line, trajectory_lines);
        }
        else if (lineRelationship == Line::LineRelationship::INTERSECT) {
            float angle = trajectory_line->getXangle();
            float diff = angle - baseAngle;
            // 归一化到 [-π, π]
            while (diff <= -PI) diff += 2 * PI;
            while (diff > PI) diff -= 2 * PI;

            if (diff < 0) { // 右侧
                float rightAngle = -diff;
                if (rightAngle < minRightAngle) {
                    minRightAngle = rightAngle;
                    finalIntersection = intersection;
                }
            }
            //finalIntersection->setComeFrom(current_line->getId());
            //finalIntersection->setComeFrom(trajectory_line->getId());
        }
    }
    // 绘制finalIntersection点
    DrawWarp::GetInstance().addShape(finalIntersection);
    return finalIntersection;
    // return DrawWarp::GetInstance().CreateShape<Line>(0, 0, 1000, 1000, Colors::BLACK);
}




// ===== test ======
void xdn_test::apply()  {
    auto a = DrawWarp::GetInstance().CreateShape<Line>(200, 0, 300, 0, Colors::BLACK);
    auto b = DrawWarp::GetInstance().CreateShape<Line>(0, 0, 200, 0, Colors::RED);
    DrawWarp::GetInstance().addShape(a);
    DrawWarp::GetInstance().addShape(b);
}

// ===== Algorithm1:2006 =====
Algorithms1::Algorithms1(std::vector<std::shared_ptr<Shape>> polygon_data){
    this->polygon_data = polygon_data;
}
int Algorithms1::step1() {
    // TODO: algo step1
    return 1;
}

void Algorithms1::step2() {
    // TODO: algo step2
}

// ...
void Algorithms1::apply(){
    // TODO: step1 + step2 + loop(step2 + step3)
    auto a = this->step1();

}

// ===== Algorithm2:2024 =====
Algorithms2::Algorithms2(std::vector<std::shared_ptr<Shape>> polygon_data){
    this->polygon_data = polygon_data;
}
void Algorithms2::apply(){
    // TODO: 2024的算法
    
}

namespace Case{
    namespace helper {
        bool testLineIntersect(std::shared_ptr<Shape> line1, std::shared_ptr<Shape> line2, Line::LineRelationship expectedRes, std::shared_ptr<Point> expectedIntersection = nullptr) {
            auto result = dynamic_cast<Line*>(line1.get())->findIntersection(*dynamic_cast<Line*>(line2.get()));
            
            // 先比较关系是否相同
            if (result.first != expectedRes) {
                return false;
            }

            // 如果预期没有交点
            if (!expectedIntersection) {
                return result.second == nullptr;
            }

            // 如果预期有交点 → 比较坐标是否接近
            if (!result.second) return false;

            Vec2 ipt = result.second->getPoint();
            Vec2 ept = expectedIntersection->getPoint();

            return std::fabs((ipt - ept).x) < EPSILON || std::fabs((ipt - ept).y) < EPSILON;
        }
        struct TwoLine {
            Line line1;
            Line line2;
            TwoLine(Point p1, Point p2, Point q1, Point q2) : line1(p1.getPoint(), p2.getPoint()), line2(q1.getPoint(), q2.getPoint()) {}
        };
    }
    bool caseTemp(helper::TwoLine twoLine, Line::LineRelationship res, std::shared_ptr<Point> intersection = nullptr) {
        auto a = DrawWarp::GetInstance().CreateShape<Line>(twoLine.line1.getStartPoint(), twoLine.line1.getEndPoint(), Colors::BLACK);
        auto b = DrawWarp::GetInstance().CreateShape<Line>(twoLine.line2.getStartPoint(), twoLine.line2.getEndPoint(), Colors::RED);
        return helper::testLineIntersect(a, b, res, intersection);
    }

}

void TestCases::apply() {
    using namespace Case;
    bool res = true;
    res &= caseTemp(helper::TwoLine(Point(200, 0), Point(300, 0), Point(0, 0), Point(200, 0)), Line::LineRelationship::NOTINTERSECT);
    assert(res);
    res &= caseTemp(helper::TwoLine(Point(0, 0), Point(200, 0), Point(200, 0), Point(300, 0)), Line::LineRelationship::PARTOVERLAP);
    assert(res);
    res &= caseTemp(helper::TwoLine(Point(0, 0), Point(100, 0), Point(200, 0), Point(300, 0)), Line::LineRelationship::NOTINTERSECT);
    assert(res);
    res &= caseTemp(helper::TwoLine(Point(0, 0), Point(100, 100), Point(100, 100), Point(200, 200)), Line::LineRelationship::PARTOVERLAP);
    assert(res);
    res &= caseTemp(helper::TwoLine(Point(0, 0), Point(100, 100), Point(200, 200), Point(300, 300)), Line::LineRelationship::NOTINTERSECT);
    assert(res);
    res &= caseTemp(helper::TwoLine(Point(0, 0), Point(200, 0), Point(100, 0), Point(300, 0)), Line::LineRelationship::PARTOVERLAP);
    assert(res);
    res &= caseTemp(helper::TwoLine(Point(0, 0), Point(0, 200), Point(0, -100), Point(0, 100)), Line::LineRelationship::NOTINTERSECT);
    assert(res);
    res &= caseTemp(helper::TwoLine(Point(0, 0), Point(200, 0), Point(100, -100), Point(100, 200)), Line::LineRelationship::INTERSECT, std::make_shared<Point>(100, 0));
    assert(res);
    res &= caseTemp(helper::TwoLine(Point(0, 0), Point(200, 0), Point(100, 0), Point(100, 100)), Line::LineRelationship::INTERSECT, std::make_shared<Point>(100, 0));
    assert(res);
    res &= caseTemp(helper::TwoLine(Point(0, 0), Point(0, 100), Point(0, 200), Point(0, 300)), Line::LineRelationship::NOTINTERSECT);
    assert(res);
    res &= caseTemp(helper::TwoLine(Point(0, 0), Point(0, 100), Point(0, 100), Point(0, 300)), Line::LineRelationship::PARTOVERLAP);
    assert(res);
    res &= caseTemp(helper::TwoLine(Point(200, 0), Point(0, 0), Point(100, 0), Point(-100, 0)), Line::LineRelationship::PARTOVERLAP);
    assert(res);
    res &= caseTemp(helper::TwoLine(Point(200, 0), Point(0, 0), Point(-100, 0), Point(100, 0)), Line::LineRelationship::PARTOVERLAP);
    assert(res);
    res &= caseTemp(helper::TwoLine(Point(0, 0), Point(200, 0), Point(-100, 0), Point(100, 0)), Line::LineRelationship::NOTINTERSECT);
    assert(res);
}

