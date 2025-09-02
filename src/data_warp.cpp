#include <data_warp.h>
#include <UI.h>         // for EventActivator
// ======= common ========
static std::shared_ptr<Line> getMinRightAngleIntersection(std::shared_ptr<Line> current_line, std::vector<std::shared_ptr<Line>> trajectory_lines){
    // TODO:
    return nullptr;
    // return DrawWarp::GetInstance().CreateShape<Line>(0, 0, 1000, 1000, Colors::BLACK);
}

// ===== test ======
void xdn_test::apply()  {
    auto a = DrawWarp::GetInstance().CreateShape<Line>(200, 0, 300, 0, Colors::BLACK);
    auto b = DrawWarp::GetInstance().CreateShape<Line>(0, 0, 200, 0, Colors::RED);
    DrawWarp::GetInstance().addShape(a);
    DrawWarp::GetInstance().addShape(b);
    //auto res = a->isIntersectToLine(*b);
    //switch (res)
    //{
    //case Line::LineRelationship::PARTOVERLAP:
    //    std::cout << "isIntersect:  " << "PARTOVERLAP" << std::endl;
    //    break;
    //case Line::LineRelationship::NOTINTERSECT:
    //    std::cout << "isIntersect:  " << "NOTINTERSECT" << std::endl;
    //    break;
    //case Line::LineRelationship::INTERSECT:
    //    std::cout << "isIntersect:  " << "INTERSECT" << std::endl;
    //    break;
    //default:
    //    break;
    //}
}

// ===== Algorithm1:2006 =====
Algorithms1::Algorithms1(std::vector<std::shared_ptr<Shape>> polygon_data){
    this->polygon_data = polygon_data;
}
void Algorithms1::apply(){
    // TODO:
    
}

// ===== Algorithm2:2024 =====
Algorithms2::Algorithms2(std::vector<std::shared_ptr<Shape>> polygon_data){
    this->polygon_data = polygon_data;
}
void Algorithms2::apply(){
    // TODO:
    
}

namespace Case{
    namespace helper {
        bool testLineIntersect(std::shared_ptr<Shape> line1, std::shared_ptr<Shape> line2, Line::LineRelationship expectedRes, std::shared_ptr<Point> expectedIntersection = nullptr) {
            auto result = dynamic_cast<Line*>(line1.get())->findIntersection(*dynamic_cast<Line*>(line2.get()));
            
            // 先比较关系是否相同
            if (result.first != expectedRes) {
                std::cout << "expected Res=" << (int)expectedRes
                    << " actual Res=" << (int)result.first << std::endl;
                return false;
            }

            // 如果预期没有交点
            if (!expectedIntersection) {
                return result.second == nullptr;
            }

            // 如果预期有交点 → 比较坐标是否接近
            if (!result.second) return false;

            Vec2 ipt = (*result.second).getPoint();
            Vec2 ept = (*expectedIntersection).getPoint();

            if (std::fabs(ipt.x - ept.x) >= EPSILON || std::fabs(ipt.y - ept.y) >= EPSILON) {
                std::cout << "expected (" << ept.x << "," << ept.y
                    << ") but got (" << ipt.x << "," << ipt.y << ")" << std::endl;
                return false;
            }

            return true;
        }
        struct TwoLine {
            Line line1;
            Line line2;
            TwoLine(Point p1, Point p2, Point q1, Point q2) : line1(p1, p2), line2(q1,q2) {}
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
}

