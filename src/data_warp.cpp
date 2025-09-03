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
    // Step 1: ����ǵ���ʼ��ʸ������ֹ��ʸ��
    Vec2 angvec1 = P1 - P2;
    Vec2 angvec2 = P3 - P2;

    // Step 2: ˳ʱ�����ʱ����ת90��õ�������
    Vec2 Vs = angvec1.RotateClockwise90();    // ��ʼ������
    Vec2 Ve = angvec2.RotateCounterClockwise90(); // ��ֹ������

    // Step 3: ����ߵ�ʸ���ͱߵķ�����
    Vec2 edgevec = Pt2 - Pt1;
    Vec2 edgeNormal = edgevec.RotateClockwise90();

    // ʹ�ò���жϷ������Ƿ��ڷ�Χ��
    return IsNormalInRange(Vs, Ve, edgeNormal);
}

static std::shared_ptr<Point> getMinRightAngleIntersection(std::shared_ptr<Line> current_line, std::vector<std::shared_ptr<Line>> trajectory_lines){
    // TODO:���Ҳ�н���С���߶εĽ���

    float minRightAngle = std::numeric_limits<float>::max();
    std::shared_ptr<Point> finalIntersection = nullptr;
    // ��ǰ�ߵķ�������
    float baseAngle = current_line->getXangle();

    // ����Ҳ�н���С�����ǲ����ص����� => �ϲ����߶� ���滻ԭ����������

    for (auto& trajectory_line : trajectory_lines) {
        auto res = current_line->findIntersection(*trajectory_line);
        auto lineRelationship = res.first;
        auto intersection = res.second;

        // 1.���ཻ�� => ����
        // 2.�����ص� => �ϲ����߶� => ����ǵ�ǰ�ߵ���㣬�յ��Ǵ������ڵ�ǰ��֮��ĵ�
        // 3.�ཻ�� => ���뵱ǰ�ߵ��Ҳ�н�+����
        if (lineRelationship == Line::LineRelationship::NOTINTERSECT) {
            continue;
        }
        else if (lineRelationship == Line::LineRelationship::PARTOVERLAP) {
            // ��ǰ�ߵ�������Ϊx��С�ĵ㣻��ǰ�ߵ��յ����Ϊx���ĵ㣬x�����
            // ��ǰ�ߵ�������Ϊy��С�ĵ㣻��ǰ�ߵ��յ����Ϊy���ĵ�
            // �ռ������߶εĶ˵�
            std::vector<std::shared_ptr<Point>> pts = {
                std::make_shared<Point>(current_line->getStartPoint()),
                std::make_shared<Point>(current_line->getEndPoint()),
                std::make_shared<Point>(trajectory_line->getStartPoint()),
                std::make_shared<Point>(trajectory_line->getEndPoint())
            };

            // ��������Ȱ� x ������� x ����ٰ� y ����
            auto cmp = [](const std::shared_ptr<Point>& a, const std::shared_ptr<Point>& b) {
                if (a->getPoint().x != b->getPoint().x) return a->getPoint().x < b->getPoint().x;
                return a->getPoint().y < b->getPoint().y;
            };

            auto new_start = *std::min_element(pts.begin(), pts.end(), cmp);
            auto new_end = *std::max_element(pts.begin(), pts.end(), cmp);

            // �����µĺϲ��߶�
            auto merged_line = std::make_shared<Line>(
                new_start->getPoint().x, new_start->getPoint().y,
                new_end->getPoint().x, new_end->getPoint().y
                );

            return getMinRightAngleIntersection(merged_line, trajectory_lines);
        }
        else if (lineRelationship == Line::LineRelationship::INTERSECT) {
            float angle = trajectory_line->getXangle();
            float diff = angle - baseAngle;
            // ��һ���� [-��, ��]
            while (diff <= -PI) diff += 2 * PI;
            while (diff > PI) diff -= 2 * PI;

            if (diff < 0) { // �Ҳ�
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
    // ����finalIntersection��
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
    // TODO: 2024���㷨
    
}

namespace Case{
    namespace helper {
        bool testLineIntersect(std::shared_ptr<Shape> line1, std::shared_ptr<Shape> line2, Line::LineRelationship expectedRes, std::shared_ptr<Point> expectedIntersection = nullptr) {
            auto result = dynamic_cast<Line*>(line1.get())->findIntersection(*dynamic_cast<Line*>(line2.get()));
            
            // �ȱȽϹ�ϵ�Ƿ���ͬ
            if (result.first != expectedRes) {
                return false;
            }

            // ���Ԥ��û�н���
            if (!expectedIntersection) {
                return result.second == nullptr;
            }

            // ���Ԥ���н��� �� �Ƚ������Ƿ�ӽ�
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

