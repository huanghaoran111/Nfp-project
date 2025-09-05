#include <data_warp.h>
#include <UI.h>         // for EventActivator

/* 
 * common functions:
 * IsNormalInRange - �ж��ߵķ������Ƿ��ڶ��������������Χ��
 * IsPointandLinePossibleContact - �ж϶�������Ƿ���ܽӴ�
 * getClosetIntersection - ���뵱ǰ���ཻ�������߶εĽ����У��������Ŀ�꽻������Ľ���
 * getMinRightAngleLine - ��ȡ����������߶����뵱ǰ���Ҳ�н���С���߶�
 * getOuterNFP - ��ȡ��ΧNFP
 * 
 * algo xxxx
 * xxxxx - xxxx
 */

// ======= common ========
static bool IsNormalInRange(const Vec2& VecStart, const Vec2& VecEnd, const Vec2& normal) {
    // ������

    float crossStart_normal = VecStart.Cross(normal);
    float crossStart_end = VecStart.Cross(VecEnd);
    float crossNormal_end = normal.Cross(VecEnd);

    bool condition1 = !(std::signbit(crossStart_end) ^ std::signbit(crossStart_normal));
    bool condition2 = !(std::signbit(crossNormal_end) ^ std::signbit(crossStart_end));

    return crossStart_end * crossStart_normal >= 0 && crossNormal_end * crossStart_end >= 0;
}

static bool IsPointandLinePossibleContact(const Vec2& P1, const Vec2& P2, const Vec2& P3, const Vec2& Pt1, const Vec2& Pt2) {
    // ������
     
    // ����ǵ���ʼ��ʸ������ֹ��ʸ��
    Vec2 angvec1 = P1 - P2;
    Vec2 angvec2 = P3 - P2;

    // ˳ʱ�����ʱ����ת90��õ�������
    Vec2 Vs = angvec1.RotateClockwise90();    // ��ʼ������
    Vec2 Ve = angvec2.RotateCounterClockwise90(); // ��ֹ������

    // ����ߵ�ʸ���ͱߵķ�����
    Vec2 edgevec = Pt2 - Pt1;
    Vec2 edgeNormal = edgevec.RotateClockwise90();

    // ʹ�ò���жϷ������Ƿ��ڷ�Χ��
    return IsNormalInRange(Vs, Ve, edgeNormal);
}

static std::shared_ptr<Point> getClosetIntersection(
    std::shared_ptr<Point> targetIntersection,
    std::shared_ptr<Line> current_line,
    std::vector<std::shared_ptr<Line>> trajectory_lines) {
    // TODO:����current_line�ཻ�������߶εĽ����У��������targetIntersection����Ľ���
    
    std::shared_ptr<Point> closetIntersection = nullptr;
    float minDistance = std::numeric_limits<float>::max();
    for (auto& trajectory_line : trajectory_lines) {
        auto res = current_line->findIntersection(*trajectory_line);
        auto lineRelationship = res.first;
        auto intersection = res.second;
        // ����ཻ => �󽻵�
        if (lineRelationship == Line::LineRelationship::INTERSECT) {
            // �ų��������Ŀ��㱾������
            if (intersection->getPoint() == targetIntersection->getPoint()) {
                continue;
            }
            auto distance = Line(intersection->getPoint(), targetIntersection->getPoint()).getLength();
            if (distance < minDistance && abs(distance - minDistance) > EPSILON) {
                minDistance = distance;
                closetIntersection = intersection;
            }
        }
        // �����ص����Ⱥϲ������������
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

            return getClosetIntersection(targetIntersection, merged_line, trajectory_lines);
        }
    }
    // ����finalIntersection��
    if (closetIntersection != nullptr) {
        DrawWarp::GetInstance().addShape(closetIntersection);
    }
    return closetIntersection;
}

static std::shared_ptr<Line> getMinRightAngleLine(
    std::shared_ptr<Point> intersection,
    std::shared_ptr<Line> current_line, 
    std::vector<std::shared_ptr<Line>> trajectory_lines) {
    // TODO:���뵱ǰ���Ҳ�н���С���߶�
    //trajectory_lines�о������������line����current_line�Ҳ�н���С���߶�

    float minRightAngle = std::numeric_limits<float>::max();
    float current_line_angle = current_line->getXangle();
    std::shared_ptr<Line> final_line = nullptr;
    // �����켣���е���
    for (auto& trajectory_line : trajectory_lines) {
        // �����ǰ�������߾�������
        if (trajectory_line->whereIsPointOnLine(*intersection) == 0) {
            // �������current_line���Ҳ�н�
            float trajectory_line_angle = trajectory_line->getXangle();
            float angleDiff = trajectory_line_angle - current_line_angle;
            // ���ǶȲ�淶����[0, 2��]
            if (angleDiff <= 0) {
                angleDiff += 2 * PI;
            }
            // ѡ���Ҳ�н���С���߶�
            if (angleDiff < minRightAngle) {
                minRightAngle = angleDiff;
                final_line = trajectory_line;
            }
        }
    }
    return final_line;
}

/*
 * start_line - ��ʼ��ȡλ��
 * end_line - ��ֹ��ȡλ�� ��ֻ����ȡ�ֲ�����NFP�Ż��õ���������������㷨Ĭ��end_line=start_line��
 */
static std::vector<std::shared_ptr<Point>> getOuterNFP(
    std::shared_ptr<Line> start_line, 
    std::shared_ptr<Line> end_line,
    std::vector<std::shared_ptr<Line>> trajectory_lines) {
    // TODO:��ȡNFP��Χ��
     
    // ��ʼ����ǰ�߶�Ϊstart_line
    auto current_line = start_line;
    // ��ʼ������Ϊstart_line�����
    auto closetIntersection = DrawWarp::GetInstance().CreateShape<Point>(start_line->getStartPoint());
    std::vector<std::shared_ptr<Point>> finalNFP;
    // ��start_line��������NFP
    finalNFP.push_back(DrawWarp::GetInstance().CreateShape<Point>(start_line->getStartPoint()));
    do {
        // ������current_line�ཻ�������߶εĽ����У��������intersection����Ľ���
        closetIntersection = getClosetIntersection(closetIntersection, current_line, trajectory_lines);
        // ����current_lineΪtrajectory_lines�о������������line����current_line�Ҳ�н���С���߶�
        current_line = getMinRightAngleLine(closetIntersection, current_line, trajectory_lines);
        
        // ��������������NFP
        finalNFP.push_back(closetIntersection);
    } while (closetIntersection->getPoint() != end_line->getStartPoint());
   
    return finalNFP;
}


// ===== test ======
void xdn_test::apply()  {
    auto a = DrawWarp::GetInstance().CreateShape<Line>(200, 0, 300, 0, Colors::BLACK);
    auto b = DrawWarp::GetInstance().CreateShape<Line>(0, 0, 200, 0, Colors::RED);
    DrawWarp::GetInstance().addShape(a);
    DrawWarp::GetInstance().addShape(b);
}

// ===== Algorithm1:GridNFP 2006 =====
GridNFPAlgorithm::GridNFPAlgorithm(std::vector<std::shared_ptr<Shape>> polygon_data){
    this->polygon_data = polygon_data;
}
int GridNFPAlgorithm::step1() {
    // TODO: algo step1
    return 1;
}

void GridNFPAlgorithm::step2() {
    // TODO: algo step2
}

// ...
void GridNFPAlgorithm::apply(){
    // TODO: step1 + step2 + loop(step2 + step3)
    auto a = this->step1();

}

// ===== Algorithm2:LocalContour 2024 =====
LocalContourNFPAlgorithm::LocalContourNFPAlgorithm(std::vector<std::shared_ptr<Shape>> polygon_data){
    this->polygon_data = polygon_data;
}

//float DeterminePositiveDirection(const std::vector<Vec2>& vertices) {
//    int minIdx = FindLowestPoint(vertices);
//    // ������͵�ǰһ��������� prevIdx
//    // �����͵��ǵ�һ���㣨minIdx == 0������ôǰһ����������һ���㣨�γɻ�״���ӣ�
//    // ����ǰһ���������͵�ǰ��ĵ㡣
//    size_t prevIdx = (minIdx == 0) ? vertices.size() - 1 : static_cast<size_t>(minIdx) - 1;
//    size_t nextIdx = (static_cast<size_t>(minIdx) + 1) % vertices.size();
//
//    Vec2 edge1 = vertices[minIdx] - vertices[prevIdx];
//    Vec2 edge2 = vertices[nextIdx] - vertices[minIdx];
//
//    /*
//    ������Ϊ������ʾ�� edge1 �� edge2 ����ʱ�뷽��
//    ������Ϊ������ʾ��˳ʱ�뷽��
//    ������Ϊ�㣬��ʾ�������㹲�ߡ�
//    */
//    float positiveDirection = edge1.Cross(edge2);
//    return positiveDirection;
//}
//
//void LocalContourNFPAlgorithm::checkConvexity() {
//    // Ĭ�����е�Ϊ͹�㣨0��
//    std::vector<int> convexity(vertices.size(), 0);
//
//    float positiveDirection = ConvexityCheck::DeterminePositiveDirection(vertices);
//
//    for (size_t i = 0; i < vertices.size(); ++i) {
//        int prevIdx = (i == 0) ? vertices.size() - 1 : i - 1;
//        int nextIdx = (i + 1) % vertices.size();
//
//        Vec2 edge1 = vertices[i] - vertices[prevIdx];
//        Vec2 edge2 = vertices[nextIdx] - vertices[i];
//
//        float crossZ = edge1.Cross(edge2);
//
//        if (crossZ * positiveDirection < 0) convexity[i] = 1;  // ����
//        else if (std::fabs(crossZ) < EPSILON) convexity[i] = 0; // ���ߵĵ�Ҳ��Ϊ����
//        else convexity[i] = 0; // ͹��
//    }
//    return convexity;
//}

void LocalContourNFPAlgorithm::apply(){
    // TODO: 2024���㷨
    // step1������ζ��㰼͹�Է���
    // step2��͹���㷨�������߹���
    // step3������˫͹����ε�NFP
    // step4�����ݸ��������������ֲ�����
    // step5�����߽Ӵ��ж�
    // step6���켣�߼��������㷨
    // step7����ΧNFP��ȡ�㷨
    // step8��NFP�ϲ��㷨
}

// ===== Algorithm3:TwoLocalContour 2024o =====
TwoLocalContourNFPAlgorithm::TwoLocalContourNFPAlgorithm(std::vector<std::shared_ptr<Shape>> polygon_data) {
    this->polygon_data = polygon_data;
}
void TwoLocalContourNFPAlgorithm::apply() {
    // TODO: �����µ��㷨

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
        bool testGetMinRightAngleLine(std::shared_ptr<Point> intersection, std::shared_ptr<Line> currentLine, std::vector <std::shared_ptr<Line>> trajectoryLines, std::shared_ptr<Line> expectedLine = nullptr) {
            auto result = getMinRightAngleLine(intersection, currentLine, trajectoryLines);

            if (!expectedLine) {
                return result == nullptr;
            }

            if (!result) return false;

            auto rStart = result->getStartPoint();
            auto rEnd = result->getEndPoint();
            auto eStart = expectedLine->getStartPoint();
            auto eEnd = expectedLine->getEndPoint();

            return ((std::fabs(rStart.x - eStart.x) < EPSILON && std::fabs(rStart.y - eStart.y) < EPSILON &&
                std::fabs(rEnd.x - eEnd.x) < EPSILON && std::fabs(rEnd.y - eEnd.y) < EPSILON)
                ||
                (std::fabs(rStart.x - eEnd.x) < EPSILON && std::fabs(rStart.y - eEnd.y) < EPSILON &&
                    std::fabs(rEnd.x - eStart.x) < EPSILON && std::fabs(rEnd.y - eStart.y) < EPSILON));
        }
        struct CurrentAndTrajectories {
            std::shared_ptr<Line> current;
            std::vector<std::shared_ptr<Line>> trajectories;
            CurrentAndTrajectories(Line c, std::initializer_list<Line> ts) {
                current = DrawWarp::GetInstance().CreateShape<Line>(c.getStartPoint(), c.getEndPoint(), Colors::BLACK);
                for (auto& t : ts) {
                    trajectories.push_back(DrawWarp::GetInstance().CreateShape<Line>(t.getStartPoint(), t.getEndPoint(), Colors::RED));
                }
            }
        };
        bool testGetClosetIntersection(std::shared_ptr<Point> targetIntersection, std::shared_ptr<Line> current_line, std::vector<std::shared_ptr<Line>> trajectory_lines, std::shared_ptr<Point> expectedIntersection = nullptr) {
            auto res = getClosetIntersection(targetIntersection, current_line, trajectory_lines);
            if (!expectedIntersection) {
                return res == nullptr;
            }
            if (!res) return false;

            Vec2 ipt = res->getPoint();
            Vec2 ept = expectedIntersection->getPoint();
            return std::fabs(ipt.x - ept.x) < EPSILON && std::fabs(ipt.y - ept.y) < EPSILON;
        }
        bool testGetOuterNFP(std::shared_ptr<Line> start_line, std::shared_ptr<Line> end_line, std::vector<std::shared_ptr<Line>> trajectory_lines, std::vector<std::shared_ptr<Point>> expectedPoints) {
            auto result = getOuterNFP(start_line, end_line, trajectory_lines);

            if (result.size() != expectedPoints.size()) {
                return false;
            }

            for (size_t i = 0; i < result.size(); ++i) {
                Vec2 r = result[i]->getPoint();
                Vec2 e = expectedPoints[i]->getPoint();
                if (std::fabs(r.x - e.x) > EPSILON || std::fabs(r.y - e.y) > EPSILON) {
                    return false;
                }
            }
            return true;
        }
    }
    bool caseLineIntersect(helper::TwoLine twoLine, 
        Line::LineRelationship res, 
        std::shared_ptr<Point> intersection = nullptr) {
        auto a = DrawWarp::GetInstance().CreateShape<Line>(twoLine.line1.getStartPoint(), twoLine.line1.getEndPoint(), Colors::BLACK);
        auto b = DrawWarp::GetInstance().CreateShape<Line>(twoLine.line2.getStartPoint(), twoLine.line2.getEndPoint(), Colors::RED);
        return helper::testLineIntersect(a, b, res, intersection);
    }
    bool caseRightAngle(helper::CurrentAndTrajectories ct, 
        std::shared_ptr<Point> intersection, 
        std::shared_ptr<Line> expected = nullptr) {
        return helper::testGetMinRightAngleLine(intersection, ct.current, ct.trajectories, expected);
    }
    bool caseClosetIntersection(std::shared_ptr<Point> targetIntersection,
        std::shared_ptr<Line> currentLine,
        std::vector<std::shared_ptr<Line>> trajectoryLines,
        std::shared_ptr<Point> expectedIntersection = nullptr) {
        return helper::testGetClosetIntersection(targetIntersection, currentLine, trajectoryLines, expectedIntersection);
    }
    bool caseOuterNFP(
        std::shared_ptr<Line> start_line,
        std::shared_ptr<Line> end_line,
        std::vector<std::shared_ptr<Line>> trajectory_lines,
        std::vector<std::shared_ptr<Point>> expectedPoints){
        return helper::testGetOuterNFP(start_line, end_line, trajectory_lines, expectedPoints);
    }

}

void TestCases::apply() {
    using namespace Case;
    bool res = true;
    res &= caseLineIntersect(helper::TwoLine(Point(200, 0), Point(300, 0), Point(0, 0), Point(200, 0)), Line::LineRelationship::NOTINTERSECT);
    assert(res);
    res &= caseLineIntersect(helper::TwoLine(Point(0, 0), Point(200, 0), Point(200, 0), Point(300, 0)), Line::LineRelationship::PARTOVERLAP);
    assert(res);
    res &= caseLineIntersect(helper::TwoLine(Point(0, 0), Point(100, 0), Point(200, 0), Point(300, 0)), Line::LineRelationship::NOTINTERSECT);
    assert(res);
    res &= caseLineIntersect(helper::TwoLine(Point(0, 0), Point(100, 100), Point(100, 100), Point(200, 200)), Line::LineRelationship::PARTOVERLAP);
    assert(res);
    res &= caseLineIntersect(helper::TwoLine(Point(0, 0), Point(100, 100), Point(200, 200), Point(300, 300)), Line::LineRelationship::NOTINTERSECT);
    assert(res);
    res &= caseLineIntersect(helper::TwoLine(Point(0, 0), Point(200, 0), Point(100, 0), Point(300, 0)), Line::LineRelationship::PARTOVERLAP);
    assert(res);
    res &= caseLineIntersect(helper::TwoLine(Point(0, 0), Point(0, 200), Point(0, -100), Point(0, 100)), Line::LineRelationship::NOTINTERSECT);
    assert(res);
    res &= caseLineIntersect(helper::TwoLine(Point(0, 0), Point(200, 0), Point(100, -100), Point(100, 200)), Line::LineRelationship::INTERSECT, std::make_shared<Point>(100, 0));
    assert(res);
    res &= caseLineIntersect(helper::TwoLine(Point(0, 0), Point(200, 0), Point(100, 0), Point(100, 100)), Line::LineRelationship::INTERSECT, std::make_shared<Point>(100, 0));
    assert(res);
    res &= caseLineIntersect(helper::TwoLine(Point(0, 0), Point(0, 100), Point(0, 200), Point(0, 300)), Line::LineRelationship::NOTINTERSECT);
    assert(res);
    res &= caseLineIntersect(helper::TwoLine(Point(0, 0), Point(0, 100), Point(0, 100), Point(0, 300)), Line::LineRelationship::PARTOVERLAP);
    assert(res);
    res &= caseLineIntersect(helper::TwoLine(Point(200, 0), Point(0, 0), Point(100, 0), Point(-100, 0)), Line::LineRelationship::PARTOVERLAP);
    assert(res);
    res &= caseLineIntersect(helper::TwoLine(Point(200, 0), Point(0, 0), Point(-100, 0), Point(100, 0)), Line::LineRelationship::PARTOVERLAP);
    assert(res);
    res &= caseLineIntersect(helper::TwoLine(Point(0, 0), Point(200, 0), Point(-100, 0), Point(100, 0)), Line::LineRelationship::NOTINTERSECT);
    assert(res);
    res &= caseLineIntersect(helper::TwoLine(Point(0, 0), Point(200, 0), Point(200, 0), Point(300, 300)), Line::LineRelationship::INTERSECT, std::make_shared<Point>(200, 0));
    assert(res);
    std::cout << "All LineIntersect tests passed!" << std::endl;
    // ---------- getMinRightAngleLine ���� ----------
    // case 1: û�н��� �� ���ӦΪ nullptr
    res &= caseRightAngle(
        helper::CurrentAndTrajectories(
            Line(Point(0, 0), Point(200, 0)),
            { Line(Point(0, 100), Point(200, 100)) } // ƽ�У����ཻ
        ),
        std::make_shared<Point>(100, 0),  // ��һ�����㣬�� trajectory ������
        nullptr
    );
    assert(res);
    // case 2: ��һ�ཻ �� Ӧ��ѡΨһ������
    {
        auto expected = DrawWarp::GetInstance().CreateShape<Line>(Point(100, -100), Point(100, 100), Colors::RED);
        res &= caseRightAngle(
            helper::CurrentAndTrajectories(
                Line(Point(0, 0), Point(200, 0)),
                { Line(Point(100, -100), Point(100, 100)) }
            ),
            std::make_shared<Point>(100, 0),
            expected
        );
        assert(res);
    }
    // case 3: ����ཻ �� ѡ�񾭹������
    {
        auto expected = DrawWarp::GetInstance().CreateShape<Line>(Point(100, 0), Point(150, 50), Colors::RED); // ��ֱ���н�С
        res &= caseRightAngle(
            helper::CurrentAndTrajectories(
                Line(Point(0, 0), Point(200, 0)),
                {
                    Line(Point(100, 0), Point(150, 50)),  // ���ϣ��нǴ�
                    Line(Point(50, -50), Point(50, 50))   // ��ֱ���н�С
                }
            ),
            std::make_shared<Point>(100, 0),
            expected
        );
        assert(res);
    }
    // case 4: ��������ཻ���� �� ѡ���Ҳ�н���С��
    {
        auto expected = DrawWarp::GetInstance().CreateShape<Line>(Point(100, 0), Point(150, 50), Colors::RED);
        res &= caseRightAngle(
            helper::CurrentAndTrajectories(
                Line(Point(0, 0), Point(200, 0)),
                {
                    Line(Point(100, 0), Point(150, 50)),
                    Line(Point(100, -50), Point(100, 50))
                }
            ),
            std::make_shared<Point>(100, 0),
            expected
        );
        assert(res);
    }
    {
        auto expected = DrawWarp::GetInstance().CreateShape<Line>(Point(-100, 100), Point(100, -100), Colors::RED);
        res &= caseRightAngle(
            helper::CurrentAndTrajectories(
                Line(Point(0, 0), Point(100, 100)),
                {
                    Line(Point(100, -100), Point(-100, 100)),
                    Line(Point(-100, 100), Point(100, -100))
                }
            ),
            std::make_shared<Point>(0, 0),
            expected
        );
        assert(res);
    }
    {
        auto expected = DrawWarp::GetInstance().CreateShape<Line>(Point(200, 50), Point(-200, -50), Colors::RED);
        res &= caseRightAngle(
            helper::CurrentAndTrajectories(
                Line(Point(-100, -100), Point(100, 100)),
                {
                    Line(Point(200, 50), Point(-200, -50)),
                    Line(Point(-100, 50), Point(100, -50))
                }
            ),
            std::make_shared<Point>(0, 0),
            expected
        );
        assert(res);
    }
    {
        auto expected = DrawWarp::GetInstance().CreateShape<Line>(Point(0, 0), Point(100, 50), Colors::RED);
        res &= caseRightAngle(
            helper::CurrentAndTrajectories(
                Line(Point(0, 0), Point(100, 0)),
                {
                    Line(Point(0, 0), Point(100, 50)),
                    Line(Point(0, 0), Point(-100, -50))
                }
            ),
            std::make_shared<Point>(0, 0),
            expected
        );
        assert(res);
    }
    {
        auto expected = DrawWarp::GetInstance().CreateShape<Line>(Point(0, 0), Point(-100, -50), Colors::RED);
        res &= caseRightAngle(
            helper::CurrentAndTrajectories(
                Line(Point(-100, -100), Point(100, 100)),
                {
                    Line(Point(0, 0), Point(100, 50)),
                    Line(Point(0, 0), Point(-100, -50))
                }
            ),
            std::make_shared<Point>(0, 0),
            expected
        );
        assert(res);
    }
    {
        auto expected = DrawWarp::GetInstance().CreateShape<Line>(Point(-100, -100), Point(100, 100), Colors::RED);
        res &= caseRightAngle(
            helper::CurrentAndTrajectories(
                Line(Point(0, 0), Point(100, 0)),
                {
                    Line(Point(100, 50), Point(-100, -50)),
                    Line(Point(-100, -100), Point(100, 100))
                }
            ),
            std::make_shared<Point>(0, 0),
            expected
        );
        assert(res);
    }
    std::cout << "All getMinRightAngleLine tests passed!" << std::endl;
    // ---------- getClosetIntersection ���� ----------
    {
        auto current = DrawWarp::GetInstance().CreateShape<Line>(Point(0, 0), Point(200, 0), Colors::BLACK);
        std::vector<std::shared_ptr<Line>> trajectories = {
            DrawWarp::GetInstance().CreateShape<Line>(Point(100, -100), Point(100, 100), Colors::RED),
            DrawWarp::GetInstance().CreateShape<Line>(Point(150, -100), Point(150, 100), Colors::RED)
        };
        // case 1: ����Ľ����� (100,0)
        res &= caseClosetIntersection(
            std::make_shared<Point>(0, 0),
            current,
            trajectories,
            std::make_shared<Point>(100, 0)
        );
        assert(res);
        // case 2: Ŀ�꿿�� (150,0)������Ӧѡ (150,0)
        res &= caseClosetIntersection(
            std::make_shared<Point>(140, 0),
            current,
            trajectories,
            std::make_shared<Point>(150, 0)
        );
        assert(res);
        // case 3: û�н��㣬Ӧ���� nullptr
        {
            auto current2 = DrawWarp::GetInstance().CreateShape<Line>(Point(0, 0), Point(200, 0), Colors::BLACK);
            std::vector<std::shared_ptr<Line>> noIntersect = {
                DrawWarp::GetInstance().CreateShape<Line>(Point(0, 100), Point(200, 100), Colors::RED)
            };
            res &= caseClosetIntersection(
                std::make_shared<Point>(50, 0),
                current2,
                noIntersect,
                nullptr
            );
            assert(res);
        }
        // �����ص�
        {
            auto current3 = DrawWarp::GetInstance().CreateShape<Line>(Point(0, 0), Point(100, 0), Colors::BLACK);
            std::vector<std::shared_ptr<Line>> trajectories = {
                DrawWarp::GetInstance().CreateShape<Line>(Point(50, 0), Point(200, 0), Colors::RED),
                DrawWarp::GetInstance().CreateShape<Line>(Point(150, -100), Point(150, 200), Colors::RED)
            };
            res &= caseClosetIntersection(
                std::make_shared<Point>(0, 0),
                current3,
                trajectories,
                std::make_shared<Point>(150, 0)
            );
            assert(res);
        }
        // ������ڱ���Ľ���Ҫ�ų�
        {
            auto current4 = DrawWarp::GetInstance().CreateShape<Line>(Point(0, 0), Point(100, 0), Colors::BLACK);
            std::vector<std::shared_ptr<Line>> trajectories = {
                DrawWarp::GetInstance().CreateShape<Line>(Point(100, 0), Point(200, 100), Colors::RED),
                DrawWarp::GetInstance().CreateShape<Line>(Point(0, 0), Point(50, 150), Colors::RED),
            };
            res &= caseClosetIntersection(
                std::make_shared<Point>(0, 0),
                current4,
                trajectories,
                std::make_shared<Point>(100, 0)
            );
            assert(res);
        }
    }
    std::cout << "All getClosetIntersection tests passed!" << std::endl;
    // ---------- getOuterNFP ���� ----------
    {
        auto start_line = DrawWarp::GetInstance().CreateShape<Line>(Point(0, 0), Point(100, 0), Colors::BLACK);
        auto end_line = DrawWarp::GetInstance().CreateShape<Line>(Point(100, 0), Point(0, 0), Colors::BLACK);
        auto l1 = DrawWarp::GetInstance().CreateShape<Line>(Point(100, 0), Point(200, 100), Colors::BLACK);
        auto l2 = DrawWarp::GetInstance().CreateShape<Line>(Point(100, 50), Point(200, 150), Colors::BLACK);
        auto l3 = DrawWarp::GetInstance().CreateShape<Line>(Point(50, 100), Point(200, 100), Colors::BLACK);
        auto l4 = DrawWarp::GetInstance().CreateShape<Line>(Point(200, 150), Point(50, 150), Colors::BLACK);
        auto l5 = DrawWarp::GetInstance().CreateShape<Line>(Point(50, 150), Point(0, 0), Colors::BLACK);
        auto l6 = DrawWarp::GetInstance().CreateShape<Line>(Point(50, 100), Point(200, 100), Colors::BLACK);

        std::vector<std::shared_ptr<Line>> trajectories = { start_line, end_line, l1, l2, l3, l4, l5, l6 };

        std::vector<std::shared_ptr<Point>> expected = {
            DrawWarp::GetInstance().CreateShape<Point>(Point(0,0)),
            DrawWarp::GetInstance().CreateShape<Point>(Point(100,0)),
            DrawWarp::GetInstance().CreateShape<Point>(Point(200,100)),
            DrawWarp::GetInstance().CreateShape<Point>(Point(150,100)),
            DrawWarp::GetInstance().CreateShape<Point>(Point(200,150)),
            DrawWarp::GetInstance().CreateShape<Point>(Point(50,150)),
            DrawWarp::GetInstance().CreateShape<Point>(Point(0,0))
        };

        res &= caseOuterNFP(start_line, start_line, trajectories, expected);
        assert(res);
    }
    // �ص��ߴ���
    {
        auto start_line = DrawWarp::GetInstance().CreateShape<Line>(Point(0, 0), Point(100, 0), Colors::BLACK);
        auto end_line = DrawWarp::GetInstance().CreateShape<Line>(Point(100, 0), Point(0, 0), Colors::BLACK);
        auto l1 = DrawWarp::GetInstance().CreateShape<Line>(Point(100, 0), Point(200, 100), Colors::BLACK);
        auto l2 = DrawWarp::GetInstance().CreateShape<Line>(Point(100, 50), Point(200, 150), Colors::BLACK);
        auto l3 = DrawWarp::GetInstance().CreateShape<Line>(Point(50, 100), Point(200, 100), Colors::BLACK);
        auto l4 = DrawWarp::GetInstance().CreateShape<Line>(Point(200, 150), Point(50, 150), Colors::BLACK);
        auto l5 = DrawWarp::GetInstance().CreateShape<Line>(Point(50, 150), Point(0, 0), Colors::BLACK);
        auto l6 = DrawWarp::GetInstance().CreateShape<Line>(Point(50, 100), Point(200, 100), Colors::BLACK);
        auto l7 = DrawWarp::GetInstance().CreateShape<Line>(Point(0, 0), Point(50, 0), Colors::BLACK);
        auto l8 = DrawWarp::GetInstance().CreateShape<Line>(Point(25, 0), Point(100, 0), Colors::BLACK);
        auto l9 = DrawWarp::GetInstance().CreateShape<Line>(Point(25, 0), Point(50, 0), Colors::BLACK);

        std::vector<std::shared_ptr<Line>> trajectories = { start_line, end_line, l1, l2, l3, l4, l5, l6, l7, l8, l9 };

        std::vector<std::shared_ptr<Point>> expected = {
            DrawWarp::GetInstance().CreateShape<Point>(Point(0,0)),
            DrawWarp::GetInstance().CreateShape<Point>(Point(100,0)),
            DrawWarp::GetInstance().CreateShape<Point>(Point(200,100)),
            DrawWarp::GetInstance().CreateShape<Point>(Point(150,100)),
            DrawWarp::GetInstance().CreateShape<Point>(Point(200,150)),
            DrawWarp::GetInstance().CreateShape<Point>(Point(50,150)),
            DrawWarp::GetInstance().CreateShape<Point>(Point(0,0))
        };

        res &= caseOuterNFP(start_line, start_line, trajectories, expected);
        assert(res);
    }
    std::cout << "All getOuterNFP tests passed!" << std::endl;

}

