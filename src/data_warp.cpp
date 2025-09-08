#include <data_warp.h>
#include <UI.h>         // for EventActivator

/* 
 * common functions:
 * CheckPointConvexity - 判断多边形顶点凹凸性
 * IsNormalInRange - 判断线的法向量是否在顶点的两法向量范围内
 * IsPointandLinePossibleContact - 判断顶点和线是否可能接触
 * getClosetIntersection - 求与当前线相交的所有线段的交点中，交点距离目标交点最近的交点
 * getMinRightAngleLine - 获取经过交点的线段中与当前线右侧夹角最小的线段
 * getOuterNFP - 获取外围NFP
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

static bool IsPointandLinePossibleContact(const Vec2& P1, const Vec2& P2, const Vec2& P3, const Vec2& Pt1, const Vec2& Pt2) {
    // 计算角的起始边矢量和终止边矢量
    Vec2 angvec1 = P1 - P2;
    Vec2 angvec2 = P3 - P2;

    // 顺时针和逆时针旋转90°得到法向量
    Vec2 Vs = angvec1.RotateClockwise90();    // 起始法向量
    Vec2 Ve = angvec2.RotateCounterClockwise90(); // 终止法向量

    // 计算边的矢量和边的法向量
    Vec2 edgevec = Pt2 - Pt1;
    Vec2 edgeNormal = edgevec.RotateClockwise90();

    // 使用叉乘判断法向量是否在范围内
    return IsNormalInRange(Vs, Ve, edgeNormal);
}

static std::shared_ptr<Point> getClosetIntersection(
    std::shared_ptr<Point> targetIntersection,
    std::shared_ptr<Line> current_line,
    std::vector<std::shared_ptr<Line>> trajectory_lines) {
    // TODO:求与current_line相交的所有线段的交点中，交点距离targetIntersection最近的交点
    
    std::shared_ptr<Point> closetIntersection = nullptr;
    float minDistance = std::numeric_limits<float>::max();
    for (auto& trajectory_line : trajectory_lines) {
        auto res = current_line->findIntersection(*trajectory_line);
        auto lineRelationship = res.first;
        auto intersection = res.second;
        // 如果相交 => 求交点
        if (lineRelationship == Line::LineRelationship::INTERSECT) {
            // 排除交点就是目标点本身的情况
            if (intersection->getPoint() == targetIntersection->getPoint()) {
                continue;
            }
            auto distance = Line(intersection->getPoint(), targetIntersection->getPoint()).getLength();
            if (distance < minDistance && abs(distance - minDistance) > EPSILON) {
                minDistance = distance;
                closetIntersection = intersection;
            }
        }
        // 部分重叠，先合并再找最近交点
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

            return getClosetIntersection(targetIntersection, merged_line, trajectory_lines);
        }
    }
    // 绘制finalIntersection点
    if (closetIntersection != nullptr) {
        static int i = 10;
        closetIntersection->setIdx(i++);
        DrawWarp::GetInstance().addShape(closetIntersection);
    }
    return closetIntersection;
}

static std::shared_ptr<Line> getMinRightAngleLine(
    std::shared_ptr<Point> intersection,
    std::shared_ptr<Line> current_line, 
    std::vector<std::shared_ptr<Line>> trajectory_lines) {
    // TODO:求与当前线右侧夹角最小的线段
    //trajectory_lines中经过交点的所有line中与current_line右侧夹角最小的线段

    float minRightAngle = std::numeric_limits<float>::max();
    float current_line_angle = current_line->getXangle();
    std::shared_ptr<Line> final_line = nullptr;
    // 遍历轨迹线中的线
    for (auto& trajectory_line : trajectory_lines) {
        // 如果当前遍历的线经过交点
        if (trajectory_line->whereIsPointOnLine(*intersection) == 0) {
            // 求该线与current_line的右侧夹角
            float trajectory_line_angle = trajectory_line->getXangle();
            float angleDiff = trajectory_line_angle - current_line_angle;
            // 将角度差规范化到[0, 2π]
            if (angleDiff <= 0) {
                angleDiff += 2 * PI;
            }
            // 选择右侧夹角最小的线段
            if (angleDiff < minRightAngle) {
                minRightAngle = angleDiff;
                final_line = trajectory_line;
            }
        }
    }
    return final_line;
}

static std::vector<std::shared_ptr<Line>> MinkowskiSumNFP(
    std::shared_ptr<Polygon> polygonA,
    std::shared_ptr<Polygon> polygonB,
    std::shared_ptr<Point> startPoint
) {
    // TODO：Minkowski和求NFP
    //size_t n = polygonA->getLines().size();
    auto reversePolygonB = DrawWarp::GetInstance().CreateShape<Polygon>(polygonB->reversePoints());
    std::vector<std::shared_ptr<Line>> AandrevBlines;
    AandrevBlines.resize(polygonA->getLines().size() + reversePolygonB->getLines().size());
    for (int i = 0; i < polygonA->getLines().size(); i++) {
        AandrevBlines[i] = DrawWarp::GetInstance().CreateShape<Line>(*(polygonA->getLines()[i]));
        std::ostringstream oss;
        oss << "A" << i + 1;
        AandrevBlines[i]->setComeFrom(oss.str());
    }
    for (int i = polygonA->getLines().size(); i < AandrevBlines.size(); i++) {
        AandrevBlines[i] = DrawWarp::GetInstance().CreateShape<Line>(*(polygonB->getLines()[i - polygonA->getLines().size()]));
        std::ostringstream oss;
        oss << "B" << i - polygonA->getLines().size() + 1;
        AandrevBlines[i]->setComeFrom(oss.str());
    }
    std::stable_sort(AandrevBlines.begin(), AandrevBlines.end(), [](auto line1, auto line2) {
        return (line1->getEndPoint() - line1->getStartPoint()).angle() < (line2->getEndPoint() - line2->getStartPoint()).angle();
    });
    std::vector<std::shared_ptr<Line>> res;
    auto current_vec = AandrevBlines[0]->getStartPoint();
    res.push_back(AandrevBlines[0]);
    for (int i = 1; i < AandrevBlines.size() - 1; i++) {
        auto line = DrawWarp::GetInstance().CreateShape<Line>(AandrevBlines[i - 1]->getEndPoint()
            , AandrevBlines[i - 1]->getEndPoint() + AandrevBlines[i]->getEndPoint() - AandrevBlines[i]->getStartPoint());
        line->setComeFrom(AandrevBlines[i]->getComeFrom());
        res.push_back(line);
    }
    return res;
}

/*
 * start_line - 起始提取位置
 * end_line - 终止提取位置 （只有提取局部轮廓NFP才会用到这个参数，其他算法默认end_line=start_line）
 */
static std::vector<std::shared_ptr<Point>> getOuterNFP(
    std::shared_ptr<Line> start_line, 
    std::shared_ptr<Line> end_line,
    std::vector<std::shared_ptr<Line>> trajectory_lines) {
    // TODO:提取NFP外围线
     
    // 初始化当前线段为start_line
    auto current_line = start_line;
    // 初始化交点为start_line的起点
    auto closetIntersection = DrawWarp::GetInstance().CreateShape<Point>(start_line->getStartPoint());
    std::vector<std::shared_ptr<Point>> finalNFP;
    
    // 将start_line的起点加入NFP
    finalNFP.push_back(DrawWarp::GetInstance().CreateShape<Point>(start_line->getStartPoint()));
    
    do {
        // 查找与current_line相交的所有线段的交点中，交点距离intersection最近的交点
        closetIntersection = getClosetIntersection(closetIntersection, current_line, trajectory_lines);
        
        // 更新current_line为trajectory_lines中经过交点的所有line中与current_line右侧夹角最小的线段
        current_line = getMinRightAngleLine(closetIntersection, current_line, trajectory_lines);
        
        // 将该最近交点加入NFP
        finalNFP.push_back(closetIntersection);
    } while (closetIntersection->getPoint() != end_line->getEndPoint());
   
    return finalNFP;
}


// ===== test ======
void xdn_test::apply()  {
    auto a = DrawWarp::GetInstance().CreateShape<Line>(200, 0, 300, 0, Colors::BLACK);
    auto b = DrawWarp::GetInstance().CreateShape<Line>(0, 0, 200, 0, Colors::RED);
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



void LocalContourNFPAlgorithm::apply(){
    // TODO: 2024的算法
    // step1：多边形顶点凹凸性分析
    // step2：凸化算法及辅助边构造
    // step3：生成双凸多边形的NFP
    // step4：根据辅助边生成两个局部轮廓
    // step5：点线接触判断
    // step6：轨迹线集合生成算法
    // step7：外围NFP提取算法
    // step8：NFP合并算法
}

// ===== Algorithm3:TwoLocalContour 2024o =====
TwoLocalContourNFPAlgorithm::TwoLocalContourNFPAlgorithm(std::vector<std::shared_ptr<Shape>> polygon_data) {
    this->polygon_data = polygon_data;
}
void TwoLocalContourNFPAlgorithm::apply() {
    // TODO: 第三章的算法

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
        /*bool testCheckPointConvexity(std::shared_ptr<Polygon> polygon, std::vector<int> expectedConvexity) {
            auto res = DrawWarp::GetInstance().CreateShape<ConvexityPolygon>(polygon->getPoints());

            if (res->getConvexityPoints().size() != expectedConvexity.size()) {
                return false;
            }

            for (size_t i = 0; i < res->getConvexityPoints().size(); ++i) {
                if (res[i] != expectedConvexity[i]) {
                    return false;
                }
            }
            return true;
        }*/
        struct SimplePolygon {
            std::shared_ptr<Polygon> poly;
            SimplePolygon(std::initializer_list<Vec2> points) {
                std::vector<std::shared_ptr<Point>> pts;
                for (auto& p : points) {
                    pts.push_back(DrawWarp::GetInstance().CreateShape<Point>(p));
                }
                poly = DrawWarp::GetInstance().CreateShape<Polygon>(pts);
            }
        };
        bool testMinkowskiSumNFP(std::shared_ptr<Polygon> _polygonA, std::shared_ptr<Polygon> _polygonB, std::shared_ptr<Point> startPoint, std::vector<std::shared_ptr<Line>> expectedRes) {
            auto result = MinkowskiSumNFP(_polygonA, _polygonB, startPoint);

            if (result.size() != expectedRes.size()) {
                return false;
            }

            for (size_t i = 0; i < result.size(); ++i) {
                Vec2 r = result[i]->getStartPoint();
                Vec2 e = expectedRes[i]->getStartPoint();

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
    /*bool caseCheckPointConvexity(std::shared_ptr<Polygon> polygon, std::vector<int> expectedConvexity) {
        return helper::testCheckPointConvexity(polygon, expectedConvexity);
    }*/
    bool caseMinkowskiSumNFP(
        std::shared_ptr<Polygon> polygonA,
        std::shared_ptr<Polygon> polygonB,
        std::shared_ptr<Point> startPoint,
        std::vector<std::shared_ptr<Line>> expectedRes
    ) {
        return helper::testMinkowskiSumNFP(polygonA, polygonB, startPoint, expectedRes);
    }

}

void TestCases::apply() {
    using namespace Case;
    bool res = true;
    {
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

    }
    std::cout << "All LineIntersect tests passed!" << std::endl;
    // ---------- getMinRightAngleLine 测试 ----------
    // case 1: 没有交点 → 结果应为 nullptr
    {
        res &= caseRightAngle(
            helper::CurrentAndTrajectories(
                Line(Point(0, 0), Point(200, 0)),
                { Line(Point(0, 100), Point(200, 100)) } // 平行，不相交
            ),
            std::make_shared<Point>(100, 0),  // 给一个交点，但 trajectory 不经过
            nullptr
        );
        assert(res);
    }
    // case 2: 单一相交 → 应该选唯一那条线
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
    // case 3: 多个相交 → 选择经过交点的
    {
        auto expected = DrawWarp::GetInstance().CreateShape<Line>(Point(100, 0), Point(150, 50), Colors::RED); // 垂直，夹角小
        res &= caseRightAngle(
            helper::CurrentAndTrajectories(
                Line(Point(0, 0), Point(200, 0)),
                {
                    Line(Point(100, 0), Point(150, 50)),  // 右上，夹角大
                    Line(Point(50, -50), Point(50, 50))   // 垂直，夹角小
                }
            ),
            std::make_shared<Point>(100, 0),
            expected
        );
        assert(res);
    }
    // case 4: 多个经过相交的线 → 选择右侧夹角最小的
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
    // ---------- getClosetIntersection 测试 ----------
    {
        auto current = DrawWarp::GetInstance().CreateShape<Line>(Point(0, 0), Point(200, 0), Colors::BLACK);
        std::vector<std::shared_ptr<Line>> trajectories = {
            DrawWarp::GetInstance().CreateShape<Line>(Point(100, -100), Point(100, 100), Colors::RED),
            DrawWarp::GetInstance().CreateShape<Line>(Point(150, -100), Point(150, 100), Colors::RED)
        };
        // case 1: 最近的交点是 (100,0)
        res &= caseClosetIntersection(
            std::make_shared<Point>(0, 0),
            current,
            trajectories,
            std::make_shared<Point>(100, 0)
        );
        assert(res);
        // case 2: 目标靠近 (150,0)，所以应选 (150,0)
        res &= caseClosetIntersection(
            std::make_shared<Point>(140, 0),
            current,
            trajectories,
            std::make_shared<Point>(150, 0)
        );
        assert(res);
        // case 3: 没有交点，应返回 nullptr
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
        // 部分重叠
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
        // 交点等于本身的交点要排除
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
    // ---------- getOuterNFP 测试 ----------
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

        res &= caseOuterNFP(start_line, end_line, trajectories, expected);
        assert(res);
    }
    // 重叠线处理
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

        res &= caseOuterNFP(start_line, end_line, trajectories, expected);
        assert(res);
    }
    std::cout << "All getOuterNFP tests passed!" << std::endl;
    // ---------- CheckPointConvexity 测试 ----------
    //{
    //    // 凸三角形：所有点都是凸点
    //    auto p1 = DrawWarp::GetInstance().CreateShape<Point>(Point(0, 0));
    //    auto p2 = DrawWarp::GetInstance().CreateShape<Point>(Point(1, 0));
    //    auto p3 = DrawWarp::GetInstance().CreateShape<Point>(Point(0, 1));
    //    std::vector<std::shared_ptr<Point>> pts1 = { p1, p2, p3 };
    //    auto polygon1 = DrawWarp::GetInstance().CreateShape<Polygon>(pts1);

    //    std::vector<int> expected1 = { 0, 0, 0 };
    //    res &= caseCheckPointConvexity({ polygon1 }, expected1);
    //    assert(res);
    //}
    //{
    //    // 凹多边形
    //    auto p1 = DrawWarp::GetInstance().CreateShape<Point>(Point(0, 0));
    //    auto p2 = DrawWarp::GetInstance().CreateShape<Point>(Point(3, 0));
    //    auto p3 = DrawWarp::GetInstance().CreateShape<Point>(Point(2, 1));  // 凹点
    //    auto p4 = DrawWarp::GetInstance().CreateShape<Point>(Point(3, 2));
    //    auto p5 = DrawWarp::GetInstance().CreateShape<Point>(Point(0, 2));
    //    std::vector<std::shared_ptr<Point>> pts2 = { p1, p2, p3, p4, p5 };
    //    auto polygon2 = DrawWarp::GetInstance().CreateShape<Polygon>(pts2);

    //    std::cout << "Stored points order:" << std::endl;
    //    for (int i = 0; i < polygon2->getPoints().size(); i++) {
    //        std::cout << "P" << i << ": (" << polygon2->getPoints()[i]->getPoint().x
    //            << ", " << polygon2->getPoints()[i]->getPoint().y << ")" << std::endl;
    //    }

    //    std::vector<int> expected2 = { 0, 0, 1, 0, 0 };
    //    res &= caseCheckPointConvexity({ polygon2 }, expected2);
    //    assert(res);
    //}
    //{
    //    // 含共线点：所有点都视为凸点
    //    auto p1 = DrawWarp::GetInstance().CreateShape<Point>(Point(0, 0));
    //    auto p2 = DrawWarp::GetInstance().CreateShape<Point>(Point(1, 0));
    //    auto p3 = DrawWarp::GetInstance().CreateShape<Point>(Point(2, 0)); // 共线
    //    auto p4 = DrawWarp::GetInstance().CreateShape<Point>(Point(1, 1));
    //    std::vector<std::shared_ptr<Point>> pts3 = { p1, p2, p3, p4 };
    //    auto polygon3 = DrawWarp::GetInstance().CreateShape<Polygon>(pts3);

    //    std::vector<int> expected3 = { 0, 0, 0, 0 };
    //    res &= caseCheckPointConvexity({ polygon3 }, expected3);
    //    assert(res);
    //}
    std::cout << "All CheckPointConvexity tests passed!" << std::endl;
    std::cout << "---------- MinkowskiSumNFP 测试 ----------" << std::endl;
    {
        // A：一个三角形 (0,0)-(2,0)-(2,2)
        std::vector<std::shared_ptr<Point>> ptsA = {
            DrawWarp::GetInstance().CreateShape<Point>(Point(0,0)),
            DrawWarp::GetInstance().CreateShape<Point>(Point(2,0)),
            DrawWarp::GetInstance().CreateShape<Point>(Point(2,2))
        };
        auto polyA = DrawWarp::GetInstance().CreateShape<Polygon>(ptsA);

        // B：一个三角形 (0,0)-(1,0)-(1,1)
        std::vector<std::shared_ptr<Point>> ptsB = {
            DrawWarp::GetInstance().CreateShape<Point>(Point(0,0)),
            DrawWarp::GetInstance().CreateShape<Point>(Point(1,0)),
            DrawWarp::GetInstance().CreateShape<Point>(Point(1,1))
        };
        auto polyB = DrawWarp::GetInstance().CreateShape<Polygon>(ptsB);

        // 起始点 (0,0)
        auto start = DrawWarp::GetInstance().CreateShape<Point>(Point(0, 0));

        // 预期结果
        std::vector<std::shared_ptr<Line>> expected = {
            DrawWarp::GetInstance().CreateShape<Line>(Point(0,0), Point(3,0)),
            DrawWarp::GetInstance().CreateShape<Line>(Point(3,0), Point(3,3)),
            DrawWarp::GetInstance().CreateShape<Line>(Point(3,3), Point(0,3)),
            DrawWarp::GetInstance().CreateShape<Line>(Point(0,3), Point(0,0))
        };

        res &= caseMinkowskiSumNFP(polyA, polyB, start, expected);
        assert(res);
    }
    std::cout << "All MinkowskiSumNFP tests passed!" << std::endl;

}

