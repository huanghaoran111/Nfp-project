#include <data_warp.h>
#include <UI.h>         // for EventActivator
#include <array>
#include <chrono>
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
namespace NFP{
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

static std::shared_ptr<NFP::Point> getClosetIntersection(
    std::shared_ptr<NFP::Point> targetIntersection,
    std::shared_ptr<NFP::Line> current_line,
    std::vector<std::shared_ptr<NFP::Line>> trajectory_lines) {
    // TODO:求与current_line相交的所有线段的交点中，交点距离targetIntersection最近的交点
    std::cout << "new ClosetIntersection \n    targetIntersection: " 
        << targetIntersection->getPoint().x << ", " << targetIntersection->getPoint().y 
        << "\n    current_line: " << current_line->getStartPoint().x << ", " << current_line->getStartPoint().y << " -> " << current_line->getEndPoint().x << ", " << current_line->getEndPoint().y
        << std::endl;
    std::shared_ptr<Point> closetIntersection = nullptr;
    float minDistance = std::numeric_limits<float>::max();
    for (auto& trajectory_line : trajectory_lines) {
        auto res = current_line->findIntersection(*trajectory_line);
        auto lineRelationship = res.first;
        auto intersection = res.second;
        std::cout << "new check Line: " << trajectory_line->getStartPoint().x << ", " << trajectory_line->getStartPoint().y << " -> " << trajectory_line->getEndPoint().x << ", " << trajectory_line->getEndPoint().y << std::endl;
        // 如果相交 => 求交点
        if (lineRelationship == NFP::Line::LineRelationship::INTERSECT) {
            // 排除交点就是目标点本身的情况
            if (intersection->getPoint() == targetIntersection->getPoint()) {
                continue;
            }
            std::cout << "Line INTERSECT" << std::endl;
            auto distance = Line(intersection->getPoint(), targetIntersection->getPoint()).getLength();
            if (distance < minDistance && abs(distance - minDistance) > EPSILON && distance > EPSILON) {
                minDistance = distance;
                closetIntersection = intersection;
                std::cout << "closetIntersection reset" << std::endl;
            }
            else {
                std::cout << "closetIntersection not reset" << std::endl;
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
            std::cout << "Line PARTOVERLAP" << std::endl;
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
    if(closetIntersection == nullptr)
        assert(closetIntersection != nullptr);
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
        if (trajectory_line->whereIsPointOnLineSegment(*intersection) == 0) {
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
    /*if (intersection->getPoint() == final_line->getEndPoint()) {
        assert(intersection->getPoint() != final_line->getEndPoint());
    }*/
    return DrawWarp::GetInstance().CreateShape<Line>(intersection->getPoint(), final_line->getEndPoint());
}

static std::vector<std::shared_ptr<Line>> MinkowskiSumNFP(
    std::shared_ptr<Polygon> polygonA,
    std::shared_ptr<Polygon> polygonB,
    std::shared_ptr<Point> startPoint
) {
    // TODO：Minkowski和求NFP
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
        AandrevBlines[i] = DrawWarp::GetInstance().CreateShape<Line>(*(reversePolygonB->getLines()[i - polygonA->getLines().size()]));
        std::ostringstream oss;
        oss << "B" << i - polygonA->getLines().size() + 1;
        AandrevBlines[i]->setComeFrom(oss.str());
    }
    std::stable_sort(AandrevBlines.begin(), AandrevBlines.end(), [](auto line1, auto line2) {
        return (line1->getEndPoint() - line1->getStartPoint()).angle() < (line2->getEndPoint() - line2->getStartPoint()).angle();
    });

    std::vector<std::shared_ptr<Line>> res;
    // 设置起始点：如果提供了startPoint则使用，否则使用原点(0,0)
    auto currentVec = startPoint ? startPoint->getPoint() : Vec2(0, 0);
    //auto current_vec = AandrevBlines[0]->getStartPoint();
    //res.push_back(AandrevBlines[0]);
    for (int i = 0; i < AandrevBlines.size(); i++) {
        // 获取当前边的起点和终点坐标
        Vec2 lineStart = AandrevBlines[i]->getStartPoint();
        Vec2 lineEnd = AandrevBlines[i]->getEndPoint();

        // 计算向量分量
        float dx = lineEnd.x - lineStart.x;
        float dy = lineEnd.y - lineStart.y;

        // 计算下一个点坐标
        Vec2 nextVec(currentVec.x + dx, currentVec.y + dy);

        //auto line = DrawWarp::GetInstance().CreateShape<Line>(AandrevBlines[i - 1]->getEndPoint()
            //, AandrevBlines[i - 1]->getEndPoint() + AandrevBlines[i]->getEndPoint() - AandrevBlines[i]->getStartPoint());
        // 创建新的线段
        auto line = DrawWarp::GetInstance().CreateShape<Line>(currentVec, nextVec);
        line->setComeFrom(AandrevBlines[i]->getComeFrom());
        res.push_back(line);
        // 更新当前点为下一个点
        currentVec = nextVec;
    }
    return res;
}

static std::shared_ptr<Point> FindPolygonCenter(std::shared_ptr<Polygon> polygon) {
    auto points = polygon->getPoints();
    // 初始化为多边形第一个点的坐标
    auto minX = points[0]->getPoint().x, maxX = points[0]->getPoint().x;
    auto minY = points[0]->getPoint().y, maxY = points[0]->getPoint().y;

    // 遍历多边形的所有顶点，找出最大值和最小值
    for (auto& point : points) {
        if (point->getPoint().x < minX) minX = point->getPoint().x;
        if (point->getPoint().x > maxX) maxX = point->getPoint().x;
        if (point->getPoint().y < minY) minY = point->getPoint().y;
        if (point->getPoint().y > maxY) maxY = point->getPoint().y;
    }

    // 计算中心点
    float centerX = (minX + maxX) / 2.0f;
    float centerY = (minY + maxY) / 2.0f;

    auto res = DrawWarp::GetInstance().CreateShape<Point>(centerX, centerY);

    return res;
}


//static std::vector<std::shared_ptr<Line>> GenerateTrajectoryLines(
//    std::shared_ptr<Polygon> polygonA,
//    std::shared_ptr<Polygon> polygonB,
//    std::shared_ptr<Point> PrefB) {
//
//    std::vector<std::shared_ptr<Line>> res;
//
//    // 找到多边形 A 的中心点
//    auto centerA = FindPolygonCenter(polygonA);
//
//    //参考点不会变，参考点的坐标会改变
//    // 找到多边形B的最低点作为参考点
//    //auto PrefB = polygonB->getPoints()[0];
//    auto refB = PrefB->getPoint();
//
//    // A是固定多边形
//    auto nB = polygonB->getPoints().size();
//    for (size_t i = 0; i < nB; ++i) {
//        Vec2 P1 = polygonB->getPoints()[(i + nB - 1) % nB]->getPoint();
//        Vec2 P2 = polygonB->getPoints()[i]->getPoint();
//        Vec2 P3 = polygonB->getPoints()[(i + 1) % nB]->getPoint();
//        // 遍历 A 的所有边
//        for (auto edgeA : polygonA->getLines()) {
//            // 计算A边的法向量是否在角度范围内
//            if (!IsPointandLinePossibleContact(P1, P2, P3, edgeA->getStartPoint(), edgeA->getEndPoint())) continue;
//
//            // 如果在角度范围内，则生成轨迹线
//            refB = PrefB->getPoint() - P2 + edgeA->getStartPoint();
//            auto T_ij = refB + edgeA->getEndPoint() - edgeA->getStartPoint();
//
//            res.push_back(DrawWarp::GetInstance().CreateShape<Line>(refB, T_ij));
//        }
//    }
//    return res;
//}



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
    
    Vec2 endPos;
    if (end_line != nullptr) {
        endPos = end_line->getEndPoint();
    }
    else {
        endPos = finalNFP[0]->getPoint();
    }

    do {
        // 查找与current_line相交的所有线段的交点中，交点距离intersection最近的交点
        closetIntersection = getClosetIntersection(closetIntersection, current_line, trajectory_lines);
        
        // 更新current_line为trajectory_lines中经过交点的所有line中与current_line右侧夹角最小的线段
        current_line = getMinRightAngleLine(closetIntersection, current_line, trajectory_lines);
        // 将该最近交点加入NFP
        finalNFP.push_back(closetIntersection);
    } while (closetIntersection->getPoint() != endPos);
    return finalNFP;
}

static std::shared_ptr<Point> FlipBoth(std::shared_ptr<Point> point) {
    auto res = DrawWarp::GetInstance().CreateShape<Point>(-point->getPoint().x, -point->getPoint().y);
    return res;
}

// ===== test ======
void xdn_test::apply()  {
    auto a = DrawWarp::GetInstance().CreateShape<Line>(200, 0, 300, 0, Colors::BLACK);
    auto b = DrawWarp::GetInstance().CreateShape<Line>(0, 0, 200, 0, Colors::RED);
}


// 移动碰撞法
void MovingCollisionNFPAlgorithm::apply() {
    std::vector<NFP::Point> res = MoveNFPFunc(this->polygon_data);
    for (int i = 1; i < res.size(); i++) {
        DWCreateShape<Line>(res[i - 1], res[i]);
    }
}
MovingCollisionNFPAlgorithm::MovingCollisionNFPAlgorithm(std::vector<std::vector<std::shared_ptr<Point>>> polygon_data){
    this->polygon_data = polygon_data;
}

TrajectoryNFPAlgorithm::TrajectoryNFPAlgorithm(std::vector<std::vector<std::shared_ptr<Point>>> polygon_data) {
    this->polygon_data = polygon_data;
}

static std::shared_ptr<Line> FindStartLine(std::vector<std::shared_ptr<Line>> TrajectoryLines) {
    // 找到 Y 坐标最小的线段的起点
    float minY = std::numeric_limits<float>::max();
    for (auto line : TrajectoryLines) {
        if (line->getStartPoint().y < minY) {
            minY = line->getStartPoint().y;
        }
    }
    // 找到所有起点等于最小Y值的线段
    std::vector<std::shared_ptr<Line>> minYLines;
    for (auto line : TrajectoryLines) {
        float currentY = line->getStartPoint().y;
        if (std::abs(currentY - minY) < EPSILON) {
            minYLines.push_back(line);
        }
    }
    // 从这些线段中选择与X轴夹角最小的线段作为起始线段
    std::shared_ptr<Line> startLine = minYLines[0];
    float minAngle = std::numeric_limits<float>::max();
    for (auto line : minYLines) {
        Vec2 start = line->getStartPoint();
        Vec2 end = line->getEndPoint();
        float angle = atan2(end.y - start.y, end.x - start.x);

        // 确保角度在[0, 2π)范围内
        if (angle < 0) {
            angle += 2 * PI;
        }

        if (angle < minAngle) {
            minAngle = angle;
            startLine = line;
        }
    }
    return startLine;
}

// ...
void TrajectoryNFPAlgorithm::apply(){
    auto polygonA = std::make_shared<Polygon>(polygon_data[0]);
    auto polygonB = std::make_shared<Polygon>(polygon_data[1]);
    auto start = std::chrono::high_resolution_clock::now();
    auto trajectoryLines = GenerateTrajectoryLinesSet(polygonA, polygonB);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "GenerateTrajectoryLinesSet use " << duration.count() << " microseconds" << std::endl;
    
    auto startLine = FindStartLine(trajectoryLines);
    auto finalNFP = getOuterNFP(startLine, nullptr, trajectoryLines);
    //DrawWarp::GetInstance().clearShapes();
   /* for (int i = 0; i < finalNFP.size(); i++) {
        DWCreateShape<Line>(finalNFP[i]->getPoint(), finalNFP[(i+1) % finalNFP.size()]->getPoint());
    }*/
}


std::vector<std::shared_ptr<Line>> TrajectoryNFPAlgorithm::GenerateTrajectoryLinesSet(
    std::shared_ptr<Polygon> polygonA, std::shared_ptr<Polygon> polygonB) {

    std::vector<std::shared_ptr<Line>> trajectoryLinesA;
    std::vector<std::shared_ptr<Line>> trajectoryLinesB;
    std::vector<std::shared_ptr<Line>> finalTrajectoryLines;

    // 找到多边形 A 的中心点
    auto centerA = FindPolygonCenter(polygonA)->getPoint();

    //参考点不会变，参考点的坐标会改变
    // 找到多边形B的最低点作为参考点
    auto PrefB = polygonB->getPoints()[0];
    auto refB = PrefB->getPoint();
    // 找到多边形A的最低点作为参考点 
    auto PrefA = polygonA->getPoints()[0];
    auto refA = PrefA->getPoint();

    Vec2 PrefA_R;
    PrefA_R.x = 2 * centerA.x - PrefA->getPoint().x; // 根据中心点水平翻转
    PrefA_R.y = 2 * centerA.y - PrefA->getPoint().y; // 根据中心点垂直翻转

    Vec2 T_ij;
    // A不动
    auto nB = polygonB->getPoints().size();
    for (size_t i = 0; i < nB; ++i) {
        Vec2 P1 = polygonB->getPoints()[(i + nB - 1) % nB]->getPoint();
        Vec2 P2 = polygonB->getPoints()[i]->getPoint();
        Vec2 P3 = polygonB->getPoints()[(i + 1) % nB]->getPoint();
        // 遍历 A 的所有边
        for (auto edgeA : polygonA->getLines()) {
            // 计算A边的法向量是否在角度范围内
            if (!IsPointandLinePossibleContact(P1, P2, P3, edgeA->getStartPoint(), edgeA->getEndPoint())) continue;

            // 如果在角度范围内，则生成轨迹线
            refB = PrefB->getPoint() - P2 + edgeA->getStartPoint();
            T_ij = refB + edgeA->getEndPoint() - edgeA->getStartPoint();

            trajectoryLinesB.push_back(DrawWarp::GetInstance().CreateShape<Line>(refB, T_ij));
        }
    }
    // B不动
    size_t nA = polygonA->getPoints().size();
    for (size_t i = 0; i < nA; ++i) {
        Vec2 P1 = polygonA->getPoints()[(i + nA - 1) % nA]->getPoint();
        Vec2 P2 = polygonA->getPoints()[i]->getPoint();
        Vec2 P3 = polygonA->getPoints()[(i + 1) % nA]->getPoint();
        // 遍历 B 的所有边
        for (auto edgeB : polygonB->getLines()) {
            // 计算B边的法向量是否在角度范围内
            if (!IsPointandLinePossibleContact(P1, P2, P3, edgeB->getStartPoint(), edgeB->getEndPoint())) continue;
            // 如果在角度范围内，则生成轨迹线
            refA = PrefA->getPoint() - P2 + edgeB->getStartPoint();
            T_ij = refA + edgeB->getEndPoint() - edgeB->getStartPoint();
            trajectoryLinesA.push_back(DrawWarp::GetInstance().CreateShape<Line>(refA, T_ij));
        }
    }
    // 将trajectoryLinesB以centerA为中心水平垂直翻转
    for (auto line : trajectoryLinesB) {
        // 对称翻转轨迹线，以CenterA为中心
        auto flippedStart = FlipBoth(DrawWarp::GetInstance().CreateShape<Point>(line->getStartPoint() - centerA))->getPoint() + centerA;
        auto flippedEnd = FlipBoth(DrawWarp::GetInstance().CreateShape<Point>(line->getEndPoint() - centerA))->getPoint() + centerA;

        // 计算偏移量
        Vec2 offset;
        offset.x = PrefB->getPoint().x - PrefA_R.x;
        offset.y = PrefB->getPoint().y - PrefA_R.y;

        // 应用偏移量
        auto alignedStart = flippedStart + offset;
        auto alignedEnd = flippedEnd + offset;
        finalTrajectoryLines.push_back(DrawWarp::GetInstance().CreateShape<Line>(alignedStart, alignedEnd));
    }
    finalTrajectoryLines.insert(finalTrajectoryLines.end(), trajectoryLinesA.begin(), trajectoryLinesA.end());
    if(EventActivator::GetInstance().HasEvent("ShowTrajectoryLines")){
        for(auto line : finalTrajectoryLines){
            DrawWarp::GetInstance().addShape<Line>(line);
            DrawWarp::GetInstance().addShape<Point>(DrawWarp::GetInstance().CreateShape<Point>(line->getStartPoint()));
            DrawWarp::GetInstance().addShape<Point>(DrawWarp::GetInstance().CreateShape<Point>(line->getEndPoint()));
        }
    }
    //DrawWarp::GetInstance().clearShapes();

    for (int i = 0; i < finalTrajectoryLines.size(); i++) {
        Vec2 startPoint = finalTrajectoryLines[i]->getStartPoint();
        Vec2 endPoint = finalTrajectoryLines[i]->getEndPoint();
        DWCreateShape<Line>(startPoint, endPoint);
    }

    return finalTrajectoryLines;
}

// ===== Algorithm2:LocalContour 2024 =====
LocalContourNFPAlgorithm::LocalContourNFPAlgorithm(std::vector<std::vector<std::shared_ptr<Point>>> polygon_data){
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
TwoLocalContourNFPAlgorithm::TwoLocalContourNFPAlgorithm(std::vector<std::vector<std::shared_ptr<Point>>> polygon_data) {
    this->polygon_data = polygon_data;
}
void TwoLocalContourNFPAlgorithm::apply() {
    // TODO: 第三章的算法

}


static std::vector<std::shared_ptr<Line>> GenerateTrajectoryLinesSet(
    std::shared_ptr<Polygon> LocalContourA,
    std::shared_ptr<Polygon> LocalContourB,
    std::shared_ptr<Point> PrefA,
    std::shared_ptr<Point> LocalContourPrefA) {

    // 要点1：
    // 由于变成局部轮廓+局部轮廓的形式，以最低点为参考点的轨迹线位置可能会发生变化
    // 导致局部轮廓轨迹线拼接时出现问题
    // 因此 输入原始多边形的参考点 计算局部轮廓的参考点
    // 如果参考点与原多边形的不一致
    // 则偏移参考点的量
    // 由于是B对齐到A 所以当B偏移的时候才需要加上参考点不同时导致的偏移量
    // 要点2：
    // 当局部轮廓的宽度与原多边形的宽度不一致时，
    // 目前出现的情况是：
    // demo3：最左边的点不一致时，Lend偏移量为局部轮廓和最左边的点之间的偏移量

    std::vector<std::shared_ptr<Line>> trajectoryLinesArB;  // 保存多边形 A 的轨迹线
    std::vector<std::shared_ptr<Line>> trajectoryLinesBrA;  // 保存多边形 B 的轨迹线
    std::vector<std::shared_ptr<Line>> finalTrajectoryLines;  // 保存最终轨迹线

    //参考点不会变，参考点的坐标会改变
    // 找到多边形B的最低点作为参考点
    auto LocalContourPrefB = LocalContourB->getPoints()[0];
    auto refB = LocalContourPrefB->getPoint();
    // 找到多边形A的最低点作为参考点 
    auto refA = LocalContourPrefA->getPoint();

    // 当参考点发生变化时 计算此时参考点的偏移量
    auto LocalContourOffset = PrefA->getPoint() - LocalContourPrefA->getPoint();

    // 找到多边形 A 的中心点
    auto centerA = FindPolygonCenter(LocalContourA)->getPoint() + LocalContourOffset;

    Vec2 PrefA_R;
    PrefA_R.x = 2 * centerA.x - LocalContourPrefA->getPoint().x; // 根据中心点水平翻转
    PrefA_R.y = 2 * centerA.y - LocalContourPrefA->getPoint().y; // 根据中心点垂直翻转

    Vec2 T_ij;
    // A不动 B绕A
    auto nB = LocalContourB->getPoints().size();
    for (size_t i = 0; i < nB; ++i) {
        Vec2 P1 = LocalContourB->getPoints()[(i + nB - 1) % nB]->getPoint();
        Vec2 P2 = LocalContourB->getPoints()[i]->getPoint();
        Vec2 P3 = LocalContourB->getPoints()[(i + 1) % nB]->getPoint();
        // 遍历 A 的所有边
        for (auto edgeA : LocalContourA->getLines()) {
            // 计算A边的法向量是否在角度范围内
            if (!IsPointandLinePossibleContact(P1, P2, P3, edgeA->getStartPoint(), edgeA->getEndPoint())) continue;
            // 如果在角度范围内，则生成轨迹线
            // 起点坐标
            refB = LocalContourPrefB->getPoint() - P2 + edgeA->getStartPoint() + LocalContourOffset;
            // 终点坐标
            T_ij = refB + edgeA->getEndPoint() - edgeA->getStartPoint();

            trajectoryLinesBrA.push_back(DrawWarp::GetInstance().CreateShape<Line>(refB, T_ij));

        }
    }
    // B不动 A绕B
    auto nA = LocalContourA->getPoints().size();
    for (size_t i = 0; i < nA; ++i) {
        Vec2 P1 = LocalContourA->getPoints()[(i + nA - 1) % nA]->getPoint();
        Vec2 P2 = LocalContourA->getPoints()[i]->getPoint();
        Vec2 P3 = LocalContourA->getPoints()[(i + 1) % nA]->getPoint();
        // 遍历 B 的所有边
        for (auto edgeB : LocalContourB->getLines()) {
            // 计算B边的法向量是否在角度范围内
            if (!IsPointandLinePossibleContact(P1, P2, P3, edgeB->getStartPoint(), edgeB->getEndPoint())) continue;

            // 如果在角度范围内，则生成轨迹线
            refA = PrefA->getPoint() - P2 + edgeB->getEndPoint();
            T_ij = refA + edgeB->getEndPoint() - edgeB->getStartPoint();

            trajectoryLinesArB.push_back(DrawWarp::GetInstance().CreateShape<Line>(refA, T_ij));
        }
    }
    // 将trajectoryLinesB以centerA为中心水平垂直翻转
    for (auto line : trajectoryLinesBrA) {
        // 对称翻转轨迹线，以CenterA为中心
        // 这里不知道为什么centerA已经加过偏移量还需要加
        auto flippedStart = FlipBoth(DrawWarp::GetInstance().CreateShape<Point>(line->getStartPoint() - centerA))->getPoint() + centerA + LocalContourOffset;
        auto flippedEnd = FlipBoth(DrawWarp::GetInstance().CreateShape<Point>(line->getEndPoint() - centerA))->getPoint() + centerA + LocalContourOffset;

        // 计算偏移量
        Vec2 offset;
        // 这里也没思考为什么要加偏移量
        offset.x = LocalContourPrefB->getPoint().x - PrefA_R.x + LocalContourOffset.x;
        offset.y = LocalContourPrefB->getPoint().y - PrefA_R.y + LocalContourOffset.y;

        // 应用偏移量
        auto alignedStart = flippedStart + offset;
        auto alignedEnd = flippedEnd + offset;

        // 保存更新后的轨迹线，交换轨迹线起点和终点，改变轨迹线方向
        finalTrajectoryLines.push_back(DrawWarp::GetInstance().CreateShape<Line>(alignedStart, alignedEnd));
     }

    // Step 5: 将多边形A绕B的轨迹线原样加入到最终轨迹线
    finalTrajectoryLines.insert(finalTrajectoryLines.end(), trajectoryLinesArB.begin(), trajectoryLinesArB.end());

    return finalTrajectoryLines;
}


MinkowskiSumNFPAlgorithm::MinkowskiSumNFPAlgorithm(std::vector<std::vector<std::shared_ptr<Point>>> polygon_data){
    this->polygon_data = polygon_data;
}

void MinkowskiSumNFPAlgorithm::apply() {
    // TODO: 算法实现
}

// 基于三角剖分与边界合法性判定的临界多边形算法
// ===== Algorithm4: DelaunayTriangulationNFP =====
// 完全凸边提取
static bool isExtractConvexHullEdges(
    std::shared_ptr<Line> line,
    std::shared_ptr<Polygon> polygonA,
    std::shared_ptr<Polygon> polygonB
) {
    

    return true;
}

DelaunayTriangulationNFPAlgorithm::DelaunayTriangulationNFPAlgorithm(std::vector<std::vector<std::shared_ptr<Point>>> polygon_data) {
    this->polygon_data = polygon_data;
}

class TriPoint{
public:
    TriPoint(NFP::Point p)
        : m_point(p.getPoint()){}
    Vec2 getStartPos(Vec2 originPos){
        return m_point.getPoint() + this->StartPos;
    }
    void setStartPos(Vec2 originPos){
        this->StartPos = originPos - this->m_point.getPoint();
    }
    NFP::Point m_point;
    TriPoint& operator=(const TriPoint& t){
        m_point = t.m_point;
        this->StartPos = t.StartPos;
        return *this;
    }
    ~TriPoint() = default;
private:
    // StartPos是一个向量
    Vec2 StartPos;
};

class TriLine{
public:
    enum class LineAttr{
        CONVEX,
        GENERATE,
        OTHER
    };
    enum class ShapeID {
        A,
        B
    };
    TriLine(NFP::Line l)
        : m_line(l.getStartPoint(), l.getEndPoint()), come_from(ShapeID::A){}
    
    void setLineType(LineAttr attr){
        this->m_attr = attr;
    }
    auto getLineType(){
        return this->m_attr;
    }
    TriLine& operator=(const TriLine& t){
        this->m_line = t.m_line;
        this->m_attr = t.m_attr;
        this->come_from = t.come_from;
        return *this;
    }
    ~TriLine() = default;
    ShapeID come_from;
    NFP::Line m_line;
    LineAttr m_attr;
};

class Triangle{
public:
    
    Triangle() = default;
    Triangle(
        std::shared_ptr<NFP::Point> p1, std::shared_ptr<NFP::Point> p2, std::shared_ptr<NFP::Point> p3,
        std::shared_ptr<NFP::Line> line1, std::shared_ptr<NFP::Line> line2, std::shared_ptr<NFP::Line> line3,
        NFP::TriangulatedPolygon::LineType lt1, NFP::TriangulatedPolygon::LineType lt2, NFP::TriangulatedPolygon::LineType lt3
    ): m_points{*p1, *p2, *p3}, m_lines{*line1, *line2, *line3} {
        #define SET_LINETYPE_TO_TRILINE(idx) switch (lt##idx)                   \
        {                                                                       \
        case NFP::TriangulatedPolygon::LineType::AbsolutelyConvexLine:               \
            m_lines[idx - 1].setLineType(TriLine::LineAttr::CONVEX); break;     \
        case NFP::TriangulatedPolygon::LineType::Generated:                          \
            m_lines[idx - 1].setLineType(TriLine::LineAttr::GENERATE); break;   \
        case NFP::TriangulatedPolygon::LineType::Regular:                            \
            m_lines[idx - 1].setLineType(TriLine::LineAttr::OTHER); break;      \
        }
        SET_LINETYPE_TO_TRILINE(1)
        SET_LINETYPE_TO_TRILINE(2)
        SET_LINETYPE_TO_TRILINE(3)
        #undef SET_LINETYPE_TO_TRILINE
    }
    TriPoint getHighestPoint(){
        auto highestPoint = this->m_points[0];
        for(int i = 1; i < 3; i++){
            if(this->m_points[i].m_point.getPoint().y > highestPoint.m_point.getPoint().y){
                highestPoint = this->m_points[i];
            }else if(this->m_points[i].m_point.getPoint().y == highestPoint.m_point.getPoint().y){
                if(this->m_points[i].m_point.getPoint().x > highestPoint.m_point.getPoint().x){
                    highestPoint = this->m_points[i];
                }else if (this->m_points[i].m_point.getPoint().x == highestPoint.m_point.getPoint().x) {
                    throw "error";
                }
            }
        }
        return highestPoint;
    }
    TriPoint getLowestPoint(){
        auto lowestPoint = this->m_points[0];
        for(int i = 1; i < 3; i++){
            if(this->m_points[i].m_point.getPoint().y < lowestPoint.m_point.getPoint().y){
                lowestPoint = this->m_points[i];
            }else if(this->m_points[i].m_point.getPoint().y == lowestPoint.m_point.getPoint().y){
                if(this->m_points[i].m_point.getPoint().x < lowestPoint.m_point.getPoint().x){
                    lowestPoint = this->m_points[i];
                }else if (this->m_points[i].m_point.getPoint().x == lowestPoint.m_point.getPoint().x){
                    throw "error";
                }
            }
        }
        return lowestPoint;
    }
    void setShapeId(TriLine::ShapeID id) {
        for (auto& m : this->m_lines) {
            m.come_from = id;
        }
    }
    Triangle& operator=(const Triangle& t){
        memcpy(&this->m_points, &t.m_points, sizeof(TriPoint) * 3);
        memcpy(&this->m_lines, &t.m_lines, sizeof(TriLine) * 3);
        this->setShapeId(m_lines[0].come_from);
        return *this;
    }
    Triangle(const Triangle& t) : m_points{t.m_points[0], t.m_points[1], t.m_points[2]}, m_lines{t.m_lines[0], t.m_lines[1], t.m_lines[2]} {}
    ~Triangle() = default;
    std::array<TriPoint, 3> m_points;
    std::array<TriLine, 3> m_lines;
};

static std::vector<Line> MinkowskiSumNFP(
    Triangle polygonA,
    Triangle polygonB,
    Vec2 startPos
) {
    auto sortTriangle = [](Vec2 basePoint, Vec2 p1, Vec2 p2)->std::pair<Vec2, Vec2> {
        auto vecp1 = p1 - basePoint;
        auto vecp2 = p2 - basePoint;
        if((vecp1 ^ vecp2) > 0){
            return std::make_pair(p1, p2);
        }else{
            return std::make_pair(p2, p1);
        }
    };
    auto findUnsolvedIndex = [](const NFP::Triangle& triangle, Vec2 target)->std::pair<int, int> {
        auto res = std::make_pair<int, int>(0, 0);
        for (int i = 0; i < 3; i++) {
            if (target == triangle.m_points[i].m_point.getPoint()) {
                std::get<0>(res) = (i + 1) % 3;
                std::get<1>(res) = (i + 2) % 3;
            }
        }
        return res;
    };
    auto PA1 = polygonA.getLowestPoint();
    auto [idxPA2, idxPA3] = findUnsolvedIndex(polygonA, PA1.m_point.getPoint());
    auto [PA2, PA3] = sortTriangle(PA1.m_point.getPoint(), polygonA.m_points[idxPA2].m_point.getPoint(), polygonA.m_points[idxPA3].m_point.getPoint());
    auto idxPA1 = (~(idxPA2 ^ idxPA3)) & 0x3;
    if (polygonA.m_points[idxPA2].m_point.getPoint() != PA2) std::swap(idxPA2, idxPA3);
    auto PB1 = polygonB.getHighestPoint();
    auto [idxPB2, idxPB3] = findUnsolvedIndex(polygonB, PB1.m_point.getPoint());
    auto [PB3, PB2] = sortTriangle(PB1.m_point.getPoint(), polygonB.m_points[idxPB2].m_point.getPoint(), polygonB.m_points[idxPB3].m_point.getPoint());
    auto idxPB1 = (~(idxPB2 ^ idxPB3)) & 0x3;
    if (polygonB.m_points[idxPB2].m_point.getPoint() != PB2) std::swap(idxPB2, idxPB3);
    int indexPBToLine[3] = {};
    for (int i = 0; i < 3; i++) {
        polygonB.m_lines[i].m_line = NFP::Line(polygonB.m_lines[i].m_line.getEndPoint(), polygonB.m_lines[i].m_line.getStartPoint());
        if (polygonB.m_points[idxPB1].m_point.getPoint() == polygonB.m_lines[i].m_line.getStartPoint()) indexPBToLine[idxPB1] = i;
        if (polygonB.m_points[idxPB2].m_point.getPoint() == polygonB.m_lines[i].m_line.getStartPoint()) indexPBToLine[idxPB2] = i;
        if (polygonB.m_points[idxPB3].m_point.getPoint() == polygonB.m_lines[i].m_line.getStartPoint()) indexPBToLine[idxPB3] = i;
    }
    // 至此 PA1 为polygonA的最低点，其对应的索引为idxPA1，逆序排序，点的顺序为PA1 PA2 PA3, 对应的索引为idxPA1, idxPA2, idxPA3
    // 至此 PB1 为polygonB的最高点，其对应的索引为idxPB1，顺序排序，点的顺序为PB1 PB2 PB3, 对应的索引为idxPB1, idxPB2, idxPB3
    Vec2 StartPosition = PA1.m_point.getPoint() + startPos;         // StartPosition为六边形的起始点
    std::array<std::pair<Vec2, int>, 6> vecs = {
        std::make_pair(PA2 - PA1.m_point.getPoint(), idxPA1),
        std::make_pair(PA3 - PA2, idxPA2),
        std::make_pair(PA1.m_point.getPoint() - PA3, idxPA3),
        std::make_pair(PB2 - PB1.m_point.getPoint(), idxPB1 + 3),
        std::make_pair(PB3 - PB2, idxPB2 + 3),
        std::make_pair(PB1.m_point.getPoint() - PB3, idxPB3 + 3),
    };
    
    auto angleCompare = [](const std::pair<Vec2, int>& a, const std::pair<Vec2, int>& b) {
        double angleA = NFP::Line(NFP::Vec2(0, 0), std::get<0>(a)).getXangle();
        double angleB = NFP::Line(NFP::Vec2(0, 0), std::get<0>(b)).getXangle();
        if (angleA < 0) angleA += 2 * PI;
        if (angleB < 0) angleB += 2 * PI;
        if (angleA != angleB) {
            return angleA < angleB;
        }
        return std::get<1>(a) < std::get<1>(b);
    };
    std::sort(vecs.begin(), vecs.end(), angleCompare);
    
    auto res = std::vector<Line>();
    res.reserve(6);
    auto firstLineIdx = std::get<1>(vecs[0]);
    res.push_back(Line(StartPosition, StartPosition + vecs[0].first));
    for (int i = 1; i < 6; i++) {
        startPos = res.back().getEndPoint();
        res.push_back(Line(startPos, startPos + vecs[i].first));
        // if (lineIdx < 3) {
        //     polygonA.m_lines[lineIdx].m_line.MoveTo(0, startPos.x, startPos.y);
            
        // }
        // else{
        //     int indexB = indexPBToLine[(lineIdx - 3) % 3];
        //     polygonB.m_lines[indexB].m_line.MoveTo(0, startPos.x, startPos.y);
        //     res.push_back(polygonB.m_lines[indexB]);
        // }
    }
    Vec2 res_mid = Vec2(0, 0);
    for(int i = 0; i < 6; i++){
        res_mid = res_mid + res[i].getEndPoint() - res[i].getStartPoint();
    }
    //std::cout << "res_mid is (" << res_mid.x << ", " << res_mid.y << ")" << std::endl;
    //DrawWarp::GetInstance().clearShapes();
    /*for(auto elem : res){
        DWCreateShape<Line>(elem);
        DWCreateShape<Point>(elem.getEndPoint());
        DWCreateShape<Point>(elem.getStartPoint());
    }*/
    return res;
}

void DelaunayTriangulationNFPAlgorithm::apply() {
    // TODO: 第四章的算法
    // assert(this->polygon_data.size() == 2);
    auto polygonA = std::make_shared<NFP::TriangulatedPolygon>(polygon_data[0], "A");
    auto polygonB = std::make_shared<NFP::TriangulatedPolygon>(polygon_data[1], "B");
    std::vector<Triangle> Atri;
    std::vector<Triangle> Btri;
    auto Amap = polygonA->getLineMapToIndex();
    auto ArawPoints = std::static_pointer_cast<NFP::Polygon>(polygonA->raw_polygon)->getPoints();
    auto ArawPointsMapToIdx = std::static_pointer_cast<NFP::Polygon>(polygonA->raw_polygon)->getVec2ToIndex();
    auto Bmap = polygonB->getLineMapToIndex();
    auto BrawPoints = std::static_pointer_cast<NFP::Polygon>(polygonB->raw_polygon)->getPoints();
    auto BrawPointsMapToIdx = std::static_pointer_cast<NFP::Polygon>(polygonB->raw_polygon)->getVec2ToIndex();
    Atri.reserve(polygonA->triangulates.size());
    Btri.reserve(polygonB->triangulates.size());
    for(int i = 0; i < polygonA->triangulates.size(); i++){
        std::pair<Vec2, Vec2> line1 = std::make_pair(std::get<0>(polygonA->triangulates[i]), std::get<1>(polygonA->triangulates[i]));
        std::pair<Vec2, Vec2> line2 = std::make_pair(std::get<1>(polygonA->triangulates[i]), std::get<2>(polygonA->triangulates[i]));
        std::pair<Vec2, Vec2> line3 = std::make_pair(std::get<2>(polygonA->triangulates[i]), std::get<0>(polygonA->triangulates[i]));
        auto idxline1 = std::get<1>(*Amap.find(line1));
        auto idxline2 = std::get<1>(*Amap.find(line2));
        auto idxline3 = std::get<1>(*Amap.find(line3));
        auto idxp1 = ArawPointsMapToIdx.find(std::get<0>(polygonA->triangulates[i]))->second;
        auto idxp2 = ArawPointsMapToIdx.find(std::get<1>(polygonA->triangulates[i]))->second;
        auto idxp3 = ArawPointsMapToIdx.find(std::get<2>(polygonA->triangulates[i]))->second;
        Atri.push_back(Triangle(
            ArawPoints[idxp1], ArawPoints[idxp2], ArawPoints[idxp3],
            polygonA->lines[idxline1], polygonA->lines[idxline2], polygonA->lines[idxline3],
            polygonA->lineTypes[idxline1], polygonA->lineTypes[idxline2], polygonA->lineTypes[idxline3]
        ));
        Atri.back().m_points[0].m_point.setIdx(idxp1);
        Atri.back().m_points[1].m_point.setIdx(idxp2);
        Atri.back().m_points[2].m_point.setIdx(idxp3);
        auto lowestPoint = Atri.back().getLowestPoint();
        Atri.back().m_points[0].setStartPos(lowestPoint.m_point.getPoint());
        Atri.back().m_points[1].setStartPos(lowestPoint.m_point.getPoint());
        Atri.back().m_points[2].setStartPos(lowestPoint.m_point.getPoint());
    }
    std::vector<Vec2> startPos; startPos.reserve(Btri.size());
    for(int i = 0; i < polygonB->triangulates.size(); i++){
        std::pair<Vec2, Vec2> line1 = std::make_pair(std::get<0>(polygonB->triangulates[i]), std::get<1>(polygonB->triangulates[i]));
        std::pair<Vec2, Vec2> line2 = std::make_pair(std::get<1>(polygonB->triangulates[i]), std::get<2>(polygonB->triangulates[i]));
        std::pair<Vec2, Vec2> line3 = std::make_pair(std::get<2>(polygonB->triangulates[i]), std::get<0>(polygonB->triangulates[i]));
        auto idxline1 = std::get<1>(*Bmap.find(line1));
        auto idxline2 = std::get<1>(*Bmap.find(line2));
        auto idxline3 = std::get<1>(*Bmap.find(line3));
        auto idxp1 = BrawPointsMapToIdx.find(std::get<0>(polygonB->triangulates[i]))->second;
        auto idxp2 = BrawPointsMapToIdx.find(std::get<1>(polygonB->triangulates[i]))->second;
        auto idxp3 = BrawPointsMapToIdx.find(std::get<2>(polygonB->triangulates[i]))->second;
        Btri.emplace_back(
            BrawPoints[idxp1], BrawPoints[idxp2], BrawPoints[idxp3],
            polygonB->lines[idxline1], polygonB->lines[idxline2], polygonB->lines[idxline3],
            polygonB->lineTypes[idxline1], polygonB->lineTypes[idxline2], polygonB->lineTypes[idxline3]
        );
        Btri.back().setShapeId(TriLine::ShapeID::B);
        Btri.back().m_points[0].m_point.setIdx(idxp1);
        Btri.back().m_points[1].m_point.setIdx(idxp2);
        Btri.back().m_points[2].m_point.setIdx(idxp3);
        auto highestPoint = Btri.back().getHighestPoint();
        startPos.push_back(polygonB->raw_polygon->getPoints()[0]->getPoint() - highestPoint.m_point.getPoint());
    }
    std::vector<std::vector<NFP::Line>> trianglesResult;
    for (int i = 0; i < Atri.size(); i++) {
        for (int j = 0; j < Btri.size(); j++) {
            auto Atriangle = Atri[i];
            auto Btriangle = Btri[j];
            auto minkowskiResult = MinkowskiSumNFP(Atriangle, Btriangle, startPos[j]);
            trianglesResult.push_back(minkowskiResult);
        }
    }
    //DrawWarp::GetInstance().clearShapes();
    std::vector<std::shared_ptr<Line>> trajectory_lines;
    for (int i = 0; i < trianglesResult.size(); i++) {
        for (int j = 0; j < trianglesResult[i].size(); j++) {
            Vec2 startPoint = trianglesResult[i][j].getStartPoint();
            Vec2 endPoint = trianglesResult[i][j].getEndPoint();
            if (startPoint != endPoint) {
                trajectory_lines.push_back(DrawWarp::GetInstance().CreateShape<Line>(startPoint, endPoint));
            }
        }
    }
    if (EventActivator::GetInstance().HasEvent("DTMinkowskiSum")) {
        for (int i = 0; i < trajectory_lines.size(); i++) {
            DrawWarp::GetInstance().addShape(trajectory_lines[i]);
            EventActivator::GetInstance().RemoveEvent("DTMinkowskiSum");
        }
    }
    // DrawWarp::GetInstance().clearShapes();
    auto startLine = FindStartLine(trajectory_lines);
    auto res = getOuterNFP(startLine, nullptr, trajectory_lines);
    for(int i = 1; i < res.size(); i++){
        DWCreateShape<Line>(*(res[i - 1]), *(res[i]));
    }
}

namespace Case{
    using Shape = NFP::Shape;
    using Point = NFP::Point;

    namespace helper {
        bool testLineIntersect(std::shared_ptr<Shape> line1, std::shared_ptr<Shape> line2, Line::LineRelationship expectedRes, std::shared_ptr<Point> expectedIntersection = nullptr) {
            auto result = std::static_pointer_cast<Line>(line1)->findIntersection(*std::static_pointer_cast<Line>(line2));
            
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
        return helper::testLineIntersect(std::static_pointer_cast<Shape>(a), std::static_pointer_cast<Shape>(b), res, intersection);
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

}

}