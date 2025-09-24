#include <fstream>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Projection_traits_xy_3.h>

#include <CGAL/boost/graph/copy_face_graph.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh/IO/PLY.h>

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/draw_triangulation_2.h>
#include <CGAL/mark_domain_in_triangulation.h>
#include <CGAL/Polygon_2.h>
#include <util.hpp>
#include <Shape.hpp>

#include <nlohmann/json.hpp>
#include <Logger.h>

std::vector<tranglationPoints> delaunay_triangulation(std::vector<std::shared_ptr<NFP::Point>> points){
    using K = CGAL::Exact_predicates_inexact_constructions_kernel;
    using Projection_traits = CGAL::Projection_traits_xy_3<K>;
    using Vb = CGAL::Triangulation_vertex_base_2<Projection_traits>;
    using Fb = CGAL::Constrained_triangulation_face_base_2<Projection_traits>;
    using TDS = CGAL::Triangulation_data_structure_2<Vb, Fb>;
    using Itag = CGAL::Exact_predicates_tag;
    using CDT = CGAL::Constrained_Delaunay_triangulation_2<Projection_traits, TDS, Itag>;
    using Face_handle = CDT::Face_handle;
    using CDTPoint = CDT::Point;
    using Polygon_2 = CGAL::Polygon_2<Projection_traits>;
    using Mesh = CGAL::Surface_mesh<CDTPoint>;
    using vertex_descriptor = Mesh::Vertex_index;

    std::vector<tranglationPoints> res;

    Polygon_2 polygon1;
    for (auto &p : points){
        polygon1.push_back(CDTPoint(p->getPoint().x, p->getPoint().y, 0));
    }

    CDT cdt;
    cdt.insert_constraint(polygon1.vertices_begin(), polygon1.vertices_end(), true);
    std::unordered_map<Face_handle, bool> in_domain_map;
    boost::associative_property_map< std::unordered_map<Face_handle, bool> >
        in_domain(in_domain_map);
    //Mark facets that are inside the domain bounded by the polygon
    CGAL::mark_domain_in_triangulation(cdt, in_domain);
    int i = 0;
    for (Face_handle f : cdt.finite_face_handles())
    {
        if (get(in_domain, f)) {       
            res.push_back(std::make_tuple(
                NFP::Vec2(f->vertex(0)->point().x(), f->vertex(0)->point().y()),
                NFP::Vec2(f->vertex(1)->point().x(), f->vertex(1)->point().y()),
                NFP::Vec2(f->vertex(2)->point().x(), f->vertex(2)->point().y())
            ));
            // std::cout << "----------------------------------------\n";
            // std::cout << "Triangle " << i++ << ":\n";
            // std::cout << "  Vertex 0: " << f->vertex(0)->point() << "\n";
            // std::cout << "  Vertex 1: " << f->vertex(1)->point() << "\n";
            // std::cout << "  Vertex 2: " << f->vertex(2)->point() << "\n";
        }
    }
    return res;
}

std::vector<std::vector<std::shared_ptr<NFP::Point>>> getDataFromJson(const std::string& jsonPath) {
    std::ifstream file(jsonPath);
    nlohmann::json j;
    file >> j;
    std::vector<std::vector<std::shared_ptr<NFP::Point>>> res;
    std::vector<std::shared_ptr<NFP::Point>> A;
    assert(j.contains("data"));
    assert(j["data"].contains("A"));
    for (const auto& point : j["data"]["A"]) {
        float x = point["x"];
        float y = point["y"];
        A.push_back(std::make_shared<NFP::Point>(x, y));
    }
    res.push_back(A);
    std::vector<std::shared_ptr<NFP::Point>> B;
    assert(j["data"].contains("B"));
    for (const auto& point : j["data"]["B"]) {
        float x = point["x"];
        float y = point["y"];
        B.push_back(std::make_shared<NFP::Point>(x, y));
    }
    res.push_back(B);
    return res;
}