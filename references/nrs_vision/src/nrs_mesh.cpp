#include "nrs_mesh.h"

Point_set nrs_mesh::outlier_remove(Point_set &points)
{

    // 이웃 24개를 고려하여 상위 2% 이상치를 제거합니다.
    auto rout_it = CGAL::remove_outliers<CGAL::Sequential_tag>(
        points,
        24,
        points.parameters().threshold_percent(2.0));

    points.remove(rout_it, points.end());
    std::cout << points.number_of_removed_points() << " point(s) are outliers." << std::endl;
    points.collect_garbage();
    return points;
}

Point_set nrs_mesh::grid_simplify(Point_set &points)
{

    double spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(points, 20);
    auto gsim_it = CGAL::grid_simplify_point_set(points, 1.2 * spacing);
    points.remove(gsim_it, points.end());
    std::cout << points.number_of_removed_points() << " point(s) removed after simplification." << std::endl;
    return points;
}

PointList nrs_mesh::convert_to_point_list(const Point_set &points)
{
    PointList point_list;
    for (const auto &point : points.points())
    {
        point_list.push_back(std::make_pair(point, Vector_3(0, 0, 0))); // 노말 초기값 0
    }
    return point_list;
}

PointList nrs_mesh::estimate_normal(PointList &points)
{

    CGAL::jet_estimate_normals<CGAL::Sequential_tag>(
        points,
        18,
        CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>())
            .normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>()));
    return points;
}

PointList nrs_mesh::bilateral_smooth(PointList &points, const int k, const double sharpness_angle, const int iter_number)
{

    for (int i = 0; i < iter_number; ++i)
    {
        CGAL::bilateral_smooth_point_set<CGAL::Sequential_tag>(
            points,
            k,
            CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>())
                .normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>())
                .sharpness_angle(sharpness_angle));
    }
    return points;
}

void nrs_mesh::generate_mesh(const PointList &point_list, const std::string &output_filename, const double relative_alpha, const double relative_offset)
{
    Point_container pointwrap;
    for (const auto &point : point_list)
    {
        pointwrap.push_back(point.first);
    }

    Mesh meshwrap;
    CGAL::Bbox_3 bbox = CGAL::bbox_3(std::cbegin(pointwrap), std::cend(pointwrap));
    const double diag_length = std::sqrt(
        CGAL::square(bbox.xmax() - bbox.xmin()) +
        CGAL::square(bbox.ymax() - bbox.ymin()) +
        CGAL::square(bbox.zmax() - bbox.zmin()));
    const double alpha = diag_length / relative_alpha;
    const double offset = diag_length / relative_offset;
    CGAL::alpha_wrap_3(pointwrap, alpha, offset, meshwrap);

    CGAL::IO::write_polygon_mesh(output_filename, meshwrap, CGAL::parameters::stream_precision(17));
}


// reconstruction 함수 구현
void nrs_mesh::reconstruction(Point_set &input_points, const int &bilateral_k, const double &bilateral_sharpness_angle,
                              const int &bilateral_iter_number, const double &relative_alpha, const double &relative_offset,
                              const std::string &output_mesh_path)
{

    // input_points = outlier_remove(input_points);

    // (필요 시 grid simplification도 수행할 수 있음)
    // points = grid_simplify(points);

    // 3. Point_set -> (Point, Normal) 리스트로 변환
    PointList point_list = convert_to_point_list(input_points);

    // 4. 노말 추정
    point_list = estimate_normal(point_list);

    // 6. 양측 평활화
    // point_list = bilateral_smooth(point_list, bilateral_k, bilateral_sharpness_angle, bilateral_iter_number);

    // 7. 메쉬 생성 (출력 파일로 저장)
    generate_mesh(point_list, output_mesh_path, relative_alpha, relative_offset);

    std::cout << "Remeshing done. Mesh saved to " << output_mesh_path << std::endl;
}
