#ifndef NRS_MESH_H
#define NRS_MESH_H

#include <string>
#include <vector>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <array>

// CGAL 관련 헤더

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <CGAL/Point_set_3.h>
#include <CGAL/remove_outliers.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/bilateral_smooth_point_set.h>
#include <CGAL/alpha_wrap_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Bbox_3.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/IO/write_points.h>


typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Vector_3 Vector_3;

// CGAL::Point_set_3를 점 집합 타입으로 사용
typedef CGAL::Point_set_3<Point_3> Point_set;

// 점과 노말을 저장하는 pair 및 리스트 정의
typedef std::pair<Point_3, Vector_3> PointVectorPair;
typedef std::vector<PointVectorPair> PointList;

// 메쉬 생성을 위한 컨테이너와 메쉬 타입 (여기서는 Surface_mesh 사용)
typedef std::vector<Point_3> Point_container;
typedef CGAL::Surface_mesh<Point_3> Mesh;

class nrs_mesh
{
public:
  // Kernel 및 타입 정의

  // 이상치 제거: 주어진 점 집합에서 일정 비율 이상치를 제거
  Point_set outlier_remove(Point_set &points);

  // 그리드 단순화: 점 집합을 평균 간격에 기반하여 단순화
  Point_set grid_simplify(Point_set &points);

  // 점 집합을 노말이 0으로 초기화된 (점, 노말) 리스트로 변환
  PointList convert_to_point_list(const Point_set &points);

  // 제트 방법을 사용하여 점 목록에 노말을 추정
  PointList estimate_normal(PointList &points);

  // 양측 평활화: 지정된 이웃 크기, sharpness, 반복 횟수를 사용하여 점 목록을 평활화
  PointList bilateral_smooth(PointList &points, const int k, const double sharpness_angle, const int iter_number);

  // 메쉬 생성: 점 목록으로부터 메쉬를 생성하여 파일에 저장 (알파 값과 오프셋은 상대 길이로 산출)
  void generate_mesh(const PointList &point_list, const std::string &output_filename, const double relative_alpha, const double relative_offset);
  // reconstruction 함수: pcd 파일, YAML 파일, 출력 메쉬 파일 경로를 인자로 받아 전체 메쉬 재구성 파이프라인 수행
  void reconstruction(Point_set &input_points, const int &bilateral_k, const double &bilateral_sharpness_angle,
                      const int &bilateral_iter_number, const double &relative_alpha, const double &relative_offset,
                      const std::string &output_mesh_path);

};

#endif // NRS_MESH_H
