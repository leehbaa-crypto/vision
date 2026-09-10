#include "nrs_io.h"

// 포인트 클라우드 메시지를 PCL 포맷으로 변환 후 PCD 파일로 저장하는 함수 구현
bool nrs_io::savePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    // file_path_와 file_name_가 설정되어 있는지 확인
    if (pcd_file_path.empty())
    {
        ROS_ERROR("File path is not set in nrs_io.");
        return false;
    }
    std::stringstream ss;
    ss << pcd_file_path
       << std::setw(2) << std::setfill('0')
       << pcd_file_counter << ".pcd";

    // ——————————————
    // ★ PCD 저장 전 반드시 metadata 맞추기 ★
    cloud->width    = static_cast<uint32_t>(cloud->points.size());
    cloud->height   = 1;
    cloud->is_dense = false;  // NaN 이 있을 수 있으면 false
    // ——————————————

    // PCD 파일 저장 (ASCII 형식)
    if (pcl::io::savePCDFileASCII(ss.str(), *cloud) == 0)
    {
        ROS_INFO("Saved PCD file: %s", ss.str().c_str());
        pcd_file_counter++; // 다음 파일 저장을 위해 카운터 증가
        return true;
    }
    else
    {
        ROS_ERROR("Failed to save PCD file.");
        return false;
    }
}

bool nrs_io::savefMcMatrix(const std_msgs::Float64MultiArray::ConstPtr &msg)
{

    if (msg->data.size() != 16)
    {
        ROS_ERROR("Matrix size is not 4x4");
        return false;
    }

    std::stringstream ss;
    ss << fMc_file_path << std::setw(2) << std::setfill('0') << fMc_file_counter << ".txt";
    std::ofstream file(ss.str());
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open file to save fMc matrix");
        return false;
    }

    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            file << msg->data[i * 4 + j] << " ";
        }
        file << "\n";
    }
    file.close();
    ROS_INFO("Saved fMc matrix to: %s", ss.str().c_str());

    fMc_file_counter++;
    return true;
}


std::vector<Eigen::Matrix4d> nrs_io::loadfMcMatrix(const std::string &registration_input_fMc_path)
{
    std::vector<Eigen::Matrix4d> matrices;

    // 폴더 존재 여부 및 디렉토리 여부 확인
    if (!fs::exists(registration_input_fMc_path) || !fs::is_directory(registration_input_fMc_path))
    {
        std::cerr << "Error: Specified path does not exist: " << registration_input_fMc_path << std::endl;
        return matrices;
    }

    // 디렉토리 내의 파일 경로를 저장할 벡터
    std::vector<fs::path> file_paths;
    for (const auto &entry : fs::directory_iterator(registration_input_fMc_path))
    {
        if (fs::is_regular_file(entry.path()) && entry.path().extension() == ".txt")
        {
            file_paths.push_back(entry.path());
        }
    }

    // 파일 이름(예: "scan_fMc_00.txt")을 기준으로 오름차순 정렬
    std::sort(file_paths.begin(), file_paths.end(), [](const fs::path &a, const fs::path &b) {
        return a.filename().string() < b.filename().string();
    });

    // 정렬된 순서대로 각 파일에서 4×4 행렬 읽어오기
    for (const auto &path : file_paths)
    {
        std::ifstream file(path.string());
        if (!file.is_open())
        {
            std::cerr << "Error: Unable to open file: " << path << std::endl;
            continue;
        }

        Eigen::Matrix4d matrix;
        bool readError = false;
        // 파일에서 4×4 행렬 값을 읽어옴
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                if (!(file >> matrix(i, j)))
                {
                    std::cerr << "Error: Failed to read matrix from file: " << path << std::endl;
                    readError = true;
                    break;
                }
            }
            if (readError)
                break;
        }

        if (!readError)
            matrices.push_back(matrix);
        else
            std::cerr << "Error reading matrix. Please check the file: " << path << std::endl;
    }

    return matrices;
}

std::vector<PointCloud::Ptr> nrs_io::load_PCD_Data(const std::string &file_paths)
{
    std::vector<PointCloud::Ptr> clouds;
    fs::path directory(file_paths);
    fs::directory_iterator end_iter;
    if (fs::exists(directory) && fs::is_directory(directory))
    {
        std::vector<std::string> fileNames;
        for (fs::directory_iterator dir_iter(directory); dir_iter != end_iter; ++dir_iter)
        {
            if (fs::is_regular_file(dir_iter->status()) && dir_iter->path().extension() == ".pcd")
            {
                fileNames.push_back(dir_iter->path().filename().string());
            }
        }
        std::sort(fileNames.begin(), fileNames.end());
        for (const auto &fileName : fileNames)
        {
            PointCloud::Ptr cloud(new PointCloud);
            if (pcl::io::loadPCDFile<PointT>((directory / fileName).string(), *cloud) == 0)
            {
                clouds.push_back(cloud);
                std::cout << fileName << std::endl;
            }
        }
    }
    else
    {
        std::cerr << "Invalid folder path." << std::endl;
    }
    return clouds;
}

Point_set nrs_io::load_Point_set_Data(const std::string &fname)
{
    Point_set points;
    std::ifstream stream(fname, std::ios_base::binary);
    if (!stream)
    {
        throw std::runtime_error("Error: cannot read file " + fname);
    }
    // 파일 스트림을 12번째 줄까지 건너뜁니다.
    for (int i = 0; i < 12; ++i)
    {
        std::string dummy;
        std::getline(stream, dummy);
    }
    Point_3 point;
    while (stream >> point)
    {
        points.insert(point);
    }
    std::cout << "Read " << points.size() << " point(s)" << std::endl;
    if (points.empty())
    {
        throw std::runtime_error("Error: no points read from file " + fname);
    }
    return points;
}

void nrs_io::loadYAML(const std::string &filename, float &thetaX, float &thetaY, float &thetaZ,
                      float &X, float &Y, float &Z, int &n,
                      float minrange[4], float maxrange[4],
                      float &downsampleparam, int &sor_mean, double &sor_thresh,
                      double &ror_radius, int &ror_neighbor, float &smoothing_radius,
                      float &relative_alpha, float &relative_offset,
                      int &bilateral_k, double &bilateral_sharpness_angle, int &bilateral_iter_number)
{
    // YAML 파일을 읽어들이기 위한 ifstream 객체 생성
    std::ifstream fin(filename);
    if (!fin.is_open())
    {
        throw std::runtime_error("Failed to open YAML file: " + filename);
    }

    // YAML 파싱
    YAML::Node doc = YAML::Load(fin);

    // hand_eye_calibration 정보 불러오기 (각도는 radian 단위로 변환)
    thetaX = doc["hand_eye_calibration"][0].as<float>() * M_PI / 180;
    thetaY = doc["hand_eye_calibration"][1].as<float>() * M_PI / 180;
    thetaZ = doc["hand_eye_calibration"][2].as<float>() * M_PI / 180;

    X = doc["hand_eye_calibration"][3].as<float>();
    Y = doc["hand_eye_calibration"][4].as<float>();
    Z = doc["hand_eye_calibration"][5].as<float>();

    int m = 0;
    for (const auto &element : doc["minrange"])
    {
        minrange[m++] = element.as<float>();
    }

    int k = 0;
    for (const auto &element : doc["maxrange"])
    {
        maxrange[k++] = element.as<float>();
    }

    downsampleparam = doc["downsampleparam"][0].as<float>();
    sor_mean = doc["sor_mean"][0].as<int>();
    sor_thresh = doc["sor_thresh"][0].as<double>();
    ror_radius = doc["ror_radius"][0].as<double>();
    ror_neighbor = doc["ror_neighbor"][0].as<int>();
    smoothing_radius = doc["smoothing_radius"][0].as<float>();
    relative_alpha = doc["relative_alpha"][0].as<float>();
    relative_offset = doc["relative_offset"][0].as<float>();
    bilateral_k = doc["bilateral_k"][0].as<int>();
    bilateral_sharpness_angle = doc["bilateral_sharpness_angle"][0].as<double>();
    bilateral_iter_number = doc["bilateral_iter_number"][0].as<int>();
}
