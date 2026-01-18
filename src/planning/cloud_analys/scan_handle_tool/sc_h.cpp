#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>
cv::Mat high_map;
std::vector<double> map_origin{0.0, 0.0, 0.0};  //左下角点云坐标
double yaml_resolution{0.05};
template <typename T>
T yaml_get_value(const YAML::Node & node, const std::string & key)
{
  try {
    return node[key].as<T>();
  } catch (YAML::Exception & e) {
    std::stringstream ss;
    ss << "Failed to parse YAML tag '" << key << "' for reason: " << e.msg;
    throw YAML::Exception(e.mark, ss.str());
  }
}
int main(int argc, char ** argv)
{
  if (argc < 7) {
    std::cout << "Please provide the path to the YAML file and the PCD file." << std::endl;
    return -1;
  }
  std::string yaml_path = argv[1];
  std::string pcd_path = argv[2];
  double min_z = std::stod(argv[3]);
  double max_z = std::stod(argv[4]);
  std::string save_path = argv[5];
  double height_percent = std::stod(argv[6]);

  //读取yaml文件，获取地图信息
  YAML::Node config = YAML::LoadFile(yaml_path);
  map_origin = yaml_get_value<std::vector<double>>(config, "origin");
  yaml_resolution = yaml_get_value<double>(config, "resolution");
  std::string pgm_path;
  int index = yaml_path.find_last_of("/");
  pgm_path = yaml_path.substr(0, index + 1) + yaml_get_value<std::string>(config, "image");
  std::cout<<"input info "<<std::endl;
  std::cout<<"yaml path: "<<yaml_path<<std::endl;
  std::cout<<"save path: "<<save_path<<std::endl;
  std::cout<<"min z : "<<min_z<<" max_z "<<max_z<<" height_pre "<<height_percent<<std::endl;

  cv::Mat pgm_map = cv::imread(pgm_path, cv::IMREAD_UNCHANGED);
  if (pgm_map.empty()) {
    std::cerr << " -- [ERROR] Failed to load map image file: " << pgm_path << std::endl;
    return -1;
  }

  high_map = cv::Mat::zeros(pgm_map.rows, pgm_map.cols, CV_16SC1);
  cv::Mat visit_map = cv::Mat::zeros(pgm_map.rows, pgm_map.cols, CV_8UC1);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
    PCL_ERROR("Couldn't read file %s \n", pcd_path.c_str());
    return -1;
  }
  std::cout<<"input cloud size "<<cloud->size()<<std::endl;
  pcl::PassThrough<pcl::PointXYZ> pass_filter;
  pass_filter.setInputCloud(cloud);
  pass_filter.setFilterFieldName("z");
  pass_filter.setFilterLimits(min_z, max_z);
  pass_filter.filter(*cloud);
  std::cout<<"filtered size "<<cloud->size()<<std::endl;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> grid_map(pgm_map.rows * pgm_map.cols);
  pcl::io::savePCDFile ("/home/dji/project/HERO2025_Sentry/src/planning/cloud_analys/high_map/test.pcd",*cloud);
    double  min_z_total = 100.5;
    std::cout<<min_z_total;
    for (const auto & point : cloud->points) {
      int index_x = static_cast<int>((point.x - map_origin[0]) / yaml_resolution);
      int index_y = static_cast<int>((point.y - map_origin[1]) / yaml_resolution);
      if (index_x >= 0 && index_x < pgm_map.cols && index_y >= 0 && index_y < pgm_map.rows) {
        int index_l = index_y * pgm_map.cols + index_x;
        grid_map[index_l].points.push_back(point);
        if (point.z < min_z_total){ min_z_total = point.z;}
      }
    }
  // 每个格子内部的点按照高度进行排序,取95%高度作为该格子的高度值
  for (auto & cell : grid_map) {
    std::sort(
      cell.points.begin(), cell.points.end(),
      [](const pcl::PointXYZ & a, const pcl::PointXYZ & b) { return a.z < b.z; });
  }
  int count = 0;
  for (int y = 0; y < pgm_map.rows; ++y) {
    for (int x = 0; x < pgm_map.cols; ++x) {
      int index_l = (pgm_map.rows - 1 - y) * pgm_map.cols + x;
      const auto & cell = grid_map[index_l];
      if (cell.points.empty()) {
        high_map.at<int16_t>(y, x) = 0;
      } else {
        size_t idx_95 = static_cast<size_t>(cell.points.size() * height_percent);
        high_map.at<int16_t>(y, x) = static_cast<int16_t>((cell.points[idx_95].z) * 1000);
        visit_map.at<uint8_t>(y, x) = 1;
        std::cout<<" data : "<<high_map.at<int16_t>(y, x)<<std::endl;
        count++;
      }
    }
  }
  std::vector<std::pair<int, int>> directions = {{1, 0}, {-1, 0}, {0, 1},  {0, -1},
                                                 {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
  for (int y = 0; y < pgm_map.rows; ++y) {
    for (int x = 0; x < pgm_map.cols; ++x) {
      if (visit_map.at<uint8_t>(y, x) == 0 || high_map.at<int16_t>(y, x) < 50) {
        int16_t h = 0;
        int dod = 0;
        for (int step = 1; step < 5; step++) {
          for (const auto & dir : directions) {
            int nx = x + step * dir.first;
            int ny = y + step * dir.second;
            if (nx >= 0 && nx < pgm_map.cols && ny >= 0 && ny < pgm_map.rows) {
              if (visit_map.at<uint8_t>(ny, nx) == 1) {
                h += high_map.at<int16_t>(ny, nx);
                dod++;
              }
            }
          }
          if (dod > 0) {
            high_map.at<int16_t>(y, x) = static_cast<int16_t>(h / dod);
            break;
          }
        }
      }
    }
  }

  //save high_map
  cv::imwrite(save_path, high_map);
  cv::Mat visual_map;

  high_map.convertTo(visual_map, CV_8S, 255.0 / 800.0);  //1 m

  cv::namedWindow("High Map Debug", cv::WINDOW_NORMAL);
  cv::imshow("High Map Debug", visual_map);

  std::cout << "窗口已弹出，按任意键关闭并退出..." << std::endl;
  cv::waitKey(0);
  std::cout << "High map saved as " << save_path << " with " << count << " valid points."
            << std::endl;
    std::cout<<min_z_total;
  return 0;
}