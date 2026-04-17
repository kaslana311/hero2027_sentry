#ifndef COLOR_MAP_MODEL_HPP
#define COLOR_MAP_MODEL_HPP
#include <iostream>
#include <opencv2/opencv.hpp>

#include "yaml-cpp/yaml.h"

namespace colorMap
{
enum COLOR_ID { NONE = 0, RED = 1, GREEN = 2, BLUE = 3 };  //RED可到达GREEN,BLUE用于过洞
#define ERROR_CODE -100
/*
    yaml 读取指定索引内容哦
    */
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
class ColorMapModel
{
public:
  ColorMapModel(
    const std::string & color_map_file, const std::string & yaml_file,
    bool is_using_costcoffient = false);

  /*
  map转image
  */
  bool ColorMap2Image(unsigned int & mx, unsigned int & my, unsigned int & ix, unsigned int & iy);
  /*
  图片坐标系转化为map坐标系
  */
  bool Image2ColorMap(unsigned int & ix, unsigned int & iy, unsigned int & mx, unsigned int & my);

  /*
  世界坐标转化为image坐标
  */
  bool World2ColorImage(double & wx, double & wy, unsigned & mx, unsigned & my);
  bool World2ColorMap(double & wx, double & wy, unsigned & mx, unsigned & my);

  /* 提取彩色区域地图指定位置信息*/
  inline int GetColorMapInfo(unsigned int mx, unsigned int my);

  /*
   输入 世界真实坐标,和目标点
   返回对应位移的代价系数
  */
  double GetCostCoefficient(double position_x, double position_y, double target_x, double target_y);
  // inline bool FromtowordsIsSafe(int & cx, int & cy, int & tx, int & ty)
  // {
  //   int cost = 1;  //color_model->IntGetCostCoefficient(cx, cy, tx, ty);
  //   if (cost == 1) {
  //     return false;
  //   }
  //   return true;
  // }
  inline int ReturnKeyPointType(unsigned int mx, unsigned int my, unsigned int tx, unsigned int ty)
  {
    if (mx >= image_width_ || my >= image_height_ || tx >= image_width_ || ty >= image_height_) {
      return ERROR_CODE;
    }
    my = this->image_height_ - my - 1;
    ty = this->image_height_ - ty - 1;
    return status_
      [color_map_[mx + my * image_width_] + color_map_[tx + ty * image_width_] * status_sqrt_];
  }
  /*
  输入 
  world下的 当前位置
  map下的 目标点
  返回对应代价系数
  */
  double GetCostCoefficient(double position_x, double position_y, unsigned int tx, unsigned int ty);
  /*
  输入map下的坐标和目标点
  返回对应代价系数
  */
  double GetCostCoefficient(unsigned int px, unsigned int py, unsigned int tx, unsigned int ty);
  void UpdateNowPostion(double & position_x, double & position_y);
  /*
  返回状态,上下坡
  1,上坡
  0 平地
  -1 下坡
  暂时不支持台阶相关数据
  */
  int GetStatus();
  void PrintMAPinfo()
  {
    std::cerr << "color_map_file_: " << color_map_file_ << std::endl;
    std::cerr << "yaml_file_: " << yaml_file_ << std::endl;
    std::cerr << "image_width_: " << image_width_ << std::endl;
    std::cerr << "resolution_: " << resolution_ << std::endl;
    std::cerr << "origin_: [" << origin_[0] << ", " << origin_[1] << "]" << std::endl;
    std::cerr << "image_height_: " << image_height_ << std::endl;
  }

private:
  /*
初始化
*/
  void ReadFileAndCreateMap();
  //c参数
  std::string color_map_file_;
  bool enable_ = false;
  bool is_using_cost_coffient_map_ = false;
  std::string yaml_file_;
  size_t image_width_;
  double resolution_;
  std::vector<double> origin_;
  size_t image_height_;
  size_t image_size_;
  int * color_map_;
  int towords_num_ = 8;
  double * cost_coffient_map_;  //对应image坐标索引

  struct movetion
  {
    int dx;
    int dy;
    movetion(int x, int y) : dx(x), dy(y){};
  };
  std::vector<movetion> list_towords_{{0, 1},  {-1, 1}, {-1, 0}, {-1, -1},
                                      {0, -1}, {1, -1}, {1, 0},  {1, 1}};
  double max_ = 5;
  int length_ = 4;
  int status_sqrt_ = 4;
  double num_[16] = {1, max_, max_, 1, 1, 1, 1, 1, max_, 1, 1, 1, max_, max_, 1, 1};

  int status_[16] = {1, 1, 5, 5, 2, 1, 1, 0, 3, 1, 1, 1, 3, 1, 1, 1};

  int now_position_ = 0;
  int last_position_ = 0;
};

}  // namespace colorMap

#endif