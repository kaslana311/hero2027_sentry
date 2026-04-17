#include "towords_planner/color_map_model.hpp"
using namespace colorMap;
colorMap::ColorMapModel::ColorMapModel(
  const std::string & color_map_file, const std::string & yaml_file, bool is_using_costcoffient)
: color_map_file_(color_map_file),
  is_using_cost_coffient_map_(is_using_costcoffient),
  yaml_file_(yaml_file)
{
  ReadFileAndCreateMap();
}
/*
  map转image
  */
bool colorMap::ColorMapModel::ColorMap2Image(
  unsigned int & mx, unsigned int & my, unsigned int & ix, unsigned int & iy)
{
  if (mx > this->image_width_ || my > this->image_height_) return false;
  ix = mx;
  iy = this->image_height_ - my - 1;
  if (ix > this->image_width_ || iy > this->image_height_) return false;
  return true;
}
/*
  图片坐标系转化为map坐标系
  */
bool colorMap::ColorMapModel::Image2ColorMap(
  unsigned int & ix, unsigned int & iy, unsigned int & mx, unsigned int & my)
{
  if (ix > this->image_width_ || iy > this->image_height_) return false;
  mx = ix;
  my = this->image_height_ - iy - 1;
  if (mx > this->image_width_ || my > this->image_height_) return false;
  return true;
}
/*
  世界坐标转化为image坐标
  */
bool colorMap::ColorMapModel::World2ColorImage(
  double & wx, double & wy, unsigned & mx, unsigned & my)
{
  if (wx < this->origin_[0] || wy < this->origin_[1]) return false;

  mx = (int)((wx - this->origin_[0]) / this->resolution_);
  my = (int)((wy - this->origin_[1]) / this->resolution_);
  my = this->image_height_ - my - 1;
  if (mx < this->image_width_ && my < this->image_height_) return true;

  return false;
}
/* 提取彩色区域地图指定位置信息
in  map 下的 坐标
out  iamge下对应索引数据
*/
bool colorMap::ColorMapModel::World2ColorMap(double & wx, double & wy, unsigned & mx, unsigned & my)
{
  if (wx < this->origin_[0] || wy < this->origin_[1]) return false;
  mx = (int)((wx - this->origin_[0]) / this->resolution_);
  my = (int)((wy - this->origin_[1]) / this->resolution_);
  if (mx < this->image_width_ && my < this->image_height_) return true;
  return false;
}
inline int colorMap::ColorMapModel::GetColorMapInfo(unsigned int mx, unsigned int my)
{
  unsigned int ix, iy;
  if (ColorMap2Image(mx, my, ix, iy)) return color_map_[ix + iy * this->image_width_];
  return ERROR_CODE;
}
/*
   输入 世界真实坐标,和目标点
   返回对应位移的代价系数
  */
double colorMap::ColorMapModel::GetCostCoefficient(
  double position_x, double position_y, double target_x, double target_y)
{
  if (cost_coffient_map_ == nullptr) return ERROR_CODE;
  unsigned int px, py, tx, ty;
  if (
    World2ColorImage(position_x, position_y, px, py) &&
    World2ColorImage(target_x, target_y, tx, ty)) {
    int dx = tx - px;
    int dy = ty - py;
    int index = 0;
    for (auto & it : list_towords_) {
      if (it.dx == dx && it.dy == dy) {
        break;
      }
      index++;
    }
    return cost_coffient_map_[index * image_size_ + px + py * this->image_width_];
  }
  return ERROR_CODE;
}
/*
  输入 
  world下的 当前位置
  map下的 目标点
  返回对应代价系数
  */
double colorMap::ColorMapModel::GetCostCoefficient(
  double position_x, double position_y, unsigned int tx, unsigned int ty)
{
  if (cost_coffient_map_ == nullptr) return ERROR_CODE;
  unsigned int px, py, ix, iy;
  if (World2ColorImage(position_x, position_y, px, py) && ColorMap2Image(tx, ty, ix, iy)) {
    int dx = ix - px;
    int dy = iy - py;
    int index = 0;
    for (auto & it : list_towords_) {
      if (it.dx == dx && it.dy == dy) {
        break;
      }
      index++;
    }
    return cost_coffient_map_[index * image_size_ + px + py * this->image_width_];
  }
  return ERROR_CODE;
}
/*
  输入map下的坐标和目标点
  返回对应代价系数
  */
double colorMap::ColorMapModel::GetCostCoefficient(
  unsigned int px, unsigned int py, unsigned int tx, unsigned int ty)
{
  if (cost_coffient_map_ == nullptr) return ERROR_CODE;
  unsigned int ipx, ipy, ix, iy;
  if (ColorMap2Image(px, py, ipx, ipy) && ColorMap2Image(tx, ty, ix, iy)) {
    int dx = ix - ipx;
    int dy = iy - ipy;
    int index = 0;
    for (auto & it : list_towords_) {
      if (it.dx == dx && it.dy == dy) {
        break;
      }
      index++;
    }
    return cost_coffient_map_[index * image_size_ + ipx + ipy * this->image_width_];
  }
  return ERROR_CODE;
}
void colorMap::ColorMapModel::UpdateNowPostion(double & position_x, double & position_y)
{
  unsigned int ix, iy;
  if (World2ColorImage(position_x, position_y, ix, iy)) {
    if (color_map_[ix + iy * image_height_] != now_position_) {
      last_position_ = now_position_;
      now_position_ = color_map_[ix + iy * image_height_];
    }
  }
}
/*
  返回状态,上下坡
  1,上坡
  0 平地
  -1 下坡
  暂时不支持台阶相关数据
  */
int colorMap::ColorMapModel::GetStatus()
{
  if (now_position_ == -1 || last_position_ == -1) return ERROR_CODE;
  return status_[last_position_ + now_position_ * status_sqrt_];
}
/*
初始化
*/
void colorMap::ColorMapModel::ReadFileAndCreateMap()
{  //彩色地图读取
  cv::Mat image = cv::imread(this->color_map_file_, 1);
  if (image.empty()) return;
  this->image_height_ = image.rows;
  this->image_width_ = image.cols;
  this->image_size_ = this->image_height_ * this->image_width_;
  this->color_map_ = new int[this->image_height_ * this->image_width_];
  // 转换为 HSV
  cv::Mat hsv_image;
  cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);
  // 使用指针访问，效率比 .at 高得多,可以上tbb
  for (int i = 0; i < hsv_image.rows; ++i) {
    // 获取当前行的首地址
    cv::Vec3b * row_ptr = hsv_image.ptr<cv::Vec3b>(i);
    for (int j = 0; j < hsv_image.cols; ++j) {
      uint8_t h = row_ptr[j][0];
      uint8_t s = row_ptr[j][1];
      uint8_t v = row_ptr[j][2];

      int curr_index = i * this->image_width_ + j;

      // 鲁棒性：排除太暗或太淡的颜色
      if (s < 40 || v < 40) {
        color_map_[curr_index] = NONE;
      } else if ((h >= 0 && h <= 10) || (h >= 156 && h <= 180)) {
        color_map_[curr_index] = RED;
      } else if (h >= 35 && h <= 85) {
        color_map_[curr_index] = GREEN;

      } else if (h >= 100 && h <= 130) {
        color_map_[curr_index] = BLUE;
      } else {
        color_map_[curr_index] = NONE;
      }
    }
  }

  now_position_ = color_map_[0];
  last_position_ = color_map_[0];
#ifdef DEBUG
  // 输出color_map_到图片,调试用
  cv::Mat res_mat(image.size(), CV_8UC3);
  for (int i = 0; i < hsv_image.rows; ++i) {
    cv::Vec3b * row_ptr = res_mat.ptr<cv::Vec3b>(i);
    for (int j = 0; j < hsv_image.cols; ++j) {
      int index = i * this->image_width_ + j;
      if (color_map_[index] == RED) {
        row_ptr[j] = cv::Vec3b(0, 0, 255);
      } else if (color_map_[index] == GREEN) {
        row_ptr[j] = cv::Vec3b(0, 255, 0);
      } else if (color_map_[index] == BLUE) {
        row_ptr[j] = cv::Vec3b(255, 0, 0);
      } else {
        row_ptr[j] = cv::Vec3b(0, 0, 0);
      }
    }
  }
  cv::imwrite("/home/z/res.png", res_mat);
#endif
  cost_coffient_map_ = nullptr;
  //代价系数准备(如果需要),不好使,内存分配太大了
  if (is_using_cost_coffient_map_) {
    this->cost_coffient_map_ = new double[image_height_ * image_width_ * towords_num_];
    for (size_t j = 0; j < this->image_height_; j++) {
      for (size_t i = 0; i < this->image_width_; i++) {
        int index = 0;
        int now = color_map_[i + j * image_width_];
        for (auto & it : list_towords_) {
          int target = color_map_[i + it.dx + (j + it.dy) * image_width_];
          cost_coffient_map_[index * (image_height_ * image_width_) + i + j * image_width_] =
            num_[(target + 1) * length_ + now];
          index++;
        }
      }
    }
  }
  //读取yaml
  this->enable_ = true;
  try {
    YAML::Node config = YAML::LoadFile(yaml_file_);
    resolution_ = yaml_get_value<double>(config, "resolution");
    origin_ = yaml_get_value<std::vector<double>>(config, "origin");
  } catch (const YAML::TypedBadConversion<double> & e) {
    this->enable_ = false;
  }
}