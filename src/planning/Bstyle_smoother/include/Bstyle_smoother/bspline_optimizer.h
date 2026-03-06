#ifndef BSPLINE_OPTIMIZER_HPP_
#define BSPLINE_OPTIMIZER_HPP_

#include <Eigen/Eigen>
#include <memory>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace fast_planner
{
struct BsplineConfig
{
  /* optimization parameters */
  int order_ = 4;           // bspline degree (cubic)
  double lambda1_ = 100.0;  // 平滑性 (acc+jerk) ← 全向底盘核心: 强力平滑
  double lambda2_ = 20.0;   // 距离代价权重
  double lambda3_ = 10.0;   // 可行性（速度/加速度约束）← 全向底盘约束宽松
  double lambda4_ = 4.0;    // 终点约束
  double lambda5_ = 0.1;    // 引导路径 ← 极低, 允许弯道处大幅偏离形成圆弧
  double lambda6_ = 1.0;
  double lambda7_ = 1.0;  // 航路点
  double lambda8_ = 1.0;

  // 约束参数
  double dist0_ = 0.5;  // 安全距离 (米)
  double max_vel_ = 2.0;
  double max_acc_ = 4.0;
  double visib_min_ = 0.0;
  double wnl_ = 0.0;
  double dlmin_ = 0.0;
  //
  int algorithm1_ = 15;  // SLSQP for quadratic cost
  int algorithm2_ = 11;  // LBFGS for general cost

  // stopping criteria — 只用迭代次数控制, 不用时间限制
  int max_iteration_num1_ = 500;
  int max_iteration_num2_ = 500;
  int max_iteration_num3_ = 300;
  int max_iteration_num4_ = 300;
  double max_iteration_time1_ = 1e9;  // 不限时间
  double max_iteration_time2_ = 1e9;  // 不限时间
  double max_iteration_time3_ = 1e9;
  double max_iteration_time4_ = 1e9;
};
class BsplineOptimizer
{
public:
  static const int SMOOTHNESS;
  static const int DISTANCE;
  static const int FEASIBILITY;
  static const int ENDPOINT;
  static const int GUIDE;
  static const int WAYPOINTS;

  static const int GUIDE_PHASE;
  static const int NORMAL_PHASE;

  BsplineOptimizer() {}
  ~BsplineOptimizer();

  /* main API */
  void setEnvironment(const std::shared_ptr<nav2_costmap_2d::Costmap2D> env);
  void setParam(std::shared_ptr<BsplineConfig> nh);
  void rebuildESDF();  // 强制重建 ESDF 缓存 (确保一致性)
  Eigen::MatrixXd BsplineOptimizeTraj(
    const Eigen::MatrixXd & points, const double & ts, const int & cost_function, int max_num_id,
    int max_time_id);
  void evaluateEDTWithGrad(
    const Eigen::Vector2d & pos, double time, double & dist, Eigen::Vector2d & grad);

  /* helper function */

  // required inputs
  void setControlPoints(const Eigen::MatrixXd & points);
  void setBsplineInterval(const double & ts);
  void setCostFunction(const int & cost_function);
  void setTerminateCond(const int & max_num_id, const int & max_time_id);

  // optional inputs
  void setGuidePath(const std::vector<Eigen::Vector2d> & guide_pt);
  void setWaypoints(
    const std::vector<Eigen::Vector2d> & waypts,
    const std::vector<int> & waypt_idx);  // N-2 constraints at most

  void optimize();

  Eigen::MatrixXd getControlPoints();
  std::vector<Eigen::Vector2d> matrixToVectors(const Eigen::MatrixXd & ctrl_pts);

private:
  std::shared_ptr<nav2_costmap_2d::Costmap2D> edt_environment_;

  // ===== ESDF cache (OpenCV distanceTransform) =====
  cv::Mat esdf_cache_;         // 浮点距离场 (单位: 米)
  cv::Mat esdf_grad_x_;        // ESDF x方向梯度 (指向远离障碍物方向)
  cv::Mat esdf_grad_y_;        // ESDF y方向梯度
  double esdf_origin_x_{0.0};  // 地图原点 x (世界坐标)
  double esdf_origin_y_{0.0};  // 地图原点 y
  double esdf_resolution_{0.05};
  int esdf_size_x_{0};
  int esdf_size_y_{0};
  bool esdf_valid_{false};

  void buildESDFCache();  // 构建 ESDF 缓存

  // main input
  Eigen::MatrixXd control_points_;          // B-spline control points, N x dim
  double bspline_interval_;                 // B-spline knot span
  Eigen::Vector2d end_pt_;                  // end of the trajectory
  int dim_;                                 // dimension of the B-spline
                                            //
  std::vector<Eigen::Vector2d> guide_pts_;  // geometric guiding path points, N-6
  std::vector<Eigen::Vector2d> waypoints_;  // waypts constraints
  std::vector<int> waypt_idx_;              // waypts constraints index
                                            //
  int max_num_id_, max_time_id_;            // stopping criteria
  int cost_function_;                       // used to determine objective function
  bool dynamic_;                            // moving obstacles ?
  double start_time_;                       // global time for moving obstacles

  /* optimization parameters */
  int order_;                     // bspline degree
  double lambda1_;                // jerk smoothness weight
  double lambda2_;                // distance weight
  double lambda3_;                // feasibility weight
  double lambda4_;                // end point weight
  double lambda5_;                // guide cost weight
  double lambda6_;                // visibility cost weight
  double lambda7_;                // waypoints cost weight
  double lambda8_;                // acc smoothness
                                  //
  double dist0_;                  // safe distance
  double max_vel_, max_acc_;      // dynamic limits
  double visib_min_;              // threshold of visibility
  double wnl_;                    //
  double dlmin_;                  //
                                  //
  int algorithm1_;                // optimization algorithms for quadratic cost
  int algorithm2_;                // optimization algorithms for general cost
  int max_iteration_num_[4];      // stopping criteria that can be used
  double max_iteration_time_[4];  // stopping criteria that can be used

  /* intermediate variables */
  /* buffer for gradient of cost function, to avoid repeated allocation and
   * release of memory */
  std::vector<Eigen::Vector2d> g_q_;
  std::vector<Eigen::Vector2d> g_smoothness_;
  std::vector<Eigen::Vector2d> g_distance_;
  std::vector<Eigen::Vector2d> g_feasibility_;
  std::vector<Eigen::Vector2d> g_endpoint_;
  std::vector<Eigen::Vector2d> g_guide_;
  std::vector<Eigen::Vector2d> g_waypoints_;

  int start_fixed_{2};  // 起始端固定控制点数 (只固定P0,P1保证起点, P2/P3参与优化)
  int variable_num_;    // optimization variables
  int iter_num_;        // iteration of the solver
  std::vector<double> best_variable_;  //
  double min_cost_;                    //

  std::vector<Eigen::Vector2d> block_pts_;  // blocking points to compute visibility

  /* cost function */
  /* calculate each part of cost function with control points q as input */

  static double costFunction(
    const std::vector<double> & x, std::vector<double> & grad, void * func_data);
  void combineCost(const std::vector<double> & x, std::vector<double> & grad, double & cost);

  // q contains all control points
  void calcSmoothnessCost(
    const std::vector<Eigen::Vector2d> & q, double & cost, std::vector<Eigen::Vector2d> & gradient);
  void calcDistanceCost(
    const std::vector<Eigen::Vector2d> & q, double & cost, std::vector<Eigen::Vector2d> & gradient);
  void calcFeasibilityCost(
    const std::vector<Eigen::Vector2d> & q, double & cost, std::vector<Eigen::Vector2d> & gradient);
  void calcEndpointCost(
    const std::vector<Eigen::Vector2d> & q, double & cost, std::vector<Eigen::Vector2d> & gradient);
  void calcGuideCost(
    const std::vector<Eigen::Vector2d> & q, double & cost, std::vector<Eigen::Vector2d> & gradient);
  void calcVisibilityCost(
    const std::vector<Eigen::Vector2d> & q, double & cost, std::vector<Eigen::Vector2d> & gradient);
  void calcWaypointsCost(
    const std::vector<Eigen::Vector2d> & q, double & cost, std::vector<Eigen::Vector2d> & gradient);
  void calcViewCost(
    const std::vector<Eigen::Vector2d> & q, double & cost, std::vector<Eigen::Vector2d> & gradient);
  bool isQuadratic();

  /* for benckmark evaluation only */
public:
  std::vector<double> vec_cost_;
  std::vector<double> vec_time_;
  rclcpp::Time time_start_;

  void getCostCurve(std::vector<double> & cost, std::vector<double> & time)
  {
    cost = vec_cost_;
    time = vec_time_;
  }

  typedef std::unique_ptr<BsplineOptimizer> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace fast_planner
#endif  // BSPLINE_OPTIMIZER_HPP_
