#include "pcl/registration/icp.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/io.h"


double Deg2Rad(double deg) {
    return deg * M_PI / 180.0;
}
// 欧拉角转旋转矩阵
Eigen::Matrix4d CreateTfMatrix(double x, double y, double z, double yaw, double pitch, double roll)
{
    yaw = Deg2Rad(yaw);
    pitch = Deg2Rad(pitch);
    roll = Deg2Rad(roll);
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);
    return T;
}


int main(int argc, char const *argv[])
{
    // 创建一个点云
    // 源点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
    // 需要配准的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_source->width = 255;
    cloud_source->height = 1;
    cloud_source->is_dense = true;
    cloud_source->points.resize(cloud_source->width * cloud_source->height);
    for (auto& point : cloud_source->points)
    {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
    
    cloud_target->width = 255;
    cloud_target->height = 1;
    cloud_target->is_dense = true;
    cloud_target->points.resize(cloud_target->width * cloud_target->height);
    for (size_t i = 0; i < cloud_source->points.size(); i++)
    {
        cloud_target->points[i].x = cloud_source->points[i].x + 10.0f;
        cloud_target->points[i].y = cloud_source->points[i].y + 19.0f;
        cloud_target->points[i].z = cloud_source->points[i].z + 1.0f;
    }
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_source);
    icp.setInputTarget(cloud_target);

    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-8);
    // 在执行 icp.align(Final) 时，ICP算法会将源点云（cloud_source）根据计算出的变换矩阵进行变换，并将变换后的点云存储在 Final 中。
    // Final 中的点云数据是源点云经过配准后的结果，即源点云与目标点云（cloud_target）对齐后的点云。
    pcl::PointCloud<pcl::PointXYZ> Final;
    // 有两种配准方式，一种是直接配准，另一种是使用初始变换矩阵进行配准
    
    icp.align(Final);
    // icp.align(Final, initial_guess);


    if (icp.hasConverged())
    {
        std::cout << "ICP converged." << std::endl;
        std::cout << "The score is " << icp.getFitnessScore() << std::endl;
        std::cout << "Transformation matrix:" << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;
    }
    else
    {
        std::cout << "ICP did not converge." << std::endl;
        return (-1);
    }
    


    return 0;
}
