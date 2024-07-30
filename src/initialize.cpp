#include "initialize.hpp"

Initializer::Initializer()//コンストラクタ
    : Node("initialize")//ノードの名前
  {
    //サブスクライブorパブリッシュ = this -> 関数名＜型＞（”トピック” + etc）
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/velodyne_points", 10, std::bind(&Initializer::topic_callback, this, _1));
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_plane", 10);
  }

void calculate_angles_to_align_with_z(const Eigen::Vector3f& normal, float &pitch, float &roll)
{
    // Normalize the vector
    Eigen::Vector3f n = normal.normalized();

    // Calculate the pitch and roll needed to align the normal vector with the z-axis
    pitch = std::asin(-n[0]); // rotation around y-axis
    roll = std::atan2(n[1], n[2]); // rotation around x-axis

    // Convert to degrees
    roll = roll * 180.0 / M_PI;
    pitch = pitch * 180.0 / M_PI;

    double rounded_roll = std::round(roll * 100) / 100; 
    double rounded_pitch = std::round(pitch * 10) / 10; 

    std::cout << "Roll: " << rounded_roll << " degrees" << std::endl;
    std::cout << "Pitch: " << rounded_pitch << " degrees" << std::endl;
}

void Initializer::topic_callback(const sensor_msgs::msg::PointCloud2 &msg) const
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    //ROSからPCLに変換
    pcl::fromROSMsg(msg, *cloud);

    //CropBoxによる切り取り
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setMin(Eigen::Vector4f(1.0, -2.0, -4.0, 1.0));
    crop.setMax(Eigen::Vector4f(10.0, 2.0, 4.0, 1.0)); 
    crop.setInputCloud(cloud);
    crop.filter(*cloud_cropped);

    //RANSACの準備
    pcl::SACSegmentation<pcl::PointXYZ> seg;                              // segmentationしてくれるやつ
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);                // 取り出す点群
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); // 係数？

    // ここからRANSACするよ
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE); // 検出対象（いろいろ選べるよ）
    seg.setMethodType(pcl::SAC_RANSAC);    // 検出手法（いろいろ選べるよ）
    seg.setMaxIterations(200);             //　試す回数
    seg.setDistanceThreshold(0.15);
    // ここまでsegmentation用のオプション設定

    seg.setInputCloud(cloud_cropped);//点群の入力
    seg.segment(*inliers, *coefficients); // segmentation

    Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

    // 平面方程式の係数を出力
    // RCLCPP_INFO(this->get_logger(), "平面方程式: %f x + %f y + %f z + %f = 0", 
    //             coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);

    float pitch, roll;
    calculate_angles_to_align_with_z(plane_normal, pitch, roll);

    // 平面から原点までの距離を算出
    float A = coefficients->values[0];
    float B = coefficients->values[1];
    float C = coefficients->values[2];
    float D = coefficients->values[3];
    float height_of_velodyne = std::fabs(D) / std::sqrt(A * A + B * B + C * C);
    std::cout << "height of velodyne: " <<  round(height_of_velodyne * 100) / 100  << " m" << std::endl;

    pcl::ExtractIndices<pcl::PointXYZ> extract; // segmentした点群の抜き出し
    extract.setInputCloud(cloud);
    //extract.setInputCloud(rotated_cloud);
    extract.setInputCloud(cloud_cropped);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_filtered);

    // 回転行列の作成
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    //回転角の設定(後にIMUのデータを読み込めるようにしたい)
    float thetaX = roll * (M_PI / 180);
    float thetaY = pitch * (M_PI / 180); 
    float thetaZ = 0 * (M_PI / 180);
    //回転させます
    transform.rotate(Eigen::AngleAxisf(thetaX, Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf(thetaY, Eigen::Vector3f::UnitY()));
    transform.rotate(Eigen::AngleAxisf(thetaZ, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*cloud_filtered, *rotated_cloud, transform);

    float z_offset = height_of_velodyne;  // 平面までの距離(＝velodyneの高さを算出)
    for (auto& point : rotated_cloud->points) {
      point.z += z_offset;                // 足します
    }

    //PCLからROSに変換
    sensor_msgs::msg::PointCloud2 sensor_msg;
    pcl::toROSMsg(*rotated_cloud, sensor_msg);
    sensor_msg.header.frame_id = msg.header.frame_id;
    publisher_->publish(sensor_msg);
}