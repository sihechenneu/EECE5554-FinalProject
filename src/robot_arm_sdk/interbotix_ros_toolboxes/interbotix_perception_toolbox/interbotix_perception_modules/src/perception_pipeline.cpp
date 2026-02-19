#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include "interbotix_perception_modules/srv/filter_params.hpp"
#include "interbotix_perception_modules/msg/cluster_info.hpp"
#include "interbotix_perception_modules/srv/cluster_info_array.hpp"

using namespace std::placeholders;

typedef pcl::PointXYZRGB PointT;

namespace interbotix_perception_modules
{

class PerceptionPipelineNode : public rclcpp::Node
{
public:
  PerceptionPipelineNode()
  : Node("perception_pipeline"),
    enable_pipeline_(true),
    cloud_topic_("camera/depth/color/points"),
    voxel_leaf_size_(0.01f),
    x_filter_min_(-0.5f), y_filter_min_(-0.5f), z_filter_min_(0.0f),
    x_filter_max_(0.5f), y_filter_max_(0.5f), z_filter_max_(1.0f),
    plane_dist_thresh_(0.01f), ror_radius_search_(0.05f), cluster_tol_(0.02f),
    plane_max_iter_(100), ror_min_neighbors_(5), cluster_min_size_(50), cluster_max_size_(25000)
  {
    declare_parameter<bool>("enable_pipeline", true);
    declare_parameter<std::string>("cloud_topic", "camera/depth/color/points");
    declare_parameter<float>("voxel_leaf_size", 0.01f);
    declare_parameter<float>("x_filter_min", -0.5f);
    declare_parameter<float>("y_filter_min", -0.5f);
    declare_parameter<float>("z_filter_min", 0.0f);
    declare_parameter<float>("x_filter_max", 0.5f);
    declare_parameter<float>("y_filter_max", 0.5f);
    declare_parameter<float>("z_filter_max", 1.0f);
    declare_parameter<int>("plane_max_iter", 100);
    declare_parameter<float>("plane_dist_thresh", 0.01f);
    declare_parameter<float>("ror_radius_search", 0.05f);
    declare_parameter<int>("ror_min_neighbors", 5);
    declare_parameter<float>("cluster_tol", 0.02f);
    declare_parameter<int>("cluster_min_size", 50);
    declare_parameter<int>("cluster_max_size", 25000);

    enable_pipeline_ = get_parameter("enable_pipeline").as_bool();
    cloud_topic_ = get_parameter("cloud_topic").as_string();
    voxel_leaf_size_ = get_parameter("voxel_leaf_size").as_double();
    x_filter_min_ = get_parameter("x_filter_min").as_double();
    y_filter_min_ = get_parameter("y_filter_min").as_double();
    z_filter_min_ = get_parameter("z_filter_min").as_double();
    x_filter_max_ = get_parameter("x_filter_max").as_double();
    y_filter_max_ = get_parameter("y_filter_max").as_double();
    z_filter_max_ = get_parameter("z_filter_max").as_double();
    plane_max_iter_ = get_parameter("plane_max_iter").as_int();
    plane_dist_thresh_ = get_parameter("plane_dist_thresh").as_double();
    ror_radius_search_ = get_parameter("ror_radius_search").as_double();
    ror_min_neighbors_ = get_parameter("ror_min_neighbors").as_int();
    cluster_tol_ = get_parameter("cluster_tol").as_double();
    cluster_min_size_ = get_parameter("cluster_min_size").as_int();
    cluster_max_size_ = get_parameter("cluster_max_size").as_int();

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, 1, std::bind(&PerceptionPipelineNode::cloud_cb, this, _1));
    pub_pc_obj_ = create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud/objects", 1);
    pub_pc_filter_ = create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud/filtered", 1);
    pub_marker_obj_ = create_publisher<visualization_msgs::msg::Marker>("markers/objects", 50);
    pub_marker_crop_ = create_publisher<visualization_msgs::msg::Marker>("markers/crop_box", 1);

    srv_set_params_ = create_service<interbotix_perception_modules::srv::FilterParams>(
      "set_filter_params", std::bind(&PerceptionPipelineNode::srv_set_filter_params, this, _1, _2));
    srv_get_clusters_ = create_service<interbotix_perception_modules::srv::ClusterInfoArray>(
      "get_cluster_positions", std::bind(&PerceptionPipelineNode::srv_get_cluster_positions, this, _1, _2));
    srv_enable_ = create_service<std_srvs::srv::SetBool>(
      "enable_pipeline", std::bind(&PerceptionPipelineNode::srv_enable_pipeline, this, _1, _2));

    marker_obj_.type = visualization_msgs::msg::Marker::SPHERE;
    marker_obj_.action = visualization_msgs::msg::Marker::ADD;
    marker_obj_.lifetime = rclcpp::Duration(0, 100000000);
    marker_obj_.pose.orientation.w = 1.0;
    marker_obj_.color.a = 1.0;
    marker_obj_.scale.x = marker_obj_.scale.y = marker_obj_.scale.z = 0.01;

    marker_crop_.type = visualization_msgs::msg::Marker::CUBE;
    marker_crop_.action = visualization_msgs::msg::Marker::ADD;
    marker_crop_.pose.orientation.w = 1.0;
    marker_crop_.color.g = 1.0;
    marker_crop_.color.a = 0.3;
  }

private:
  void cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    input_ = msg;
    if (enable_pipeline_)
      perception_pipeline();
  }

  void perception_pipeline()
  {
    if (!input_) return;
    pcl::PointCloud<PointT>::Ptr cloud_raw(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*input_, *cloud_raw);
    if (cloud_raw->empty()) return;

    pcl::PointCloud<PointT>::Ptr cloud_voxel(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_cropped(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> crop;
    crop.setInputCloud(cloud_raw);
    crop.setMin(Eigen::Vector4f(x_filter_min_, y_filter_min_, z_filter_min_, 0));
    crop.setMax(Eigen::Vector4f(x_filter_max_, y_filter_max_, z_filter_max_, 0));
    crop.filter(*cloud_cropped);
    if (cloud_cropped->empty()) return;

    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud_cropped);
    vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    vg.filter(*cloud_voxel);
    if (cloud_voxel->size() < 4) return;

    sensor_msgs::msg::PointCloud2 out_filter;
    pcl::toROSMsg(*cloud_voxel, out_filter);
    pub_pc_filter_->publish(out_filter);

    marker_crop_.header = input_->header;
    marker_crop_.pose.position.x = (x_filter_min_ + x_filter_max_) / 2.0;
    marker_crop_.pose.position.y = (y_filter_min_ + y_filter_max_) / 2.0;
    marker_crop_.pose.position.z = (z_filter_min_ + z_filter_max_) / 2.0;
    marker_crop_.scale.x = x_filter_max_ - x_filter_min_;
    marker_crop_.scale.y = y_filter_max_ - y_filter_min_;
    marker_crop_.scale.z = z_filter_max_ - z_filter_min_;
    pub_marker_crop_->publish(marker_crop_);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(plane_max_iter_);
    seg.setDistanceThreshold(plane_dist_thresh_);
    seg.setInputCloud(cloud_voxel);
    seg.segment(*inliers, *coeffs);

    pcl::PointCloud<PointT>::Ptr cloud_no_plane(new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> ei;
    ei.setInputCloud(cloud_voxel);
    ei.setIndices(inliers);
    ei.setNegative(true);
    ei.filter(*cloud_no_plane);
    if (cloud_no_plane->empty()) return;

    pcl::PointCloud<PointT>::Ptr cloud_no_noise(new pcl::PointCloud<PointT>);
    pcl::RadiusOutlierRemoval<PointT> ror;
    ror.setInputCloud(cloud_no_plane);
    ror.setRadiusSearch(ror_radius_search_);
    ror.setMinNeighborsInRadius(ror_min_neighbors_);
    ror.setKeepOrganized(false);
    ror.filter(*cloud_no_noise);
    if (cloud_no_noise->empty()) return;

    sensor_msgs::msg::PointCloud2 out_obj;
    pcl::toROSMsg(*cloud_no_noise, out_obj);
    pub_pc_obj_->publish(out_obj);

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud_no_noise);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cluster_tol_);
    ec.setMinClusterSize(cluster_min_size_);
    ec.setMaxClusterSize(cluster_max_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_no_noise);
    ec.extract(cluster_indices);

    cluster_info_vector_.clear();
    int j = 0;
    for (const auto & indices : cluster_indices) {
      pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
      for (int idx : indices.indices)
        cloud_cluster->push_back((*cloud_no_noise)[idx]);
      cloud_cluster->width = cloud_cluster->size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      PointT cntrd, min_pt;
      pcl::computeCentroid(*cloud_cluster, cntrd);
      min_pt = (*cloud_cluster)[0];
      for (size_t i = 1; i < cloud_cluster->size(); i++) {
        if ((*cloud_cluster)[i].z < min_pt.z)
          min_pt = (*cloud_cluster)[i];
      }

      marker_obj_.id = j;
      marker_obj_.header = input_->header;
      marker_obj_.pose.position.x = cntrd.x;
      marker_obj_.pose.position.y = cntrd.y;
      marker_obj_.pose.position.z = cntrd.z;
      marker_obj_.color.r = cntrd.r / 255.0;
      marker_obj_.color.g = cntrd.g / 255.0;
      marker_obj_.color.b = cntrd.b / 255.0;
      pub_marker_obj_->publish(marker_obj_);

      interbotix_perception_modules::msg::ClusterInfo ci;
      ci.frame_id = input_->header.frame_id;
      ci.position.x = cntrd.x;
      ci.position.y = cntrd.y;
      ci.position.z = cntrd.z;
      ci.color.r = cntrd.r;
      ci.color.g = cntrd.g;
      ci.color.b = cntrd.b;
      ci.min_z_point.x = min_pt.x;
      ci.min_z_point.y = min_pt.y;
      ci.min_z_point.z = min_pt.z;
      ci.num_points = static_cast<int32_t>(cloud_cluster->size());
      cluster_info_vector_.push_back(ci);
      j++;
    }
  }

  void srv_set_filter_params(
    const std::shared_ptr<interbotix_perception_modules::srv::FilterParams::Request> req,
    std::shared_ptr<interbotix_perception_modules::srv::FilterParams::Response>)
  {
    voxel_leaf_size_ = req->voxel_leaf_size;
    x_filter_min_ = req->x_filter_min;
    x_filter_max_ = req->x_filter_max;
    y_filter_min_ = req->y_filter_min;
    y_filter_max_ = req->y_filter_max;
    z_filter_min_ = req->z_filter_min;
    z_filter_max_ = req->z_filter_max;
    plane_max_iter_ = req->plane_max_iter;
    plane_dist_thresh_ = req->plane_dist_thresh;
    ror_radius_search_ = req->ror_radius_search;
    ror_min_neighbors_ = req->ror_min_neighbors;
    cluster_tol_ = req->cluster_tol;
    cluster_min_size_ = req->cluster_min_size;
    cluster_max_size_ = req->cluster_max_size;
  }

  void srv_get_cluster_positions(
    const std::shared_ptr<interbotix_perception_modules::srv::ClusterInfoArray::Request>,
    std::shared_ptr<interbotix_perception_modules::srv::ClusterInfoArray::Response> res)
  {
    if (!enable_pipeline_)
      perception_pipeline();
    res->clusters = cluster_info_vector_;
  }

  void srv_enable_pipeline(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response>)
  {
    enable_pipeline_ = req->data;
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc_obj_, pub_pc_filter_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_obj_, pub_marker_crop_;
  rclcpp::Service<interbotix_perception_modules::srv::FilterParams>::SharedPtr srv_set_params_;
  rclcpp::Service<interbotix_perception_modules::srv::ClusterInfoArray>::SharedPtr srv_get_clusters_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_enable_;

  sensor_msgs::msg::PointCloud2::SharedPtr input_;
  std::vector<interbotix_perception_modules::msg::ClusterInfo> cluster_info_vector_;
  visualization_msgs::msg::Marker marker_obj_, marker_crop_;

  bool enable_pipeline_;
  std::string cloud_topic_;
  float voxel_leaf_size_, x_filter_min_, y_filter_min_, z_filter_min_;
  float x_filter_max_, y_filter_max_, z_filter_max_;
  float plane_dist_thresh_, ror_radius_search_, cluster_tol_;
  int plane_max_iter_, ror_min_neighbors_, cluster_min_size_, cluster_max_size_;
};

}  // namespace interbotix_perception_modules

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<interbotix_perception_modules::PerceptionPipelineNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
