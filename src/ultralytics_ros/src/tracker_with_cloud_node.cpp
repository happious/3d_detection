#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>


#include <custom_msgs/TrackingResultArray.h>
#include <custom_msgs/TrackingResult.h>
#include <custom_msgs/Detection3DArray.h>
#include <custom_msgs/Detection3D.h>
#include <custom_msgs/BoundingBox3D.h>
#include <custom_msgs/ObjectHypothesis.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <pcl/pcl_config.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <string>
#include <vector>
#include <deque>
#include <algorithm>
#include <sstream>
#include <limits>


// ===================== basic struct =====================
struct UV { int u; int v; bool valid; };

// ===================== Extrinsic Parameter =====================
class ExtrinsicConfig {
public:
  Eigen::Matrix3f Rt;
  Eigen::Vector3f tt;
  Eigen::Matrix3f Rt_T;
  Eigen::Matrix3f Rz180; // lidar flip

  bool load(ros::NodeHandle& pnh){
    std::vector<double> Rv, Tv;
    if(!pnh.getParam("R_cam2lidar", Rv) || Rv.size()!=9){
      ROS_ERROR("ERROR: Need Extrinsic Parameter R_cam2lidar (9 elems)");
      return false;
    }
    if(!pnh.getParam("T_cam2lidar", Tv) || Tv.size()!=3){
      ROS_ERROR("ERROR: Need Extrinsic Parameter T_cam2lidar (3 elems)");
      return false;
    }
    Rt = (Eigen::Matrix3f() <<
      (float)Rv[0], (float)Rv[1], (float)Rv[2],
      (float)Rv[3], (float)Rv[4], (float)Rv[5],
      (float)Rv[6], (float)Rv[7], (float)Rv[8]).finished();
    tt = Eigen::Vector3f((float)Tv[0], (float)Tv[1], (float)Tv[2]);

    Rt_T = Rt.transpose();
    Rz180 = (Eigen::Matrix3f() <<
       1,  0,  0,
       0, -1,  0,
       0,  0, -1).finished();
    return true;
  }
};

// ===================== mode D =====================
// p_C = Rz180 * ( Rt^T * (pL - tt) )
inline Eigen::Vector3f lidarToCam_modeD(const Eigen::Vector3f& pL, const ExtrinsicConfig& ex) {
  return ex.Rz180 * (ex.Rt_T * (pL - ex.tt));
}

// ===================== lonA + yaw0 =====================
// Ricoh THETA ERP 투영: lonA = atan2(-dx, dz), yaw=0
inline UV erpProject_lonA(const Eigen::Vector3f& pC, int W, int H){
  const float dx = pC.x(), dy = pC.y(), dz = pC.z();
  const float r  = std::sqrt(dx*dx + dz*dz);
  const float lonA = std::atan2(-dx, dz);
  const float lat  = std::atan2(dy, (r>1e-9f ? r : 1e-9f));

  int u = (int)((lonA + M_PI) / (2.0 * M_PI) * W);
  int v = (int)(((M_PI*0.5) - lat) / M_PI * H);

  if(u < 0)   u += W;
  if(u >= W)  u -= W;

  return {u, v, (v >= 0 && v < H)};
}

// ===================== wrap-around in U =====================
inline bool inBboxWrapU(int u, int v, int W, float cx, float cy, float bw, float bh, float dist_m){
  float shrink = 1.3f + 0.05f * dist_m;
  float bw_s = bw / shrink;
  float bh_s = bh / shrink;
  
  const float u0 = cx - bw_s*0.5f;
  const float u1 = cx + bw_s*0.5f;
  const float v0 = cy - bh_s*0.5f;
  const float v1 = cy + bh_s*0.5f;

  if(v < v0 || v > v1) return false;

  auto normU = [W](float uu){
    float x = std::fmod(uu, (float)W);
    if(x < 0) x += W;
    return x;
  };

  float a = normU(u0);
  float b = normU(u1);

  if(a <= b){
    return (u >= std::floor(a) && u <= std::ceil(b));
  }
  // wrap case
  return (u >= std::floor(a) && u < W) || (u >= 0 && u <= std::ceil(b));
}

// ===================== Visualization =====================
struct RGB { uint8_t r,g,b; };

inline RGB bandColor(float d, float near_m, float mid_m){
  if (d < near_m) return {255, 0,   0  };   // red
  if (d < mid_m)  return {255, 165, 0  };   // orange
  return            {0,   255, 0  };        // green
}

inline visualization_msgs::Marker makeTextMarker(
  int id, const Eigen::Vector3f& pos, float d_m,
  const RGB& c, const std::string& frame, const ros::Time& stamp,
  double scale_z, double lifetime)
{
  visualization_msgs::Marker m;
  m.header.frame_id = frame;
  m.header.stamp    = stamp;
  m.ns = "person_dist"; 
  m.id = id;
  m.type   = visualization_msgs::Marker::TEXT_VIEW_FACING;
  m.action = visualization_msgs::Marker::ADD;

  m.pose.position.x = pos.x();
  m.pose.position.y = pos.y();
  m.pose.position.z = pos.z() + 0.4;

  m.scale.z = scale_z;

  m.color.r = c.r / 255.0;
  m.color.g = c.g / 255.0;
  m.color.b = c.b / 255.0;
  m.color.a = 1.0;

  m.lifetime = ros::Duration(lifetime);

  std::ostringstream ss;
  ss << std::fixed << std::setprecision(1) << d_m << " m";
  m.text = ss.str();

  return m;
}

inline pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorizeCluster(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& cl, const RGB& col)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGB>());
  out->reserve(cl->size());
  for(const auto& p : cl->points){
    pcl::PointXYZRGB q;
    q.x = p.x; q.y = p.y; q.z = p.z;
    q.r = col.r; q.g = col.g; q.b = col.b;
    out->push_back(q);
  }
  return out;
}

// ===================== Node =====================
class TrackerWithCloudERP {
public:
  TrackerWithCloudERP(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
  {
    // ----------------- parameters -----------------
    pnh_.param<std::string>("lidar_topic", lidar_topic_, "/livox/lidar");
    pnh_.param<std::string>("tracking_result_topic", tracking_topic_, "/tracking_result");
    pnh_.param<std::string>("tracking_3d_result_topic", tracking_3d_topic_, "/tracking_3d_result");
    pnh_.param<int>("image_width",  img_w_, 1920);
    pnh_.param<int>("image_height", img_h_,  960);
    pnh_.param<int>("erp_margin", erp_m_, 20);

    // clustering
    pnh_.param<float>("voxel_leaf_size", voxel_leaf_size_, 0.03f);
    pnh_.param<float>("cluster_tolerance", cluster_tol_, 0.20f);
    pnh_.param<int>("min_cluster_size", min_cluster_, 10);
    pnh_.param<int>("max_cluster_size", max_cluster_, 15000);

    // tracking sync
    pnh_.param<double>("tracking_max_age", tracking_max_age_sec_, 5.0);

    // visualization
    pnh_.param<std::string>("colored_cloud_topic", colored_cloud_topic_, std::string("/people_cloud_colored"));
    pnh_.param<std::string>("marker_topic",        marker_topic_,        std::string("/detection_marker_dist"));
    pnh_.param<std::string>("box_marker_topic",    box_marker_topic_,    std::string("/detection_marker_box"));
    pnh_.param<double>("band_near_m", band_near_m_, 4.0);
    pnh_.param<double>("band_mid_m",  band_mid_m_,  8.0);
    pnh_.param<double>("text_scale_z", text_scale_z_, 0.6);
    pnh_.param<double>("text_lifetime", text_lifetime_, 0.2);

    // ground removal
    pnh_.param<bool>("remove_ground", remove_ground_, true);
    pnh_.param<double>("ground_dist_thresh", ground_dist_thresh_, 0.10);
    pnh_.param<double>("ground_max_tilt_deg", ground_max_tilt_deg_, 15.0);
    pnh_.param<int>("ground_max_iterations", ground_max_iterations_, 200);
    pnh_.param<double>("ground_probability", ground_probability_, 0.99);
    pnh_.param<int>("ground_min_inliers", ground_min_inliers_, 200);

    // outlier removal
    pnh_.param<bool>("remove_outliers", remove_outliers_, true);
    pnh_.param<int>("sor_mean_k", sor_mean_k_, 20);
    pnh_.param<double>("sor_stddev_mul", sor_stddev_mul_, 1.0);

    // DBSCAN
    pnh_.param<int>("dbscan_min_pts", dbscan_min_pts_, 5);
    pnh_.param<double>("dbscan_eps", dbscan_eps_, 1.0);

    // person class id list (optional)
    pnh_.getParam("person_class_ids", person_class_ids_);
    if(person_class_ids_.empty()){
      // default: 1 == person
      person_class_ids_.push_back(1);
    }

    // extrinsic
    if(!ex_.load(pnh_)) throw std::runtime_error("Extrinsic load failed");

    // publishers
    pub_det3d_        = nh_.advertise<custom_msgs::Detection3DArray>(tracking_3d_topic_, 1, true);
    pub_cloud_raw_    = nh_.advertise<sensor_msgs::PointCloud2>("detection_cloud", 1, true);
    pub_cloud_col_    = nh_.advertise<sensor_msgs::PointCloud2>(colored_cloud_topic_, 1, true);
    pub_markers_dist_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1, true);
    pub_markers_box_  = nh_.advertise<visualization_msgs::MarkerArray>(box_marker_topic_, 1, true);

    // subscribers
    sub_pc_       = nh_.subscribe(lidar_topic_, 1,  &TrackerWithCloudERP::pcCb, this);
    sub_tracking_ = nh_.subscribe(tracking_topic_, 50, &TrackerWithCloudERP::trackingCb, this);

    ROS_INFO_STREAM("[tracker_with_cloud_erp_ros1] ERP WxH="<<img_w_<<"x"<<img_h_
      << ", voxel="<<voxel_leaf_size_
      << ", DBSCAN eps="<<dbscan_eps_
      << ", dbscan_min_pts="<<dbscan_min_pts_
      << ", min_cluster="<<min_cluster_<<", max_cluster="<<max_cluster_
      << ", tracking_max_age="<<tracking_max_age_sec_<<"s"
      << ", remove_ground="<< (remove_ground_?"true":"false")
      << ", ground_dist_thresh="<<ground_dist_thresh_
      << ", ground_max_tilt_deg="<<ground_max_tilt_deg_
      << ", ground_max_iterations="<<ground_max_iterations_
      << ", ground_probability="<<ground_probability_
      << ", ground_min_inliers="<<ground_min_inliers_
      << ", remove_outliers="<<(remove_outliers_?"true":"false")
      << ", sor_mean_k="<<sor_mean_k_
      << ", sor_stddev_mul="<<sor_stddev_mul_
      << ", colored_cloud_topic="<<colored_cloud_topic_
      << ", marker_topic="<<marker_topic_
      << ", box_marker_topic="<<box_marker_topic_);
  }

private:
  // ---------------- core members ----------------
  ros::NodeHandle nh_, pnh_;
  std::string lidar_topic_, tracking_topic_, tracking_3d_topic_;
  int   img_w_{1920}, img_h_{960};
  float voxel_leaf_size_{0.05f}, cluster_tol_{0.20f};
  int   min_cluster_{10}, max_cluster_{15000};
  double tracking_max_age_sec_{5.0};

  std::string colored_cloud_topic_, marker_topic_, box_marker_topic_;
  double band_near_m_{4.0}, band_mid_m_{8.0};
  double text_scale_z_{0.6}, text_lifetime_{0.2};
  
  int erp_m_{20};

  ros::Subscriber sub_pc_, sub_tracking_;
  ros::Publisher  pub_det3d_, pub_cloud_raw_, pub_cloud_col_;
  ros::Publisher  pub_markers_dist_, pub_markers_box_;

  ExtrinsicConfig ex_;

  // tracking buffer
  std::deque<custom_msgs::TrackingResultArray> track_buf_;
  int  track_buf_max_{50};
  bool has_tracking_{false};

  // ground
  bool   remove_ground_{true};
  double ground_dist_thresh_{0.10};
  double ground_max_tilt_deg_{25.0};
  int    ground_max_iterations_{200};
  double ground_probability_{0.99};
  int    ground_min_inliers_{200};

  // outliers
  bool   remove_outliers_{true};
  int    sor_mean_k_{20};
  double sor_stddev_mul_{1.0};

  // DBSCAN
  int    dbscan_min_pts_{5};
  double dbscan_eps_{1.0};

  // person filter
  std::vector<int> person_class_ids_;

  // ===================== basic PCL utils =====================
  pcl::PointCloud<pcl::PointXYZ>::Ptr
  downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>());
    if(!cloud || cloud->empty()) return out;
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    vg.filter(*out);
    return out;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr
  removeOutliers(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
    if(!remove_outliers_ || !cloud || cloud->empty()) return cloud;

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(std::max(1, sor_mean_k_));
    sor.setStddevMulThresh(sor_stddev_mul_);
    sor.filter(*filtered);
    return filtered;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr
  removeGroundRANSAC(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
    if(!remove_ground_ || !cloud || cloud->empty()) return cloud;

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(ground_dist_thresh_);
    seg.setMaxIterations(std::max(1, ground_max_iterations_));
#if PCL_VERSION_COMPARE(>=,1,7,0)
    seg.setProbability(ground_probability_);
#endif
    seg.setInputCloud(cloud);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    seg.segment(*inliers, *coeff);

    if(inliers->indices.empty()){
      ROS_WARN_THROTTLE(1.0, "[ground] no plane found; skip removal");
      return cloud;
    }

    if((int)inliers->indices.size() < ground_min_inliers_){
      ROS_WARN_THROTTLE(1.0,
        "[ground] inliers=%zu < ground_min_inliers=%d; skip removal",
        inliers->indices.size(), ground_min_inliers_);
      return cloud;
    }

    if(coeff->values.size() < 4){
      ROS_WARN_THROTTLE(1.0, "[ground] invalid plane coeff size=%zu; skip removal",
                        coeff->values.size());
      return cloud;
    }

    Eigen::Vector3f n(coeff->values[0], coeff->values[1], coeff->values[2]);
    if(!std::isfinite(n.norm()) || n.norm() < 1e-3f){
      ROS_WARN_THROTTLE(1.0, "[ground] plane normal invalid; skip removal");
      return cloud;
    }
    n.normalize();
    Eigen::Vector3f z(0.f, 0.f, 1.f);
    double angle_deg = std::acos(std::fabs(n.dot(z))) * 180.0 / M_PI;

    if(angle_deg > ground_max_tilt_deg_){
      ROS_WARN_THROTTLE(1.0,
        "[ground] plane tilt=%.1f deg > max_tilt=%.1f → skip removal",
        angle_deg, ground_max_tilt_deg_);
      return cloud;
    }

    ROS_DEBUG_THROTTLE(1.0,
      "[ground] plane ok: tilt=%.1f deg, inliers=%zu / %zu",
      angle_deg, inliers->indices.size(), cloud->size());

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr noground(new pcl::PointCloud<pcl::PointXYZ>());
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true); // remove plane inliers
    extract.filter(*noground);
    return noground;
  }

  // ===================== DBSCAN clustering =====================
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
  densityClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    if(!cloud || cloud->empty()) return clusters;

    const int n = static_cast<int>(cloud->size());
    if(n == 0) return clusters;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(cloud);

    const int UNVISITED = -1;
    const int NOISE     = -2;
    std::vector<int> labels(n, UNVISITED);
    int current_cluster_id = 0;

    std::vector<int> neighbors;
    std::vector<float> sqr_dists;

    auto regionQuery = [&](int idx, std::vector<int>& out_idx){
      out_idx.clear();
      neighbors.clear();
      sqr_dists.clear();
      tree->radiusSearch((*cloud)[idx], dbscan_eps_, neighbors, sqr_dists);
      out_idx = neighbors;
    };

    auto expandCluster = [&](int seed_idx, int cluster_id, const std::vector<int>& seed_neighbors){
      labels[seed_idx] = cluster_id;
      std::vector<int> queue(seed_neighbors.begin(), seed_neighbors.end());

      for(size_t qi = 0; qi < queue.size(); ++qi){
        int pt_idx = queue[qi];
        if(labels[pt_idx] == NOISE){
          labels[pt_idx] = cluster_id;
        }
        if(labels[pt_idx] != UNVISITED) continue;

        labels[pt_idx] = cluster_id;

        std::vector<int> nbrs;
        regionQuery(pt_idx, nbrs);
        if((int)nbrs.size() >= dbscan_min_pts_){
          queue.insert(queue.end(), nbrs.begin(), nbrs.end());
        }
      }
    };

    for(int i = 0; i < n; ++i){
      if(labels[i] != UNVISITED) continue;

      std::vector<int> neigh;
      regionQuery(i, neigh);

      if((int)neigh.size() < dbscan_min_pts_){
        labels[i] = NOISE;
        continue;
      }

      int cid = current_cluster_id++;
      expandCluster(i, cid, neigh);
    }

    clusters.reserve(current_cluster_id);
    for(int cid = 0; cid < current_cluster_id; ++cid){
      pcl::PointCloud<pcl::PointXYZ>::Ptr cl(new pcl::PointCloud<pcl::PointXYZ>());
      for(int i = 0; i < n; ++i){
        if(labels[i] == cid){
          cl->push_back((*cloud)[i]);
        }
      }
      int sz = static_cast<int>(cl->size());
      if(sz < std::max(1, min_cluster_) ||
         sz > std::max(min_cluster_+1, max_cluster_)){
        continue;
      }
      clusters.push_back(cl);
    }

    return clusters;
  }

  // ===================== OBB + Detection3D 채우기 =====================
  void fill3DBbox(custom_msgs::Detection3D& det,
                  const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster,
                  int track_id,
                  float score)
  {
    if(!cluster || cluster->empty()) return;

    // 1) centroid
    Eigen::Vector4f centroid4;
    pcl::compute3DCentroid(*cluster, centroid4);
    Eigen::Vector3f centroid = centroid4.head<3>();

    // 2) yaw 추정 (간단: x,y 기준)
    float yaw = std::atan2(centroid.y(), centroid.x());

    // 3) 회전행렬 (world -> yaw-aligned 로테이션)
    Eigen::Affine3f tf = Eigen::Affine3f::Identity();
    tf.rotate(Eigen::AngleAxisf(-yaw, Eigen::Vector3f::UnitZ()));

    // 4) 클러스터를 yaw 보정 좌표계로 변환
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
    aligned->reserve(cluster->size());
    for(const auto& p : cluster->points){
      Eigen::Vector3f v = tf * Eigen::Vector3f(p.x, p.y, p.z);
      pcl::PointXYZ q; q.x = v.x(); q.y = v.y(); q.z = v.z();
      aligned->push_back(q);
    }

    // 5) AABB 계산 (보정 좌표계에서)
    pcl::PointXYZ min_p, max_p;
    pcl::getMinMax3D(*aligned, min_p, max_p);

    Eigen::Vector3f half((max_p.x + min_p.x)*0.5f,
                         (max_p.y + min_p.y)*0.5f,
                         (max_p.z + min_p.z)*0.5f);

    // 6) 다시 원래 좌표계로 center 복귀
    Eigen::Vector3f center = tf.inverse() * half;

    // 7) orientation (yaw 기반 quaternion)
    Eigen::Quaternionf q(tf.inverse().rotation());

    // ========== custom_msgs::Detection3D 출력 ==========
    det.bbox.center.position.x = center.x();
    det.bbox.center.position.y = center.y();
    det.bbox.center.position.z = center.z();

    det.bbox.center.orientation.x = q.x();
    det.bbox.center.orientation.y = q.y();
    det.bbox.center.orientation.z = q.z();
    det.bbox.center.orientation.w = q.w();
    
    float sx = (max_p.x - min_p.x);
    float sy = (max_p.y - min_p.y);
    float sz = (max_p.z - min_p.z);
    
    if (sy > sx) sy = sx;

    det.bbox.size_x = sx;
    det.bbox.size_y = sy;
    det.bbox.size_z = sz;

    det.hypothesis.class_id = track_id;
    det.hypothesis.score    = score;
  }

  // 3D 박스 마커
  visualization_msgs::Marker makeBoxMarker(
      int id, const custom_msgs::Detection3D& d,
      const RGB& col, const std::string& ns)
  {
    visualization_msgs::Marker m;
    m.header = d.header;
    m.ns = ns; 
    m.id = id;
    m.type   = visualization_msgs::Marker::CUBE;
    m.action = visualization_msgs::Marker::ADD;

    m.pose   = d.bbox.center;
    m.scale.x = d.bbox.size_x;
    m.scale.y = d.bbox.size_y;
    m.scale.z = d.bbox.size_z;

    m.color.r = col.r / 255.0;
    m.color.g = col.g / 255.0;
    m.color.b = col.b / 255.0;
    m.color.a = 0.35;

    m.lifetime = ros::Duration(0.0);
    return m;
  }

  bool isPersonTrack(const custom_msgs::TrackingResult& tr) const {
    // class_id가 person_class_ids_ 안에 있는지 확인
    if(person_class_ids_.empty()) return true;
    return std::find(person_class_ids_.begin(), person_class_ids_.end(), tr.class_id)
           != person_class_ids_.end();
  }

  // ===================== tracking 콜백 =====================
  void trackingCb(const custom_msgs::TrackingResultArray::ConstPtr& msg){
    track_buf_.push_back(*msg);
    while((int)track_buf_.size() > track_buf_max_) track_buf_.pop_front();
    has_tracking_ = true;
  }

  // ===================== LiDAR 콜백 =====================
  void pcCb(const sensor_msgs::PointCloud2ConstPtr& pc_msg){
    if(!has_tracking_) return;

    const ros::Time tL = pc_msg->header.stamp;

    // 1) tracking buffer에서 가장 가까운 timestamp 찾기
    int idx0 = -1;
    int idx1 = -1;
    
    for (int i = 0; i < (int)track_buf_.size(); ++i) {
        ros::Time tt = track_buf_[i].header.stamp;
        
        if (tt <= tL) idx0 = i;
        if (tt >= tL) {
            idx1 = i;
            break;
        }
    }
    
    if (idx0 < 0 || idx1 < 0) return;
    
    ros::Time t0 = track_buf_[idx0].header.stamp;
    ros::Time t1 = track_buf_[idx1].header.stamp;
    
    double dt_total = (t1 - t0).toSec();
    if (dt_total <= 1e-6) dt_total = 1e-6;
    
    double alpha = (tL - t0).toSec() / dt_total;
    alpha = std::max(0.0, std::min(1.0, alpha));
    
    custom_msgs::TrackingResultArray interp;
    interp.header.stamp = tL;
    interp.header.frame_id = track_buf_[idx0].header.frame_id;
    
    for (const auto& a : track_buf_[idx0].tracks) {
        for (const auto& b : track_buf_[idx1].tracks) {
            if (a.track_id == b.track_id) {
                custom_msgs::TrackingResult t;
                t.track_id = a.track_id;
                t.class_id = a.class_id;
                t.score = (1 - alpha) * a.score + alpha * b.score;
                
                t.x1 = (1 - alpha) * a.x1 + alpha * b.x1;
                t.y1 = (1 - alpha) * a.y1 + alpha * b.y1;
                t.x2 = (1 - alpha) * a.x2 + alpha * b.x2;
                t.y2 = (1 - alpha) * a.y2 + alpha * b.y2;
                
                interp.tracks.push_back(t);
                break;
            }
        }
    }
    
    
    //int best = -1;
    //double best_dt = 1e9;
    //for(int i=0; i<(int)track_buf_.size(); ++i){
    //  double dt = std::fabs((tL - track_buf_[i].header.stamp).toSec());
    //  if(dt < best_dt){
    //    best_dt = dt;
    //    best = i;
    //  }
    //}
    //if(best < 0) return;

    //if(tracking_max_age_sec_ >= 0.0 && best_dt > tracking_max_age_sec_){
    //  ROS_WARN_THROTTLE(1.0,
    //    "Tracking-LiDAR time gap=%.3fs > tracking_max_age=%.3f, skip",
    //    best_dt, tracking_max_age_sec_);
    //  return;
    //}
    //const custom_msgs::TrackingResultArray& tracks = track_buf_[best];
    const custom_msgs::TrackingResultArray& tracks = interp;

    // 2) LiDAR point cloud 변환
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*pc_msg, *cloud);

    // 3) 다운샘플
    auto cloud_ds = downsample(cloud);
    if(!cloud_ds || cloud_ds->empty()) return;

    // 4) 이상치 제거
    auto cloud_no_outliers = removeOutliers(cloud_ds);
    if(!cloud_no_outliers || cloud_no_outliers->empty()) return;

    // 5) 지면 제거
    auto cloud_proc = removeGroundRANSAC(cloud_no_outliers);
    if(!cloud_proc || cloud_proc->empty()) return;

    const int W = img_w_, H = img_h_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr kept_all(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored(new pcl::PointCloud<pcl::PointXYZRGB>());
    visualization_msgs::MarkerArray markers_text;
    visualization_msgs::MarkerArray markers_box;

    custom_msgs::Detection3DArray det3d_arr;
    det3d_arr.header = pc_msg->header;

    int next_text_id = 0;
    int next_box_id  = 0;

    // 6) 각 트랙에 대해 처리
    for(const auto& tr : tracks.tracks){
      if(!isPersonTrack(tr)) continue;

      // bbox를 center+size로 변환 (wrap-around 지원 위해)
      const float x1 = tr.x1;
      const float y1 = tr.y1;
      const float x2 = tr.x2;
      const float y2 = tr.y2;

      const float cx = (x1 + x2)*0.5f;
      const float cy = (y1 + y2)*0.5f;
      const float bw = (x2 - x1);
      const float bh = (y2 - y1);

      pcl::PointCloud<pcl::PointXYZ>::Ptr kept(new pcl::PointCloud<pcl::PointXYZ>());
      kept->reserve(cloud_proc->size());

      // 7) ERP 투영 후 bbox 안에 들어가는 포인트만 유지
      for(const auto& pt : cloud_proc->points){
        const Eigen::Vector3f pL(pt.x, pt.y, pt.z);
        const float dist_m = pL.norm();
        
        const Eigen::Vector3f pC = lidarToCam_modeD(pL, ex_);
        const UV uv = erpProject_lonA(pC, W, H);
        if(!uv.valid) continue;
        
        float adj_bw = bw + 2.0f * erp_m_;
        float adj_bh = bh + 2.0f * erp_m_;

        bool ok = inBboxWrapU(uv.u, uv.v, W, cx, cy, adj_bw, adj_bh, dist_m);
        if(ok) kept->push_back(pt);
      }

      if(kept->empty()) continue;

      // 8) DBSCAN clustering
      auto clusters = densityClusters(kept);
      
      if(clusters.empty()) continue;
        
      pcl::PointCloud<pcl::PointXYZ>::Ptr best = clusters[0];
      for (const auto& c : clusters) {
          if (c->size() > best->size())
              best = c;
      }

      // 가장 가까운 포인트
      Eigen::Vector3f closest_point(0.f, 0.f, 0.f);
      float min_dist2 = std::numeric_limits<float>::max();
      for(const auto& p : best->points){
          float d2 = p.x*p.x + p.y*p.y + p.z*p.z;
          if(d2 < min_dist2){
            min_dist2 = d2;
            closest_point = Eigen::Vector3f(p.x, p.y, p.z);
          }
      }
      if(!std::isfinite(min_dist2)) continue;

      float dist_m = std::sqrt(min_dist2);
      RGB col = bandColor(dist_m, (float)band_near_m_, (float)band_mid_m_);

      *cloud_colored += *colorizeCluster(best, col);
      *kept_all += *best;

      // 텍스트 마커
      auto tmk = makeTextMarker(next_text_id++, closest_point, dist_m, col,
                                pc_msg->header.frame_id, pc_msg->header.stamp,
                                text_scale_z_, text_lifetime_);
      markers_text.markers.push_back(tmk);

      // 3D Detection msg
      custom_msgs::Detection3D det3d;
      det3d.header = pc_msg->header;
      fill3DBbox(det3d, best, tr.track_id, tr.score);
      det3d_arr.detections.push_back(det3d);

      // 박스 마커
      auto bmk = makeBoxMarker(next_box_id++, det3d, col, "person_box");
      markers_box.markers.push_back(bmk);
      }

    // 9) publish
    if(!kept_all->empty()){
      sensor_msgs::PointCloud2 msg_raw;
      pcl::toROSMsg(*kept_all, msg_raw);
      msg_raw.header = pc_msg->header;
      pub_cloud_raw_.publish(msg_raw);
    }
    if(!cloud_colored->empty()){
      sensor_msgs::PointCloud2 msg_col;
      pcl::toROSMsg(*cloud_colored, msg_col);
      msg_col.header = pc_msg->header;
      pub_cloud_col_.publish(msg_col);
    }
    
    {
      visualization_msgs::Marker clear;
      clear.header = pc_msg->header;
      clear.ns = "person_dist";
      clear.id = 0;
      clear.action = visualization_msgs::Marker::DELETEALL;

      visualization_msgs::MarkerArray clear_arr;
      clear_arr.markers.push_back(clear);

      pub_markers_dist_.publish(clear_arr);
    }
    
    if(!markers_text.markers.empty()){
      pub_markers_dist_.publish(markers_text);
    }
    
    {
      visualization_msgs::Marker clear;
      clear.header = pc_msg->header;
      clear.ns = "person_box";
      clear.id = 0;
      clear.action = visualization_msgs::Marker::DELETEALL;
      
      visualization_msgs::MarkerArray clear_arr;
      clear_arr.markers.push_back(clear);
      
      pub_markers_box_.publish(clear_arr);
      
    }
    
    if(!markers_box.markers.empty()){
      pub_markers_box_.publish(markers_box);
    }
    if(!det3d_arr.detections.empty()){
      pub_det3d_.publish(det3d_arr);
    }

    //ROS_DEBUG_THROTTLE(1.0,
    //  "LiDAR-Tracking dt=%.3fs (buf=%lu)", 
    //  best_dt, (unsigned long)track_buf_.size());
  }
};

// ===================== main =====================
int main(int argc, char** argv){
  ros::init(argc, argv, "tracker_with_cloud_erp_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  try{
    TrackerWithCloudERP node(nh, pnh);
    ros::spin();
  }catch(const std::exception& e){
    ROS_ERROR("init failed: %s", e.what());
  }
  return 0;
}

