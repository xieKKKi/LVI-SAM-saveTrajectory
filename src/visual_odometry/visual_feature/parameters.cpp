#include "parameters.h"

std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
std::string POINT_CLOUD_TOPIC;
std::string PROJECT_NAME;

std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK; // 鱼眼mask, 用于去除边缘噪声
int MAX_CNT;              // 最少特征点数 (待发布的frame至少得有150个特征点)
int MIN_DIST;             // 两个特征点至少相隔20个像素
int WINDOW_SIZE;
int FREQ;                 // 发布频率: 20Hz
double F_THRESHOLD;       // 通过基础矩阵F来判断外点的阈值: 1个像素
int SHOW_TRACK;
int STEREO_TRACK;         // false, 只是单目
int EQUALIZE;             // 光太亮或太暗，自适应直方图均衡化处理
int ROW;                  // image_height
int COL;                  // image_width
int FOCAL_LENGTH;
int FISHEYE;              // 是否使用鱼眼mask
bool PUB_THIS_FRAME;      // 是否发布当前帧的特征

// lidar to camera坐标变换
double L_C_TX;
double L_C_TY;
double L_C_TZ;
double L_C_RX;
double L_C_RY;
double L_C_RZ;

int USE_LIDAR;
int LIDAR_SKIP;

// Save camera timestamp
int saveCameraTimestamp;
string saveCameraTimestampDirectory;

// Save feature points and depth
int saveFeaturePointsAndDepth;
string saveFeaturePointsAndDepthDirectory;

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    n.getParam("vins_config_file", config_file);
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    // project name
    fsSettings["project_name"] >> PROJECT_NAME;
    std::string pkg_path = ros::package::getPath(PROJECT_NAME);

    // Save camera timestamp
    saveCameraTimestamp = fsSettings["saveCameraTimestamp"];
    fsSettings["saveCameraTimestampDirectory"] >> saveCameraTimestampDirectory;

    // Save feature points and depth
    saveFeaturePointsAndDepth = fsSettings["saveFeaturePointsAndDepth"];
    fsSettings["saveFeaturePointsAndDepthDirectory"] >> saveFeaturePointsAndDepthDirectory;

    // sensor topics
    fsSettings["image_topic"]       >> IMAGE_TOPIC;
    fsSettings["imu_topic"]         >> IMU_TOPIC;
    fsSettings["point_cloud_topic"] >> POINT_CLOUD_TOPIC;

    // lidar configurations
    fsSettings["use_lidar"] >> USE_LIDAR;
    fsSettings["lidar_skip"] >> LIDAR_SKIP;

    // feature and image settings
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];

    L_C_TX = fsSettings["lidar_to_cam_tx"];
    L_C_TY = fsSettings["lidar_to_cam_ty"];
    L_C_TZ = fsSettings["lidar_to_cam_tz"];
    L_C_RX = fsSettings["lidar_to_cam_rx"];
    L_C_RY = fsSettings["lidar_to_cam_ry"];
    L_C_RZ = fsSettings["lidar_to_cam_rz"];

    // fisheye mask
    FISHEYE = fsSettings["fisheye"];
    if (FISHEYE == 1)
    {
        std::string mask_name;
        fsSettings["fisheye_mask"] >> mask_name;
        FISHEYE_MASK = pkg_path + mask_name;
    }

    // camera config
    CAM_NAMES.push_back(config_file);

    WINDOW_SIZE = 20;
    STEREO_TRACK = false;
    FOCAL_LENGTH = 460;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;

    fsSettings.release();
    usleep(100);
}

float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

void publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    if (thisPub->getNumSubscribers() == 0)
        return;
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    thisPub->publish(tempCloud); 
}