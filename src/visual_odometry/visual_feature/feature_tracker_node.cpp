#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0


// mtx lock for two threads
std::mutex mtx_lidar;

// 将5s内的点云放在一起, 并进行体素滤波采样
// global variable for saving the depthCloud shared between two threads
pcl::PointCloud<PointType>::Ptr depthCloud(new pcl::PointCloud<PointType>());

// global variables saving the lidar point cloud
deque<pcl::PointCloud<PointType>> cloudQueue; // 保留5s内的激光点云 (world系)
deque<double> timeQueue;

// global depth register for obtaining depth of a feature
DepthRegister *depthRegister;

// feature publisher for VINS estimator
ros::Publisher pub_feature;
ros::Publisher pub_match;
ros::Publisher pub_restart;

// feature tracker variables
FeatureTracker trackerData[NUM_OF_CAM]; // 一般为单目, NUM_OF_CAM为1
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;

ofstream foutC;

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    double cur_img_time = img_msg->header.stamp.toSec();

    if(first_image_flag) // 第一帧
    {
        first_image_flag = false;
        first_image_time = cur_img_time;
        last_image_time = cur_img_time;
        return;
    }
    // 1.detect unstable camera stream
    // 1.通过时间间隔判断相机数据流是否稳定，有问题则restart
    if (cur_img_time - last_image_time > 1.0 || cur_img_time < last_image_time)
    {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true; 
        last_image_time = 0;
        pub_count = 1;
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;
    }
    last_image_time = cur_img_time;
    // 2.frequency control
    // 2.发布频率控制, 20Hz
    if (round(1.0 * pub_count / (cur_img_time - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (cur_img_time - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = cur_img_time;
            pub_count = 0;
        }
    }
    else
    {
        PUB_THIS_FRAME = false;
    }

    // 3.图像格式转换(ROS to OpenCV)
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat show_img = ptr->image; // 双目也是一张图片 (两张图片合在一起, 上下组合)
    TicToc t_r;
    // 4.主要操作: 读取图像数据 (光流跟踪, 提取特征)
    for (int i = 0; i < NUM_OF_CAM; i++) // 分别对每个相机处理
    {
        ROS_DEBUG("processing camera %d", i);
        if (i != 1 || !STEREO_TRACK) // 读取第一张图片的数据 (单目, 双目)
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), cur_img_time); // 读取图像数据 (光流跟踪, 提取特征)
        else // 读取第二张图片的数据 (双目)
        {
            // 第二张图片不需要提取特征, 只需要进行均衡化处理
            if (EQUALIZE) // 光太亮或太暗，自适应直方图均衡化处理
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
                trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
        }

        #if SHOW_UNDISTORTION
            trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
        #endif
    }

    // 5.更新当前帧新提取的特征点id
    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK) 
                completed |= trackerData[j].updateID(i); // 更新特征的id
        if (!completed)
            break;
    }

   // 6.将当前帧特征发布出去, 供estimator_node使用
   if (PUB_THIS_FRAME)
   {
        // 1.将需要发布的数据都封装在一起 (归一化坐标, 特征点id, 像素坐标, 特征点速度)
        pub_count++;
        sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud); // 待发布的特征点 (归一化坐标xy1)
        // 下面的数据加入到feature_points的通道中, 封装在一起 (后面还有深度通道)
        sensor_msgs::ChannelFloat32 id_of_point;         // 特征点id
        sensor_msgs::ChannelFloat32 u_of_point;          // 特征点像素坐标 (uv)
        sensor_msgs::ChannelFloat32 v_of_point;
        sensor_msgs::ChannelFloat32 velocity_x_of_point; // 特征点速度 (归一化平面坐标xy1)
        sensor_msgs::ChannelFloat32 velocity_y_of_point;

        feature_points->header.stamp = img_msg->header.stamp;
        feature_points->header.frame_id = "vins_body";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++) // 单目
        {
            auto &un_pts = trackerData[i].cur_un_pts;         // 去畸变后的归一化坐标 xy1
            auto &cur_pts = trackerData[i].cur_pts;           // 像素坐标系 uv
            auto &ids = trackerData[i].ids;                   // 特征点id
            auto &pts_velocity = trackerData[i].pts_velocity; // 特征点速度
            // 遍历每一个特征点
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::Point32 p; // 归一化坐标 xy1
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    feature_points->points.push_back(p);
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i);     // 特征点id
                    u_of_point.values.push_back(cur_pts[j].x);               // 像素坐标
                    v_of_point.values.push_back(cur_pts[j].y);
                    velocity_x_of_point.values.push_back(pts_velocity[j].x); // 特征点速度
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);
                }
            }
        }

        // 将数据都封装在一起
        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);

        // 2.从激光点云中获取特征点的深度
        // 2.get feature depth from lidar point cloud
        pcl::PointCloud<PointType>::Ptr depth_cloud_temp(new pcl::PointCloud<PointType>());
        mtx_lidar.lock();
        *depth_cloud_temp = *depthCloud; // 5s内的激光点云
        mtx_lidar.unlock();

        sensor_msgs::ChannelFloat32 depth_of_points = depthRegister->get_depth(img_msg->header.stamp, show_img, depth_cloud_temp, trackerData[0].m_camera, feature_points->points);
        feature_points->channels.push_back(depth_of_points);


        // 3.发布当前帧的特征点云, 给其他node使用
        // 3.skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
            pub_feature.publish(feature_points);

        
        // 4.publish features in image
        // 4.在图像上标记提取到的特征点, 发布出去用于显示
        if (pub_match.getNumSubscribers() != 0)
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::RGB8);
            //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    if (SHOW_TRACK)
                    {
                        // track count
                        double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                        cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(255 * (1 - len), 255 * len, 0), 4);
                    } else {
                        // depth 
                        if(j < depth_of_points.values.size())
                        {
                            if (depth_of_points.values[j] > 0)
                                cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(0, 255, 0), 4);
                            else
                                cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(0, 0, 255), 4);
                        }
                    }
                }
            }

            pub_match.publish(ptr->toImageMsg());
        }

        // 5.Save camera timestamp
        if(saveCameraTimestamp){
            foutC << to_string(img_msg->header.stamp.toSec()) << endl;
        }

        // 6.Save feature points and depth
        // 保存特征点像素坐标和深度
        if (saveFeaturePointsAndDepth)
        {
            ofstream foutFeaturePoints;
            foutFeaturePoints.open(saveFeaturePointsAndDepthDirectory + to_string(img_msg->header.stamp.toSec()) + ".txt");
            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    if (j < depth_of_points.values.size() && depth_of_points.values[j] > 0){
                        foutFeaturePoints << trackerData[i].cur_pts[j].x << " " << trackerData[i].cur_pts[j].y << " "
                           << depth_of_points.values[j] << endl;
                    }
                }
            }
            foutFeaturePoints.close();
        }
    }
}


void lidar_callback(const sensor_msgs::PointCloud2ConstPtr& laser_msg)
{
    static int lidar_count = -1;
    if (++lidar_count % (LIDAR_SKIP+1) != 0)
        return; // 并不是所有激光帧都要

    // 0. listen to transform 获取camera在世界坐标系的位姿(camera to world)
    static tf::TransformListener listener;
    static tf::StampedTransform transform; // camera to world
    try{
        listener.waitForTransform("vins_world", "vins_body_ros", laser_msg->header.stamp, ros::Duration(0.01));
        listener.lookupTransform("vins_world", "vins_body_ros", laser_msg->header.stamp, transform);
    } 
    catch (tf::TransformException ex){
        // ROS_ERROR("lidar no tf");
        return;
    }

    double xCur, yCur, zCur, rollCur, pitchCur, yawCur;
    xCur = transform.getOrigin().x();
    yCur = transform.getOrigin().y();
    zCur = transform.getOrigin().z();
    tf::Matrix3x3 m(transform.getRotation());
    m.getRPY(rollCur, pitchCur, yawCur);
    Eigen::Affine3f transNow = pcl::getTransformation(xCur, yCur, zCur, rollCur, pitchCur, yawCur);

    // 1. convert laser cloud message to pcl 激光点云数据格式转换
    pcl::PointCloud<PointType>::Ptr laser_cloud_in(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*laser_msg, *laser_cloud_in);

    // 2. downsample new cloud (save memory) 激光点云采样
    pcl::PointCloud<PointType>::Ptr laser_cloud_in_ds(new pcl::PointCloud<PointType>());
    static pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.setInputCloud(laser_cloud_in);
    downSizeFilter.filter(*laser_cloud_in_ds);
    *laser_cloud_in = *laser_cloud_in_ds;

    // 3. 剔除不在相机视野范围内的激光点
    // 3. filter lidar points (only keep points in camera view)
    pcl::PointCloud<PointType>::Ptr laser_cloud_in_filter(new pcl::PointCloud<PointType>());
    for (int i = 0; i < (int)laser_cloud_in->size(); ++i)
    {
        PointType p = laser_cloud_in->points[i];
        if (p.x >= 0 && abs(p.y / p.x) <= 10 && abs(p.z / p.x) <= 10)
            laser_cloud_in_filter->push_back(p);
    }
    *laser_cloud_in = *laser_cloud_in_filter;

    // TODO: transform to IMU body frame
    // 4. offset T_lidar -> T_camera 点云坐标变换(lidar to camera)
    pcl::PointCloud<PointType>::Ptr laser_cloud_offset(new pcl::PointCloud<PointType>());
    Eigen::Affine3f transOffset = pcl::getTransformation(L_C_TX, L_C_TY, L_C_TZ, L_C_RX, L_C_RY, L_C_RZ);
    pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_offset, transOffset);
    *laser_cloud_in = *laser_cloud_offset;

    // 5. transform new cloud into global odom frame 点云坐标变换(camera to world)
    pcl::PointCloud<PointType>::Ptr laser_cloud_global(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_global, transNow);

    // 6. save new cloud
    double timeScanCur = laser_msg->header.stamp.toSec();
    cloudQueue.push_back(*laser_cloud_global);
    timeQueue.push_back(timeScanCur);

    // 7. pop old cloud 只保留5s内的激光点云
    while (!timeQueue.empty())
    {
        if (timeScanCur - timeQueue.front() > 5.0)
        {
            cloudQueue.pop_front();
            timeQueue.pop_front();
        } else {
            break;
        }
    }

    std::lock_guard<std::mutex> lock(mtx_lidar);
    // 8. fuse global cloud
    depthCloud->clear();
    for (int i = 0; i < (int)cloudQueue.size(); ++i)
        *depthCloud += cloudQueue[i];

    // 9. downsample global cloud
    pcl::PointCloud<PointType>::Ptr depthCloudDS(new pcl::PointCloud<PointType>());
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.setInputCloud(depthCloud);
    downSizeFilter.filter(*depthCloudDS);
    *depthCloud = *depthCloudDS;
}

int main(int argc, char **argv)
{
    // initialize ROS node
    ros::init(argc, argv, "vins");
    ros::NodeHandle n;
    ROS_INFO("\033[1;32m----> Visual Feature Tracker Started.\033[0m");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);
    readParameters(n); // 读取一些配置参数

    // read camera params 读取相机内参, NUM_OF_CAM一般为1, 单目
    for (int i = 0; i < NUM_OF_CAM; i++) // 可能是双目, 有两个相机
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);

    // load fisheye mask to remove features on the boundry
    if(FISHEYE) // 使用鱼眼mask来去除边缘噪声
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                ROS_ERROR("load fisheye mask fail");
                ROS_BREAK();
            }
            else
                ROS_INFO("load mask success");
        }
    }

    // Save camera timestamp
    if(saveCameraTimestamp){
        // 1.create directory and remove old files, 删除文件夹再重建!!!
        saveCameraTimestampDirectory = std::getenv("HOME") + saveCameraTimestampDirectory;
        // system((std::string("exec rm -r ") + saveTXTDirectory).c_str());
        system((std::string("mkdir ") + saveCameraTimestampDirectory).c_str());    
        foutC.open(saveCameraTimestampDirectory + "camera_timestamp.txt");
    }

    // Save feature points and depth
    if(saveFeaturePointsAndDepth){
        // 1.create directory and remove old files, 删除文件夹再重建!!!
        saveFeaturePointsAndDepthDirectory = std::getenv("HOME") + saveFeaturePointsAndDepthDirectory;
        system((std::string("exec rm -r ") + saveFeaturePointsAndDepthDirectory).c_str());
        system((std::string("mkdir ") + saveFeaturePointsAndDepthDirectory).c_str());    
    }

    // initialize depthRegister (after readParameters())
    depthRegister = new DepthRegister(n);
    
    // subscriber to image and lidar
    ros::Subscriber sub_img   = n.subscribe(IMAGE_TOPIC,       5,    img_callback);
    ros::Subscriber sub_lidar = n.subscribe(POINT_CLOUD_TOPIC, 5,    lidar_callback);
    if (!USE_LIDAR)
        sub_lidar.shutdown();

    // messages to vins estimator
    pub_feature = n.advertise<sensor_msgs::PointCloud>(PROJECT_NAME + "/vins/feature/feature",     5); // 当前帧的特征
    pub_match   = n.advertise<sensor_msgs::Image>     (PROJECT_NAME + "/vins/feature/feature_img", 5); // 当前帧的图像 (标记特征点)
    pub_restart = n.advertise<std_msgs::Bool>         (PROJECT_NAME + "/vins/feature/restart",     5); // 是否重启

    // two ROS spinners for parallel processing (image and lidar)
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}