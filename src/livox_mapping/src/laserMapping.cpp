#include <math.h>

#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include<pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include "std_msgs/Float32.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

typedef pcl::PointXYZI PointType;

int kfNum = 0;

float timeLaserCloudCornerLast = 0;
float timeLaserCloudSurfLast = 0;
float timeLaserCloudFullRes = 0;

bool newLaserCloudCornerLast = false;
bool newLaserCloudSurfLast = false;
bool newLaserCloudFullRes = false;

int laserCloudCenWidth = 10;
int laserCloudCenHeight = 5;
int laserCloudCenDepth = 10;
const int laserCloudWidth = 21;
const int laserCloudHeight = 11;
const int laserCloudDepth = 21;

const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;//4851

int laserCloudValidInd[125];

int laserCloudSurroundInd[125];

//corner feature
pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudCornerLast_down(new pcl::PointCloud<PointType>());
//surf feature
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast_down(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudOri(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr coeffSel(new pcl::PointCloud<PointType>());

// pcl::PointCloud<PointType>::Ptr laserCloudSurround(new pcl::PointCloud<PointType>());
// pcl::PointCloud<PointType>::Ptr laserCloudSurround_corner(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudSurround2(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurround2_corner(new pcl::PointCloud<PointType>());
//corner feature in map
pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap(new pcl::PointCloud<PointType>());
//surf feature in map
pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap(new pcl::PointCloud<PointType>());

std::vector< Eigen::Matrix<float,7,1> > keyframe_pose;
std::vector< Eigen::Matrix4f > pose_map;
//all points
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes2(new pcl::PointCloud<PointType>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudFullResColor(new pcl::PointCloud<pcl::PointXYZRGB>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudFullResColor_pcd(new pcl::PointCloud<pcl::PointXYZRGB>());


pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum];

pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[laserCloudNum];

pcl::PointCloud<PointType>::Ptr laserCloudCornerArray2[laserCloudNum];

pcl::PointCloud<PointType>::Ptr laserCloudSurfArray2[laserCloudNum];

pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());

//optimization states 
float transformTobeMapped[6] = {0};
//optimization states after mapping
float transformAftMapped[6] = {0};
//last optimization states
float transformLastMapped[6] = {0};
float transformPredict[6] = {0};

//from reference
float transformRef[6] = {0};
float transformRefCurr[6] = {0};
float transformRefLast[6] = {0};
float transformRefLidar[6] = {0};

Eigen::Quaternionf quatRef;
float posRef[3] = {0.0F};
Eigen::Quaternionf quatRefCurr;
float posRefCurr[3] = {0.0F};
Eigen::Quaternionf quatRefLast;
float posRefLast[3] = {0.0F};

ros::Publisher pubDebug0;
ros::Publisher pubDebug1;
ros::Publisher pubDebug2;
ros::Publisher pubDebug3;
ros::Publisher pubDebug4;
ros::Publisher pubDebug5;
ros::Publisher pubDebug6;
ros::Publisher pubDebug7;
ros::Publisher pubDebug8;
ros::Publisher pubDebug9;
ros::Publisher pubDebug10;
ros::Publisher pubDebug11;
ros::Publisher pubDebug12;
ros::Publisher pubDebug13;

float deltaTTotal = 0.0F;

int n0,n1,n2,n3,n4,n5,n6,n7;

double time_ref = -1.0;
double time_livox = -1.0;
double dt0 = 0.0;
int ndt = 0;

float debug_distance = 0;
bool isPoseValid = false;
bool isDegenerate = false;

std::string logPath = "/home/mpikh/temp1/23/debug.csv";
std::ofstream log_file(logPath);


cv::Mat thermalImage;
cv::Mat thermalImage1;


double roll_, pitch_, yaw_;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    thermalImage = cv_bridge::toCvCopy(msg, "bgr8")->image;

    // thermalImage = cv_bridge::toCvShare(msg, "bgr8")->image;
    thermalImage1 = thermalImage;
    // cv::imshow("seek", thermalImage);
    // cv::waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

double rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}
double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}

Eigen::Matrix4f trans_euler_to_matrix(const float *trans)
{
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f R;
    Eigen::AngleAxisf rollAngle(Eigen::AngleAxisf(trans[0],Eigen::Vector3f::UnitX()));
    Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisf(trans[1],Eigen::Vector3f::UnitY()));
    Eigen::AngleAxisf yawAngle(Eigen::AngleAxisf(trans[2],Eigen::Vector3f::UnitZ()));
    R = pitchAngle * rollAngle * yawAngle; //zxy    
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = Eigen::Vector3f(trans[3],trans[4],trans[5]);

    return T;
}



Eigen::Matrix3f rot_mat_from_transform(const float *trans)
{
    Eigen::Matrix3f R;
    Eigen::AngleAxisf rollAngle(Eigen::AngleAxisf(trans[0],Eigen::Vector3f::UnitX()));
    Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisf(trans[1],Eigen::Vector3f::UnitY()));
    Eigen::AngleAxisf yawAngle(Eigen::AngleAxisf(trans[2],Eigen::Vector3f::UnitZ()));
    R = pitchAngle * rollAngle * yawAngle; //zxy   

    return R; 
}

float angle_from_rot_mat(Eigen::Matrix3f R)
{
    Eigen::AngleAxisf aa;
    aa = R;

    return aa.angle(); 
}

float angle_between_tranforms(const float *trans1, const float *trans2)
{
    Eigen::Matrix3f R1, R2, R12;
    R1 = rot_mat_from_transform(trans1);
    R2 = rot_mat_from_transform(trans2);
    R12 = R1 * R2.transpose();
    float a;
    a = angle_from_rot_mat(R12);
    return a;
}

float distance_between_tranforms(const float *trans1, const float *trans2)
{
    float d = sqrt((trans1[3]-trans2[3])*(trans1[3]-trans2[3]) + 
    (trans1[4]-trans2[4])*(trans1[4]-trans2[4]) + 
    (trans1[5]-trans2[5])*(trans1[5]-trans2[5]));
    return d;
}

float angle_diff(float a1, float a2)
{
    float a12 = a1 - a2;
    return atan2(sin(a12),cos(a12));
}

void transformPredictFromRef()
{
    quatRefCurr = quatRef;

    for (int i = 0; i < 3; i++)
        posRefCurr[i] = posRef[i];

    Eigen::Matrix4f T_lastRef, T_currRef;
    Eigen::Matrix4f T_lidar2pose;
    Eigen::Matrix4f T_currRefLidar;
    Eigen::Matrix4f T_predict;
    T_lidar2pose << 1, 0, 0, 0.005,
                    0, 1, 0, 0,
                    0, 0, 1, -0.085,
                    0, 0, 0, 1;
    // T_lidar2pose << 1, 0, 0, 0,
    //                 0, 1, 0, 0,
    //                 0, 0, 1, 0,
    //                 0, 0, 0, 1;

    Eigen::Matrix3f R;
    R = quatRefCurr;
    T_currRef.block<3,3>(0,0) = R;
    // T_currRef.block<3,1>(0,3) = posRefCurr;
    T_currRef(0,3) = posRefCurr[0];
    T_currRef(1,3) = posRefCurr[1];
    T_currRef(2,3) = posRefCurr[2];

    // T_currRefLidar = T_currRef * T_lidar2pose.inverse();
    T_currRefLidar = T_currRef * T_lidar2pose;
    T_predict = T_currRefLidar;

    Eigen::Matrix3f R_predict;
    Eigen::Vector3f euler_predict,t_predict;
    R_predict = T_predict.block<3,3>(0,0);
    euler_predict = R_predict.eulerAngles(1,0,2);
    t_predict = T_predict.block<3,1>(0,3);

    transformPredict[0] = euler_predict[1];
    transformPredict[1] = euler_predict[0];
    transformPredict[2] = euler_predict[2];
    transformPredict[3] = t_predict[0];
    transformPredict[4] = t_predict[1];
    transformPredict[5] = t_predict[2];
}

void transformUpdate()
{
    std_msgs::Float32 msgD;

    float angleDiff;
    float posDiff;
    angleDiff = angle_between_tranforms(transformPredict,transformTobeMapped);
    posDiff = distance_between_tranforms(transformPredict,transformTobeMapped);

    for (int i = 0; i < 6; i++)
        log_file << transformPredict[i] << ",";
    for (int i = 0; i < 6; i++)
        log_file << transformTobeMapped[i] << ",";

    log_file << angleDiff << "," << posDiff << "," << std::endl;

    // msgD.data = transformPredict[0];
    // pubDebug0.publish(msgD);
    // msgD.data = transformPredict[1];
    // pubDebug1.publish(msgD);
    // msgD.data = transformPredict[2];
    // pubDebug2.publish(msgD);
    msgD.data = 180.0 / 3.14 * roll_;
    pubDebug0.publish(msgD);
    msgD.data = 180.0 / 3.14 * pitch_;
    pubDebug1.publish(msgD);
    msgD.data = 180.0 / 3.14 * yaw_;
    pubDebug2.publish(msgD);    
    msgD.data = transformTobeMapped[0];
    pubDebug3.publish(msgD);
    msgD.data = transformTobeMapped[1];
    pubDebug4.publish(msgD);
    msgD.data = transformTobeMapped[2];
    pubDebug5.publish(msgD);
    msgD.data = transformPredict[3];
    pubDebug6.publish(msgD);
    msgD.data = transformPredict[4];
    pubDebug7.publish(msgD);
    msgD.data = transformPredict[5];
    pubDebug8.publish(msgD);
    msgD.data = transformTobeMapped[3];
    pubDebug9.publish(msgD);
    msgD.data = transformTobeMapped[4];
    pubDebug10.publish(msgD);
    msgD.data = transformTobeMapped[5];
    pubDebug11.publish(msgD);
    msgD.data = angleDiff;
    pubDebug12.publish(msgD);
    msgD.data = posDiff;
    pubDebug13.publish(msgD);

    // if ((debug_distance < 20.0) && (deltaPos < 0.2) && (deltaAngle < 2.0*3.14/180.0))
    // if ((posDiff < 5.0) && (angleDiff < 20.0*3.14/180.0) && (!isDegenerate))
    // if ((deltaPos < 0.5) && (deltaAngle < 2.0*3.14/180.0))
    // if (true)
    // if (true)
    
    // if ((posDiff < 2.0) && (angleDiff < 20.0*3.14/180.0))
    if ((posDiff < 5.0) && (angleDiff < 10.0*3.14/180.0))
    {
        isPoseValid = true;
        for (int i = 0; i < 6; i++) {
            transformLastMapped[i] = transformAftMapped[i];
            transformAftMapped[i] = transformTobeMapped[i];
        }
    }
    else
    {
        isPoseValid = false;
        for (int i = 0; i < 6; i++) {
            transformLastMapped[i] = transformAftMapped[i];
            transformAftMapped[i] = transformPredict[i];
        }   
    }

    // angleDiff = angle_between_tranforms(transformPredict,transformAftMapped);
    // posDiff = distance_between_tranforms(transformPredict,transformAftMapped);

    // for (int i = 0; i < 6; i++)
    //     log_file << transformPredict[i] << ",";
    // for (int i = 0; i < 6; i++)
    //     log_file << transformAftMapped[i] << ",";

    // log_file << angleDiff << "," << posDiff << "," << std::endl;
}

//lidar coordinate sys to world coordinate sys
void pointAssociateToMap(PointType const * const pi, PointType * const po)
{
    //rot z（transformTobeMapped[2]）
    float x1 = cos(transformTobeMapped[2]) * pi->x
            - sin(transformTobeMapped[2]) * pi->y;
    float y1 = sin(transformTobeMapped[2]) * pi->x
            + cos(transformTobeMapped[2]) * pi->y;
    float z1 = pi->z;

    //rot x（transformTobeMapped[0]）
    float x2 = x1;
    float y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
    float z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

    //rot y（transformTobeMapped[1]）then add trans
    po->x = cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2
            + transformTobeMapped[3];
    po->y = y2 + transformTobeMapped[4];
    po->z = -sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2
            + transformTobeMapped[5];
    po->intensity = pi->intensity;
}
//lidar coordinate sys to world coordinate sys USE S
void pointAssociateToMap_all(PointType const * const pi, PointType * const po)
{
    // Eigen::Matrix4f T_aft,T_last,delta_T;

    // Eigen::Matrix3f R_aft,R_last;
    // Eigen::Quaternionf Q_aft,Q_last;
    // Eigen::Vector3f t_aft,t_last;

    // T_aft = trans_euler_to_matrix(transformAftMapped);
    // T_last = trans_euler_to_matrix(transformLastMapped);

    // R_aft = T_aft.block<3,3>(0,0);
    // R_last = T_last.block<3,3>(0,0);

    // Q_aft = R_aft;
    // Q_last = R_last;

    double s;
    s = pi->intensity - int(pi->intensity);

    //std::cout<<"DEBUG pointAssociateToMap_all s: "<<pi->intensity<<std::endl;

    // Eigen::Quaternionf Q_s = Q_last.slerp(s,Q_aft);
    // Eigen::Matrix3f R_s = Q_s.matrix();

    // Eigen::Vector3f euler_s = R_s.eulerAngles(2,0,1);

    float rx = (1-s)*transformLastMapped[0] + s * transformAftMapped[0];
    float ry = (1-s)*transformLastMapped[1] + s * transformAftMapped[1];
    float rz = (1-s)*transformLastMapped[2] + s * transformAftMapped[2];
    float tx = (1-s)*transformLastMapped[3] + s * transformAftMapped[3];
    float ty = (1-s)*transformLastMapped[4] + s * transformAftMapped[4];
    float tz = (1-s)*transformLastMapped[5] + s * transformAftMapped[5];

    //rot z（transformTobeMapped[2]）
    float x1 = cos(rz) * pi->x
            - sin(rz) * pi->y;
    float y1 = sin(rz) * pi->x
            + cos(rz) * pi->y;
    float z1 = pi->z;

    //rot x（transformTobeMapped[0]）
    float x2 = x1;
    float y2 = cos(rx) * y1 - sin(rx) * z1;
    float z2 = sin(rx) * y1 + cos(rx) * z1;

    //rot y（transformTobeMapped[1]）then add trans
    po->x = cos(ry) * x2 + sin(ry) * z2 + tx;
    po->y = y2 + ty;
    po->z = -sin(ry) * x2 + cos(ry) * z2 + tz;
    po->intensity = pi->intensity;
}

void RGBpointAssociateToMap(PointType const * const pi, pcl::PointXYZRGB * const po)
{
    double s;
    s = pi->intensity - int(pi->intensity);

    // float rx = (1-s)*transformLastMapped[0] + s * transformAftMapped[0];
    // float ry = (1-s)*transformLastMapped[1] + s * transformAftMapped[1];
    // float rz = (1-s)*transformLastMapped[2] + s * transformAftMapped[2];
    // float tx = (1-s)*transformLastMapped[3] + s * transformAftMapped[3];
    // float ty = (1-s)*transformLastMapped[4] + s * transformAftMapped[4];
    // float tz = (1-s)*transformLastMapped[5] + s * transformAftMapped[5];
    float rx = transformAftMapped[0];
    float ry = transformAftMapped[1];
    float rz = transformAftMapped[2];
    float tx = transformAftMapped[3];
    float ty = transformAftMapped[4];
    float tz = transformAftMapped[5];
    //rot z（transformTobeMapped[2]）
    float x1 = cos(rz) * pi->x
            - sin(rz) * pi->y;
    float y1 = sin(rz) * pi->x
            + cos(rz) * pi->y;
    float z1 = pi->z;

    //rot x（transformTobeMapped[0]）
    float x2 = x1;
    float y2 = cos(rx) * y1 - sin(rx) * z1;
    float z2 = sin(rx) * y1 + cos(rx) * z1;

    //rot y（transformTobeMapped[1]）then add trans
    po->x = cos(ry) * x2 + sin(ry) * z2 + tx;
    po->y = y2 + ty;
    po->z = -sin(ry) * x2 + cos(ry) * z2 + tz;
    //po->intensity = pi->intensity;

    float intensity = pi->intensity;
    intensity = intensity - std::floor(intensity);

    int reflection_map = intensity*10000;

    //std::cout<<"DEBUG reflection_map "<<reflection_map<<std::endl;

    // 550.0 0.0 120.0
    // 0.0 550.0 160.0
    // 0.0 0.0 1.0

    // 0.0 -1.0 0.0 0.05
    // 0.0 0.0 -1.0 0.1
    // 1.0 0.0 0.0 0.0
    double matrix1[3][3] = {{550,0,120}, {0,550,160}, {0,0,1}}; 
    double matrix2[3][4] = {{0,-1,0,0.07}, {0,0,-1,0.06}, {1,0,0,0}};

    cv::Mat matrix_in(3, 3, CV_64F, matrix1);
    cv::Mat matrix_out(3, 4, CV_64F, matrix2);

    int row = thermalImage.rows;
    int col = thermalImage.cols;

    double matrix3[4][1] = {pi->x, pi->y, pi->z, 1};
    cv::Mat coordinate(4, 1, CV_64F, matrix3);
    
    // calculate the result of u and v
    cv::Mat result = matrix_in*matrix_out*coordinate;
    float uf = result.at<double>(0, 0);
    float vf = result.at<double>(1, 0);
    float depth = result.at<double>(2, 0);
    
    float UV[2] = {0, 0};
    UV[0] = uf / depth;
    UV[1] = vf / depth;

    int u = int(UV[0]);
    int v = int(UV[1]);

    // std::cout<<"u " << u << " v " << v << std::endl;

    if ((u < col) && (u >= 0) && (v < row) && (v >= 0))
    {
        // std::cout<<"u " << u << " v " << v << std::endl;

        // cv::circle(thermalImage1, cv::Point (u, v), 1, cv::Scalar(0,0,255), -1);

        // cv::circle(thermalImage1, cv::Point (u, v), 1, cv::Scalar(b, g, r), -1);
        // cv::imshow("seek", thermalImage);
        // cv::waitKey(1);

        cv::Vec3b pixel = thermalImage.at<cv::Vec3b>(cv::Point (u, v));
        po->r = pixel[2];
        po->g = pixel[1];
        po->b = pixel[0];
        // std::cout<<"u " << u << " v " << v << 
        // " r " << po->r << " g " << po->g << " b " << po->b << 
        // std::endl;

    }
        // po->r = 255;
        // po->g = 0;
        // po->b = 0;

}

void RGBpointAssociateToMap1(PointType const * const pi, pcl::PointXYZRGB * const po)
{
    double s;
    s = pi->intensity - int(pi->intensity);

    // float rx = (1-s)*transformLastMapped[0] + s * transformAftMapped[0];
    // float ry = (1-s)*transformLastMapped[1] + s * transformAftMapped[1];
    // float rz = (1-s)*transformLastMapped[2] + s * transformAftMapped[2];
    // float tx = (1-s)*transformLastMapped[3] + s * transformAftMapped[3];
    // float ty = (1-s)*transformLastMapped[4] + s * transformAftMapped[4];
    // float tz = (1-s)*transformLastMapped[5] + s * transformAftMapped[5];
    float rx = transformAftMapped[0];
    float ry = transformAftMapped[1];
    float rz = transformAftMapped[2];
    float tx = transformAftMapped[3];
    float ty = transformAftMapped[4];
    float tz = transformAftMapped[5];
    //rot z（transformTobeMapped[2]）
    float x1 = cos(rz) * pi->x
            - sin(rz) * pi->y;
    float y1 = sin(rz) * pi->x
            + cos(rz) * pi->y;
    float z1 = pi->z;

    //rot x（transformTobeMapped[0]）
    float x2 = x1;
    float y2 = cos(rx) * y1 - sin(rx) * z1;
    float z2 = sin(rx) * y1 + cos(rx) * z1;

    //rot y（transformTobeMapped[1]）then add trans
    po->x = cos(ry) * x2 + sin(ry) * z2 + tx;
    po->y = y2 + ty;
    po->z = -sin(ry) * x2 + cos(ry) * z2 + tz;
    //po->intensity = pi->intensity;

    float intensity = pi->intensity;
    intensity = intensity - std::floor(intensity);

    int reflection_map = intensity*10000;

    //std::cout<<"DEBUG reflection_map "<<reflection_map<<std::endl;

    if (reflection_map < 30)
    {
        int green = (reflection_map * 255 / 30);
        po->r = 0;
        po->g = green & 0xff;
        po->b = 0xff;
    }
    else if (reflection_map < 90)
    {
        int blue = (((90 - reflection_map) * 255) / 60);
        po->r = 0x0;
        po->g = 0xff;
        po->b = blue & 0xff;
    }
    else if (reflection_map < 150)
    {
        int red = ((reflection_map-90) * 255 / 60);
        po->r = red & 0xff;
        po->g = 0xff;
        po->b = 0x0;
    }
    else
    {
        int green = (((255-reflection_map) * 255) / (255-150));
        po->r = 0xff;
        po->g = green & 0xff;
        po->b = 0;
    }
}

void pointAssociateTobeMapped(PointType const * const pi, PointType * const po)
{
    //add trans then rot y
    float x1 = cos(transformTobeMapped[1]) * (pi->x - transformTobeMapped[3])
            - sin(transformTobeMapped[1]) * (pi->z - transformTobeMapped[5]);
    float y1 = pi->y - transformTobeMapped[4];
    float z1 = sin(transformTobeMapped[1]) * (pi->x - transformTobeMapped[3])
            + cos(transformTobeMapped[1]) * (pi->z - transformTobeMapped[5]);
    //rot x
    float x2 = x1;
    float y2 = cos(transformTobeMapped[0]) * y1 + sin(transformTobeMapped[0]) * z1;
    float z2 = -sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;
    //rot z
    po->x = cos(transformTobeMapped[2]) * x2
            + sin(transformTobeMapped[2]) * y2;
    po->y = -sin(transformTobeMapped[2]) * x2
            + cos(transformTobeMapped[2]) * y2;
    po->z = z2;
    po->intensity = pi->intensity;
}

void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudCornerLast2)
{
    timeLaserCloudCornerLast = laserCloudCornerLast2->header.stamp.toSec();

    laserCloudCornerLast->clear();
    pcl::fromROSMsg(*laserCloudCornerLast2, *laserCloudCornerLast);

    newLaserCloudCornerLast = true;

    for (int i = 0; i<6; i++)
        transformRefLidar[i] = transformRef[i];
    
    time_livox = timeLaserCloudCornerLast;

}

void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudSurfLast2)
{
    timeLaserCloudSurfLast = laserCloudSurfLast2->header.stamp.toSec();

    laserCloudSurfLast->clear();
    pcl::fromROSMsg(*laserCloudSurfLast2, *laserCloudSurfLast);

    newLaserCloudSurfLast = true;
}

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullRes2)
{
    timeLaserCloudFullRes = laserCloudFullRes2->header.stamp.toSec();

    laserCloudFullRes->clear();
    laserCloudFullResColor->clear();
    pcl::fromROSMsg(*laserCloudFullRes2, *laserCloudFullRes);

    newLaserCloudFullRes = true;
}

void OdometryRefHandler(const nav_msgs::Odometry::ConstPtr& msg)
{
    time_ref = msg->header.stamp.toSec();

    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    tf::Matrix3x3 m(q);
    m.getRPY(roll_, pitch_, yaw_);
    transformRef[0] = roll_;
    transformRef[1] = pitch_;
    transformRef[2] = yaw_;

    transformRef[3] = msg->pose.pose.position.x;
    transformRef[4] = msg->pose.pose.position.y;
    transformRef[5] = msg->pose.pose.position.z;

    // quatRef = q;
    quatRef.w() = msg->pose.pose.orientation.w;
    quatRef.x() = msg->pose.pose.orientation.x;
    quatRef.y() = msg->pose.pose.orientation.y;
    quatRef.z() = msg->pose.pose.orientation.z;
    posRef[0] = msg->pose.pose.position.x;
    posRef[1] = msg->pose.pose.position.y;
    posRef[2] = msg->pose.pose.position.z;

    std_msgs::Float32 msgD;
    // msgD.data = roll;
    // pubDebug3.publish(msgD);
    // msgD.data = pitch;
    // pubDebug4.publish(msgD);
    // msgD.data = yaw;
    // pubDebug5.publish(msgD);
    // msgD.data = msg->pose.pose.position.x;
    // pubDebug3.publish(msgD);
    // msgD.data = msg->pose.pose.position.y;
    // pubDebug4.publish(msgD);
    // msgD.data = msg->pose.pose.position.z;
    // pubDebug5.publish(msgD);

    // std_msgs::Float32 msgD;
    // msgD.data = transformRef[3];
    // pubDebug0.publish(msgD);

    // ROS_INFO("yaw: %f",yaw);
//   ROS_INFO("Seq: [%d]", msg->header.seq);
//   ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
//   ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
//   ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    cv::namedWindow("seek");

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("seek/image1", 1, imageCallback);

    ros::Subscriber subOdometryRef = nh.subscribe<nav_msgs::Odometry>
            ("/camera/odom/sample", 100, OdometryRefHandler);

    ros::Subscriber subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>
            ("/laser_cloud_sharp", 100, laserCloudCornerLastHandler);

    ros::Subscriber subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>
            ("/laser_cloud_flat", 100, laserCloudSurfLastHandler);

    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>
            ("/livox_cloud", 100, laserCloudFullResHandler);

    ros::Publisher pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>
            ("/laser_cloud_surround", 100);
    ros::Publisher pubLaserCloudSurround_corner = nh.advertise<sensor_msgs::PointCloud2>
            ("/laser_cloud_surround_corner", 100);

    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>
            ("/velodyne_cloud_registered", 100);

    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/aft_mapped_to_init", 1);

    // pubDebug0 = nh.advertise<std_msgs::Float32>("loam/debug0",100);
    // pubDebug1 = nh.advertise<std_msgs::Float32>("loam/debug1",100);
    // pubDebug2 = nh.advertise<std_msgs::Float32>("loam/debug2",100);
    // pubDebug3 = nh.advertise<std_msgs::Float32>("loam/debug3",100);
    // pubDebug4 = nh.advertise<std_msgs::Float32>("loam/debug4",100);
    // pubDebug5 = nh.advertise<std_msgs::Float32>("loam/debug5",100);
    // pubDebug6 = nh.advertise<std_msgs::Float32>("loam/debug6",100);
    // pubDebug7 = nh.advertise<std_msgs::Float32>("loam/debug7",100);
    // pubDebug8 = nh.advertise<std_msgs::Float32>("loam/debug8",100);
    // pubDebug9 = nh.advertise<std_msgs::Float32>("loam/debug9",100);
    // pubDebug10 = nh.advertise<std_msgs::Float32>("loam/debug10",100);
    // pubDebug11 = nh.advertise<std_msgs::Float32>("loam/debug11",100);
    // pubDebug12 = nh.advertise<std_msgs::Float32>("loam/debug12",100);
    // pubDebug13 = nh.advertise<std_msgs::Float32>("loam/debug13",100);
    pubDebug0 = nh.advertise<std_msgs::Float32>("roll",100);
    pubDebug1 = nh.advertise<std_msgs::Float32>("pitch",100);
    pubDebug2 = nh.advertise<std_msgs::Float32>("yaw",100);
    pubDebug3 = nh.advertise<std_msgs::Float32>("loam/debug3",100);
    pubDebug4 = nh.advertise<std_msgs::Float32>("loam/debug4",100);
    pubDebug5 = nh.advertise<std_msgs::Float32>("loam/debug5",100);
    pubDebug6 = nh.advertise<std_msgs::Float32>("x",100);
    pubDebug7 = nh.advertise<std_msgs::Float32>("y",100);
    pubDebug8 = nh.advertise<std_msgs::Float32>("z",100);
    pubDebug9 = nh.advertise<std_msgs::Float32>("loam/debug9",100);
    pubDebug10 = nh.advertise<std_msgs::Float32>("loam/debug10",100);
    pubDebug11 = nh.advertise<std_msgs::Float32>("loam/debug11",100);
    pubDebug12 = nh.advertise<std_msgs::Float32>("loam/debug12",100);
    pubDebug13 = nh.advertise<std_msgs::Float32>("loam/debug13",100);

    nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = "/camera_init";
    odomAftMapped.child_frame_id = "/aft_mapped";

    std::string map_file_path;
    ros::param::get("~map_file_path",map_file_path);
    double filter_parameter_corner;
    ros::param::get("~filter_parameter_corner",filter_parameter_corner);
    double filter_parameter_surf;
    ros::param::get("~filter_parameter_surf",filter_parameter_surf);

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    PointType pointOri, pointSel, coeff;

    cv::Mat matA0(10, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matB0(10, 1, CV_32F, cv::Scalar::all(-1));
    cv::Mat matX0(10, 1, CV_32F, cv::Scalar::all(0));

    cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

    // bool isDegenerate = false;
    isDegenerate = false;
    cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));
    //VoxelGrid
    pcl::VoxelGrid<PointType> downSizeFilterCorner;

    downSizeFilterCorner.setLeafSize(filter_parameter_corner, filter_parameter_corner, filter_parameter_corner);

    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    downSizeFilterSurf.setLeafSize(filter_parameter_surf, filter_parameter_surf, filter_parameter_surf);

    // pcl::VoxelGrid<PointType> downSizeFilterFull;
    // downSizeFilterFull.setLeafSize(0.15, 0.15, 0.15);

    for (int i = 0; i < laserCloudNum; i++) {
        laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
        laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
        laserCloudCornerArray2[i].reset(new pcl::PointCloud<PointType>());
        laserCloudSurfArray2[i].reset(new pcl::PointCloud<PointType>());
    }

//------------------------------------------------------------------------------------------------------
    ros::Rate rate(100);
    bool status = ros::ok();
    int nm = 0;
    while (status) {
        ros::spinOnce();

        if (newLaserCloudCornerLast && newLaserCloudSurfLast && newLaserCloudFullRes &&
                fabs(timeLaserCloudSurfLast - timeLaserCloudCornerLast) < 0.005 &&
                fabs(timeLaserCloudFullRes - timeLaserCloudCornerLast) < 0.005) {

            clock_t t1,t2,t3,t4;

            t1 = clock();

            newLaserCloudCornerLast = false;
            newLaserCloudSurfLast = false;
            newLaserCloudFullRes = false;

            // // // transformAssociateToMap();
            transformPredictFromRef();
            // if (nm == 0)
            // {
            for (int i = 0; i < 6; i++)
                transformTobeMapped[i] = transformPredict[i];
            // for (int i = 0; i < 3; i++)
            //     transformTobeMapped[i] = transformPredict[i];
            // }
            std::cout<<"DEBUG mapping start "<<std::endl;
            nm++;
 

            PointType pointOnYAxis;
            pointOnYAxis.x = 0.0;
            pointOnYAxis.y = 10.0;
            pointOnYAxis.z = 0.0;

            pointAssociateToMap(&pointOnYAxis, &pointOnYAxis);

            int centerCubeI = int((transformTobeMapped[3] + 25.0) / 50.0) + laserCloudCenWidth;
            int centerCubeJ = int((transformTobeMapped[4] + 25.0) / 50.0) + laserCloudCenHeight;
            int centerCubeK = int((transformTobeMapped[5] + 25.0) / 50.0) + laserCloudCenDepth;

            if (transformTobeMapped[3] + 25.0 < 0) centerCubeI--;
            if (transformTobeMapped[4] + 25.0 < 0) centerCubeJ--;
            if (transformTobeMapped[5] + 25.0 < 0) centerCubeK--;

            while (centerCubeI < 3) {
                for (int j = 0; j < laserCloudHeight; j++) {
                    for (int k = 0; k < laserCloudDepth; k++) {
                        int i = laserCloudWidth - 1;

                        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

                        for (; i >= 1; i--) {
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCornerArray[i - 1 + laserCloudWidth*j + laserCloudWidth * laserCloudHeight * k];
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                        }

                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeCornerPointer;
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeSurfPointer;
                        laserCloudCubeCornerPointer->clear();
                        laserCloudCubeSurfPointer->clear();
                    }
                }

                centerCubeI++;
                laserCloudCenWidth++;
            }

            while (centerCubeI >= laserCloudWidth - 3) {
                for (int j = 0; j < laserCloudHeight; j++) {
                    for (int k = 0; k < laserCloudDepth; k++) {
                        int i = 0;
                        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

                        for (; i < laserCloudWidth - 1; i++) {
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCornerArray[i + 1 + laserCloudWidth*j + laserCloudWidth * laserCloudHeight * k];
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                        }
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeCornerPointer;
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeSurfPointer;
                        laserCloudCubeCornerPointer->clear();
                        laserCloudCubeSurfPointer->clear();
                    }
                }

                centerCubeI--;
                laserCloudCenWidth--;
            }

            while (centerCubeJ < 3) {
                for (int i = 0; i < laserCloudWidth; i++) {
                    for (int k = 0; k < laserCloudDepth; k++) {
                        int j = laserCloudHeight - 1;
                        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

                        for (; j >= 1; j--) {
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCornerArray[i + laserCloudWidth*(j - 1) + laserCloudWidth * laserCloudHeight*k];
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight*k];
                        }
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeCornerPointer;
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeSurfPointer;
                        laserCloudCubeCornerPointer->clear();
                        laserCloudCubeSurfPointer->clear();
                    }
                }

                centerCubeJ++;
                laserCloudCenHeight++;
            }

            while (centerCubeJ >= laserCloudHeight - 3) {
                for (int i = 0; i < laserCloudWidth; i++) {
                    for (int k = 0; k < laserCloudDepth; k++) {
                        int j = 0;
                        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

                        for (; j < laserCloudHeight - 1; j++) {
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCornerArray[i + laserCloudWidth*(j + 1) + laserCloudWidth * laserCloudHeight*k];
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight*k];
                        }
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeCornerPointer;
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeSurfPointer;
                        laserCloudCubeCornerPointer->clear();
                        laserCloudCubeSurfPointer->clear();
                    }
                }

                centerCubeJ--;
                laserCloudCenHeight--;
            }

            while (centerCubeK < 3) {
                for (int i = 0; i < laserCloudWidth; i++) {
                    for (int j = 0; j < laserCloudHeight; j++) {
                        int k = laserCloudDepth - 1;
                        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

                        for (; k >= 1; k--) {
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth * laserCloudHeight*(k - 1)];
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k - 1)];
                        }
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeCornerPointer;
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeSurfPointer;
                        laserCloudCubeCornerPointer->clear();
                        laserCloudCubeSurfPointer->clear();
                    }
                }

                centerCubeK++;
                laserCloudCenDepth++;
            }

            while (centerCubeK >= laserCloudDepth - 3) {
                for (int i = 0; i < laserCloudWidth; i++) {
                    for (int j = 0; j < laserCloudHeight; j++) {
                        int k = 0;
                        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

                        for (; k < laserCloudDepth - 1; k++) {
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth * laserCloudHeight*(k + 1)];
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k + 1)];
                        }
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeCornerPointer;
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeSurfPointer;
                        laserCloudCubeCornerPointer->clear();
                        laserCloudCubeSurfPointer->clear();
                    }
                }

                centerCubeK--;
                laserCloudCenDepth--;
            }

            int laserCloudValidNum = 0;
            int laserCloudSurroundNum = 0;

            for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++) {
                for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++) {
                    for (int k = centerCubeK - 2; k <= centerCubeK + 2; k++) {
                        if (i >= 0 && i < laserCloudWidth &&
                                j >= 0 && j < laserCloudHeight &&
                                k >= 0 && k < laserCloudDepth) {

                            float centerX = 50.0 * (i - laserCloudCenWidth);
                            float centerY = 50.0 * (j - laserCloudCenHeight);
                            float centerZ = 50.0 * (k - laserCloudCenDepth);

                            bool isInLaserFOV = false;
                            for (int ii = -1; ii <= 1; ii += 2) {
                                for (int jj = -1; jj <= 1; jj += 2) {
                                    for (int kk = -1; kk <= 1; kk += 2) {

                                        float cornerX = centerX + 25.0 * ii;
                                        float cornerY = centerY + 25.0 * jj;
                                        float cornerZ = centerZ + 25.0 * kk;

                                        float squaredSide1 = (transformTobeMapped[3] - cornerX)
                                                * (transformTobeMapped[3] - cornerX)
                                                + (transformTobeMapped[4] - cornerY)
                                                * (transformTobeMapped[4] - cornerY)
                                                + (transformTobeMapped[5] - cornerZ)
                                                * (transformTobeMapped[5] - cornerZ);

                                        float squaredSide2 = (pointOnYAxis.x - cornerX) * (pointOnYAxis.x - cornerX)
                                                + (pointOnYAxis.y - cornerY) * (pointOnYAxis.y - cornerY)
                                                + (pointOnYAxis.z - cornerZ) * (pointOnYAxis.z - cornerZ);

                                        float check1 = 100.0 + squaredSide1 - squaredSide2
                                                - 10.0 * sqrt(3.0) * sqrt(squaredSide1);

                                        float check2 = 100.0 + squaredSide1 - squaredSide2
                                                + 10.0 * sqrt(3.0) * sqrt(squaredSide1);

                                        if (check1 < 0 && check2 > 0) {
                                            isInLaserFOV = true;
                                        }
                                    }
                                }
                            }

                            if (isInLaserFOV) {
                                laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j
                                        + laserCloudWidth * laserCloudHeight * k;
                                laserCloudValidNum++;
                            }
                            laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j
                                    + laserCloudWidth * laserCloudHeight * k;
                            laserCloudSurroundNum++;
                        }
                    }
                }
            }

            laserCloudCornerFromMap->clear();
            laserCloudSurfFromMap->clear();
            
            for (int i = 0; i < laserCloudValidNum; i++) {
                *laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
                *laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
            }
            int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
            int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();

            laserCloudCornerLast_down->clear();
            downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
            downSizeFilterCorner.filter(*laserCloudCornerLast_down);
            int laserCloudCornerLast_downNum = laserCloudCornerLast_down->points.size();

            laserCloudSurfLast_down->clear();
            downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
            downSizeFilterSurf.filter(*laserCloudSurfLast_down);
            int laserCloudSurfLast_downNum = laserCloudSurfLast_down->points.size();

            std::cout<<"DEBUG MAPPING laserCloudCornerLast_down : "<<laserCloudCornerLast_down->points.size()<<" laserCloudSurfLast_down : "
            <<laserCloudSurfLast_down->points.size()<<std::endl;
            std::cout<<"DEBUG MAPPING laserCloudCornerLast : "<<laserCloudCornerLast->points.size()<<" laserCloudSurfLast : "
            <<laserCloudSurfLast->points.size()<<std::endl;
            std::cout<<"DEBUG MAPPING laserCloudCornerFromMapNum : "<<laserCloudCornerFromMapNum<<" laserCloudSurfFromMapNum : "
            <<laserCloudSurfFromMapNum<<std::endl;

            int num_temp = 0;
            // float debug_distance = 0;

            t2 = clock();
            if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 100) {
            //if (laserCloudSurfFromMapNum > 100) {
            // if (false) {
                kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
                kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);

                // int num_temp = 0;
                num_temp = 0;

                for (int iterCount = 0; iterCount < 20; iterCount++) {
                // for (int iterCount = 0; iterCount < 50; iterCount++) {
                    num_temp++;
                    laserCloudOri->clear();
                    coeffSel->clear();
                    for (int i = 0; i < laserCloudCornerLast->points.size(); i++) {
                        pointOri = laserCloudCornerLast->points[i];

                        pointAssociateToMap(&pointOri, &pointSel);
                        //find the closest 5 points
                        if (iterCount == 0)
                            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                        if (pointSearchSqDis[4] < 1.5) {
                            float cx = 0;
                            float cy = 0;
                            float cz = 0;
                            for (int j = 0; j < 5; j++) {
                                cx += laserCloudCornerFromMap->points[pointSearchInd[j]].x;
                                cy += laserCloudCornerFromMap->points[pointSearchInd[j]].y;
                                cz += laserCloudCornerFromMap->points[pointSearchInd[j]].z;
                            }
                            cx /= 5;
                            cy /= 5;
                            cz /= 5;
                            //mean square error
                            float a11 = 0;
                            float a12 = 0;
                            float a13 = 0;
                            float a22 = 0;
                            float a23 = 0;
                            float a33 = 0;
                            for (int j = 0; j < 5; j++) {
                                float ax = laserCloudCornerFromMap->points[pointSearchInd[j]].x - cx;
                                float ay = laserCloudCornerFromMap->points[pointSearchInd[j]].y - cy;
                                float az = laserCloudCornerFromMap->points[pointSearchInd[j]].z - cz;

                                a11 += ax * ax;
                                a12 += ax * ay;
                                a13 += ax * az;
                                a22 += ay * ay;
                                a23 += ay * az;
                                a33 += az * az;
                            }
                            a11 /= 5;
                            a12 /= 5;
                            a13 /= 5;
                            a22 /= 5;
                            a23 /= 5;
                            a33 /= 5;

                            matA1.at<float>(0, 0) = a11;
                            matA1.at<float>(0, 1) = a12;
                            matA1.at<float>(0, 2) = a13;
                            matA1.at<float>(1, 0) = a12;
                            matA1.at<float>(1, 1) = a22;
                            matA1.at<float>(1, 2) = a23;
                            matA1.at<float>(2, 0) = a13;
                            matA1.at<float>(2, 1) = a23;
                            matA1.at<float>(2, 2) = a33;

                            cv::eigen(matA1, matD1, matV1);

                            if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {

                                float x0 = pointSel.x;
                                float y0 = pointSel.y;
                                float z0 = pointSel.z;
                                float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                                float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                                float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                                float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                                float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                                float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                                //OA = (x0 - x1, y0 - y1, z0 - z1),OB = (x0 - x2, y0 - y2, z0 - z2)，AB = （x1 - x2, y1 - y2, z1 - z2）
                                //cross:
                                //|  i      j      k  |
                                //|x0-x1  y0-y1  z0-z1|
                                //|x0-x2  y0-y2  z0-z2|
                                float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                                    * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                                    + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                                    * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                                    + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
                                                    * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

                                float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                                float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                            + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                                float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                                - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                                float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                                + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                                float ld2 = a012 / l12;
                                //if(fabs(ld2) > 1) continue;

                                float s = 1 - 0.9 * fabs(ld2);

                                coeff.x = s * la;
                                coeff.y = s * lb;
                                coeff.z = s * lc;
                                coeff.intensity = s * ld2;

                                if (s > 0.1) {
                                    laserCloudOri->push_back(pointOri);
                                    coeffSel->push_back(coeff);
                                }
                            }
                        }
                    }
                    //std::cout <<"DEBUG mapping select corner points : " << coeffSel->size() << std::endl;

                    for (int i = 0; i < laserCloudSurfLast_down->points.size(); i++) {
                        pointOri = laserCloudSurfLast_down->points[i];

                        pointAssociateToMap(&pointOri, &pointSel);
                        kdtreeSurfFromMap->nearestKSearch(pointSel, 8, pointSearchInd, pointSearchSqDis);

                        if (pointSearchSqDis[7] < 5.0) {

                            for (int j = 0; j < 8; j++) {
                                matA0.at<float>(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
                                matA0.at<float>(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
                                matA0.at<float>(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
                            }
                            //matA0*matX0=matB0
                            //AX+BY+CZ+D = 0 <=> AX+BY+CZ=-D <=> (A/D)X+(B/D)Y+(C/D)Z = -1
                            //(X,Y,Z)<=>mat_a0
                            //A/D, B/D, C/D <=> mat_x0
                
                            cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);  //TODO

                            float pa = matX0.at<float>(0, 0);
                            float pb = matX0.at<float>(1, 0);
                            float pc = matX0.at<float>(2, 0);
                            float pd = 1;

                            //ps is the norm of the plane normal vector
                            //pd is the distance from point to plane
                            float ps = sqrt(pa * pa + pb * pb + pc * pc);
                            pa /= ps;
                            pb /= ps;
                            pc /= ps;
                            pd /= ps;

                            bool planeValid = true;
                            for (int j = 0; j < 8; j++) {
                                if (fabs(pa * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
                                            pb * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
                                            pc * laserCloudSurfFromMap->points[pointSearchInd[j]].z + pd) > 0.2) {
                                    planeValid = false;
                                    break;
                                }
                            }

                            if (planeValid) {
                                //loss fuction
                                float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                                //if(fabs(pd2) > 0.1) continue;

                                float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

                                coeff.x = s * pa;
                                coeff.y = s * pb;
                                coeff.z = s * pc;
                                coeff.intensity = s * pd2;

                                if (s > 0.1) {
                                    laserCloudOri->push_back(pointOri);
                                    coeffSel->push_back(coeff);
                                }
                            }
                        }
                    }
                    //std::cout <<"DEBUG mapping select all points : " << coeffSel->size() << std::endl;

                    float srx = sin(transformTobeMapped[0]);
                    float crx = cos(transformTobeMapped[0]);
                    float sry = sin(transformTobeMapped[1]);
                    float cry = cos(transformTobeMapped[1]);
                    float srz = sin(transformTobeMapped[2]);
                    float crz = cos(transformTobeMapped[2]);

                    int laserCloudSelNum = laserCloudOri->points.size();
                    if (laserCloudSelNum < 50) {
                        continue;
                    }


                    //|c1c3+s1s2s3 c3s1s2-c1s3 c2s1|
                    //|   c2s3        c2c3      -s2|
                    //|c1s2s3-c3s1 c1c3s2+s1s3 c1c2|
                    //AT*A*x = AT*b
                    cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
                    cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
                    cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
                    cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
                    cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
                    cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
                    // float debug_distance = 0;
                    debug_distance = 0.0;
                    for (int i = 0; i < laserCloudSelNum; i++) {
                        pointOri = laserCloudOri->points[i];
                        coeff = coeffSel->points[i];

                        float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                                + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                                + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

                        float ary = ((cry*srx*srz - crz*sry)*pointOri.x
                                        + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                                + ((-cry*crz - srx*sry*srz)*pointOri.x
                                    + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

                        float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                                + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                                + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;

                        matA.at<float>(i, 0) = arx;
                        matA.at<float>(i, 1) = ary;
                        matA.at<float>(i, 2) = arz;
                        //TODO: the partial derivative
                        matA.at<float>(i, 3) = coeff.x;
                        matA.at<float>(i, 4) = coeff.y;
                        matA.at<float>(i, 5) = coeff.z;
                        matB.at<float>(i, 0) = -coeff.intensity;

                        debug_distance += fabs(coeff.intensity);
                    }
                    cv::transpose(matA, matAt);
                    matAtA = matAt * matA;
                    matAtB = matAt * matB;
                    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

                    //Deterioration judgment
                    if (iterCount == 0) {
                        cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
                        cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
                        cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

                        cv::eigen(matAtA, matE, matV);
                        matV.copyTo(matV2);

                        isDegenerate = false;
                        float eignThre[6] = {10, 10, 10, 10, 10, 10};
                        // float eignThre[6] = {1, 1, 1, 1, 1, 1};
                        for (int i = 5; i >= 0; i--) {
                            if (matE.at<float>(0, i) < eignThre[i]) {
                                for (int j = 0; j < 6; j++) {
                                    matV2.at<float>(i, j) = 0;
                                }
                                isDegenerate = true;
                            } else {
                                break;
                            }
                        }
                        matP = matV.inv() * matV2;
                    }

                    if (isDegenerate) {
                        cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
                        matX.copyTo(matX2);
                        matX = matP * matX2;
                        std::cout<<"isDegenerate" << std::endl;
                    }

          if (isnan(matX.at<float>(0, 0)) || isnan(matX.at<float>(1, 0)) || isnan(matX.at<float>(2, 0)) || isnan(matX.at<float>(3, 0)) || isnan(matX.at<float>(4, 0)) || isnan(matX.at<float>(5, 0)))
          {
            printf("[USER WARN]laser Odometry: NaN found in var \"matX\", this L-M optimization step is going to be ignored.\n");
          }
          else{
                    transformTobeMapped[0] += matX.at<float>(0, 0);
                    transformTobeMapped[1] += matX.at<float>(1, 0);
                    transformTobeMapped[2] += matX.at<float>(2, 0);
                    transformTobeMapped[3] += matX.at<float>(3, 0);
                    transformTobeMapped[4] += matX.at<float>(4, 0);
                    transformTobeMapped[5] += matX.at<float>(5, 0);
          }


                    float deltaR = sqrt(
                                pow(rad2deg(matX.at<float>(0, 0)), 2) +
                                pow(rad2deg(matX.at<float>(1, 0)), 2) +
                                pow(rad2deg(matX.at<float>(2, 0)), 2));
                    float deltaT = sqrt(
                                pow(matX.at<float>(3, 0) * 100, 2) +
                                pow(matX.at<float>(4, 0) * 100, 2) +
                                pow(matX.at<float>(5, 0) * 100, 2));

                    // std::cout<<"deltaR: "<<deltaR<<" deltaT: "<<deltaT<< std::endl;

                    if (deltaR < 0.05 && deltaT < 0.05) {
                        break;
                    }
                }
                std::cout<<"DEBUG num_temp: "<<num_temp << std::endl;
            }
            else
                n0++;

            transformUpdate();

            t3 = clock();
            
            // if (isPoseValid) 
            // {

            for (int i = 0; i < laserCloudCornerLast->points.size(); i++) {
                pointAssociateToMap(&laserCloudCornerLast->points[i], &pointSel);

                int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
                int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
                int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

                if (pointSel.x + 25.0 < 0) cubeI--;
                if (pointSel.y + 25.0 < 0) cubeJ--;
                if (pointSel.z + 25.0 < 0) cubeK--;

                if (cubeI >= 0 && cubeI < laserCloudWidth &&
                        cubeJ >= 0 && cubeJ < laserCloudHeight &&
                        cubeK >= 0 && cubeK < laserCloudDepth) {

                    int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                    laserCloudCornerArray[cubeInd]->push_back(pointSel);
                }
            }

            for (int i = 0; i < laserCloudSurfLast_down->points.size(); i++) {
                pointAssociateToMap(&laserCloudSurfLast_down->points[i], &pointSel);

                int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
                int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
                int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

                if (pointSel.x + 25.0 < 0) cubeI--;
                if (pointSel.y + 25.0 < 0) cubeJ--;
                if (pointSel.z + 25.0 < 0) cubeK--;

                if (cubeI >= 0 && cubeI < laserCloudWidth &&
                        cubeJ >= 0 && cubeJ < laserCloudHeight &&
                        cubeK >= 0 && cubeK < laserCloudDepth) {
                    int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                    laserCloudSurfArray[cubeInd]->push_back(pointSel);
                }
            }

            for (int i = 0; i < laserCloudValidNum; i++) {
                int ind = laserCloudValidInd[i];

                laserCloudCornerArray2[ind]->clear();
                downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
                downSizeFilterCorner.filter(*laserCloudCornerArray2[ind]);

                laserCloudSurfArray2[ind]->clear();
                downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
                downSizeFilterSurf.filter(*laserCloudSurfArray2[ind]);

                pcl::PointCloud<PointType>::Ptr laserCloudTemp = laserCloudCornerArray[ind];
                laserCloudCornerArray[ind] = laserCloudCornerArray2[ind];
                laserCloudCornerArray2[ind] = laserCloudTemp;

                laserCloudTemp = laserCloudSurfArray[ind];
                laserCloudSurfArray[ind] = laserCloudSurfArray2[ind];
                laserCloudSurfArray2[ind] = laserCloudTemp;
            }

            laserCloudSurround2->clear();
            laserCloudSurround2_corner->clear();
            for (int i = 0; i < laserCloudSurroundNum; i++) {
                int ind = laserCloudSurroundInd[i];
                *laserCloudSurround2_corner += *laserCloudCornerArray[ind];
                *laserCloudSurround2 += *laserCloudSurfArray[ind];
            }

            sensor_msgs::PointCloud2 laserCloudSurround3;
            pcl::toROSMsg(*laserCloudSurround2, laserCloudSurround3);
            laserCloudSurround3.header.stamp = ros::Time().fromSec(timeLaserCloudCornerLast);
            laserCloudSurround3.header.frame_id = "/camera_init";
            pubLaserCloudSurround.publish(laserCloudSurround3);

            sensor_msgs::PointCloud2 laserCloudSurround3_corner;
            pcl::toROSMsg(*laserCloudSurround2_corner, laserCloudSurround3_corner);
            laserCloudSurround3_corner.header.stamp = ros::Time().fromSec(timeLaserCloudCornerLast);
            laserCloudSurround3_corner.header.frame_id = "/camera_init";
            pubLaserCloudSurround_corner.publish(laserCloudSurround3_corner);
            

            laserCloudFullRes2->clear();
            *laserCloudFullRes2 = *laserCloudFullRes;

            int laserCloudFullResNum = laserCloudFullRes2->points.size();
            for (int i = 0; i < laserCloudFullResNum; i++) {

                pcl::PointXYZRGB temp_point;
                RGBpointAssociateToMap(&laserCloudFullRes2->points[i], &temp_point);
                laserCloudFullResColor->push_back(temp_point);
            }

            sensor_msgs::PointCloud2 laserCloudFullRes3;
            pcl::toROSMsg(*laserCloudFullResColor, laserCloudFullRes3);
            laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserCloudCornerLast);
            laserCloudFullRes3.header.frame_id = "/camera_init";
            pubLaserCloudFullRes.publish(laserCloudFullRes3);

            *laserCloudFullResColor_pcd += *laserCloudFullResColor;

            geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                    (transformAftMapped[2], - transformAftMapped[0], - transformAftMapped[1]);

            odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserCloudCornerLast);
            odomAftMapped.pose.pose.orientation.x = -geoQuat.y;
            odomAftMapped.pose.pose.orientation.y = -geoQuat.z;
            odomAftMapped.pose.pose.orientation.z = geoQuat.x;
            odomAftMapped.pose.pose.orientation.w = geoQuat.w;
            odomAftMapped.pose.pose.position.x = transformAftMapped[3];
            odomAftMapped.pose.pose.position.y = transformAftMapped[4];
            odomAftMapped.pose.pose.position.z = transformAftMapped[5];

            pubOdomAftMapped.publish(odomAftMapped);

            static tf::TransformBroadcaster br;
            tf::Transform                   transform;
            tf::Quaternion                  q;
            transform.setOrigin( tf::Vector3( odomAftMapped.pose.pose.position.x,
                                                odomAftMapped.pose.pose.position.y,
                                                odomAftMapped.pose.pose.position.z ) );
            // transform.setOrigin( tf::Vector3(0.0,0.0,0.0) );
            q.setW( odomAftMapped.pose.pose.orientation.w );
            q.setX( odomAftMapped.pose.pose.orientation.x );
            q.setY( odomAftMapped.pose.pose.orientation.y );
            q.setZ( odomAftMapped.pose.pose.orientation.z );
            transform.setRotation( q );
            br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "/camera_init", "/aft_mapped" ) );

            kfNum++;

            if(kfNum >= 20){
            Eigen::Matrix<float,7,1> kf_pose;
            kf_pose << -geoQuat.y,-geoQuat.z,geoQuat.x,geoQuat.w,transformAftMapped[3],transformAftMapped[4],transformAftMapped[5];
            keyframe_pose.push_back(kf_pose);
            kfNum = 0;
            }

            // // }
            // std_msgs::Float32 msgD;
            // msgD.data = 1e-2 * debug_distance;
            // pubDebug9.publish(msgD);

            t4 = clock();

            std::cout<<"mapping time : "<<t2-t1<<" "<<t3-t2<<" "<<t4-t3<<std::endl;            
        }

        status = ros::ok();
        rate.sleep();
    }
    log_file.close();

    //--------------------------save map---------------
    std::string surf_filename(map_file_path + "/surf.pcd");
    std::string corner_filename(map_file_path + "/corner.pcd");
    std::string all_points_filename(map_file_path + "/all_points.pcd");
    std::ofstream keyframe_file(map_file_path + "/key_frame.txt");
    for(auto kf : keyframe_pose){
        keyframe_file << kf[0] << " "<< kf[1] << " "<< kf[2] << " "<< kf[3] << " "
                          << kf[4] << " "<< kf[5] << " "<< kf[6] << " "<< std::endl;
    }
    keyframe_file.close();
    pcl::PointCloud<pcl::PointXYZI> surf_points, corner_points;
    surf_points = *laserCloudSurfFromMap;
    corner_points = *laserCloudCornerFromMap;
      if (surf_points.size() > 0 && corner_points.size() > 0) {
    pcl::PCDWriter pcd_writer;
    std::cout << "saving...";
    pcd_writer.writeBinary(surf_filename, surf_points);
    pcd_writer.writeBinary(corner_filename, corner_points);
    pcd_writer.writeBinary(all_points_filename, *laserCloudFullResColor_pcd);
  } else {
    std::cout << "no points saved";
  }
    //--------------------------
    //  loss_output.close();


    return 0;
}

