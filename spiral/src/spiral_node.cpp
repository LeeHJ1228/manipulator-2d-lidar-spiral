#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <deque>

class Spiralpoint {
public:
    Spiralpoint() {
        accumulated_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("accumulated_pointcloud", 1);
        scan_sub = nh.subscribe("scan", 1, &Spiralpoint::scanCallback, this);
        joint_state_sub = nh.subscribe("/joint_states", 1, &Spiralpoint::jointStateCallback, this);

        dh_parameters <<  0.0, 0.0, 0.0,
                          0.0, 1.507, 0.0,
                          0.13, 0.0, 0.0,
                          0.126, 0.0, 0.0,
                          0.124, -1.507, 0.0;

        joint_positions.resize(5);
        joint_positions.setZero();
        accumulated_cloud->width = 0;
        accumulated_cloud->height = 1;
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state) {
                joint_count++;
        ros::Time current_time = ros::Time::now();
        double elapsed_time = (current_time - last_joint_time).toSec();

        if (elapsed_time >= 1.0) {  // 1초마다 주기 출력
            double joint_hz = joint_count / elapsed_time;
            ROS_INFO_STREAM("[JOINT STATE] Publishing rate: " << joint_hz << " Hz");
            joint_count = 0;
            last_joint_time = current_time;
        }
        Eigen::VectorXf temp_positions(5);
        for (int i = 0; i < joint_state->position.size(); ++i) {
            if (joint_state->name[i] == "joint1") {
                temp_positions(0) = joint_state->position[i];
            }
            else if (joint_state->name[i] == "joint2") {
                temp_positions(1) = -joint_state->position[i] + 1.379;
            }
            else if (joint_state->name[i] == "joint3") {
                temp_positions(2) = -joint_state->position[i] - 1.379;
            }
            else if (joint_state->name[i] == "joint4") {
                temp_positions(3) = -joint_state->position[i];
            }
        }
        temp_positions(4) = 0; 
        joint_history.push_back(temp_positions); //저장해서 적용 
        if (joint_history.size() > 30) {
            joint_history.pop_front();
        }
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        int scan_points = 0;
                scan_count++;  
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lidar(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_world(new pcl::PointCloud<pcl::PointXYZ>);

        ros::Time scan_start_time = scan->header.stamp;
        size_t num_points = scan->ranges.size();
        size_t prev_joint_index = -1; // 이전 조인트 인덱스를 저장하기 위한 변수
        size_t points_per_joint_state = 70; // (1000~960)/14 = 71.xxxx~69.xxxxxx =70 
    
        ROS_INFO_STREAM("Total points in scan: " << num_points);
        for (size_t i = 0; i < num_points; ++i) {
            size_t joint_index = std::min(i / points_per_joint_state, joint_history.size() - 1);
            Eigen::Matrix4f transform_matrix = calculateTransforms(joint_history[joint_index]);
           
          if (joint_index != prev_joint_index) { // 조인트 인덱스가 변경될 때만 출력
            ROS_INFO_STREAM("Using joint state index: " << joint_index);
            ROS_INFO_STREAM("Joint values: " << joint_history[joint_index].transpose());
            ROS_INFO_STREAM("Transformation matrix: \n" << calculateTransforms(joint_history[joint_index]));
            prev_joint_index = joint_index;
        }
            float range = scan->ranges[i];
            if (!std::isinf(range)) {
                float angle = scan->angle_min + i * scan->angle_increment;
                pcl::PointXYZ point;
                point.x = -range * cos(angle);
                point.y = -range * sin(angle);
                point.z = 0;
                cloud_lidar->points.push_back(point); 

         
                pcl::PointXYZ transformed_point;
                transformed_point.x = transform_matrix(0, 0) * point.x + transform_matrix(0, 1) * point.y + transform_matrix(0, 2) * point.z + transform_matrix(0, 3);
                transformed_point.y = transform_matrix(1, 0) * point.x + transform_matrix(1, 1) * point.y + transform_matrix(1, 2) * point.z + transform_matrix(1, 3);
                transformed_point.z = transform_matrix(2, 0) * point.x + transform_matrix(2, 1) * point.y + transform_matrix(2, 2) * point.z + transform_matrix(2, 3);
                cloud_world->points.push_back(transformed_point);
                 scan_points++;  // 유효한 점 개수 증가
            }
        }

        *accumulated_cloud += *cloud_world;
        sensor_msgs::PointCloud2 accumulated_cloud_msg;
        pcl::toROSMsg(*accumulated_cloud, accumulated_cloud_msg);
        accumulated_cloud_msg.header.frame_id = "map";
        accumulated_cloud_msg.header.stamp = ros::Time::now();
        cloud_pub.publish(accumulated_cloud_msg);
         ros::Time current_time = ros::Time::now();
        double elapsed_time = (current_time - last_scan_time).toSec();

        if (elapsed_time >= 1.0) {  // 1초마다 주기 출력
            double scan_hz = scan_count / elapsed_time;
            ROS_INFO_STREAM("[LIDAR SCAN] Publishing rate: " << scan_hz << " Hz, Points per scan: " << scan->ranges.size());
            scan_count = 0;
            last_scan_time = current_time;
        }
        ROS_INFO("Current scan points: %d", scan_points);
    }

    Eigen::Matrix4f calculateTransforms(const Eigen::VectorXf& joint_positions) {     
         // 디버깅용 출력문     
        Eigen::Matrix4f T01, T02, T03, T04, T12, T23, T34, T45;
        T01 = T02 = T03 = T04 = T12 = T23 = T34 = T45 = Eigen::Matrix4f::Identity(); // 각 관절의 변환 행렬 초기화

        for (int i = 0; i < 5; ++i) {
                       
            float a = dh_parameters(i, 0);
            float alpha = dh_parameters(i, 1);
            float d = dh_parameters(i, 2);
            float theta = joint_positions(i);
          
            Eigen::Matrix4f T;
            T <<             cos(theta),              -sin(theta),           0,               a,
                sin(theta) * cos(alpha),  cos(theta) * cos(alpha), -sin(alpha), -sin(alpha) * d,
                sin(theta) * sin(alpha),  cos(theta) * sin(alpha),  cos(alpha),  cos(alpha) * d,
                                      0,                        0,           0,              1;

            if (i == 0) {
                T01 = T;
            } else if (i == 1) {
                T12 = T;
            } else if (i == 2) {
                T23 = T;
            } else if (i == 3) {
                T34 = T;
            } else if (i == 4) {
                T45 = T;
            }            
        }
        
        T02 = T01 * T12;             // Joint 2까지의 변환 행렬
        T03 = T01 * T12 * T23;       // Joint 3까지의 변환 행렬
        T04 = T01 * T12 * T23 * T34; // Joint 4가지의 변환 행렬
        end_effector_T = T01 * T12 * T23 * T34 * T45; // 모든 관절의 변환 행렬을 곱하여 최종 엔드 이펙터의 변환 행렬을 얻음
        end_effector_T(2, 3) = end_effector_T(2, 3) + (0.077 + 0.09); //DH parameter로 구현되지 못한 z축 0.077 보정 + 라이다 높이 보정 (0.09)

        return end_effector_T;
        
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber scan_sub, joint_state_sub;
    ros::Publisher cloud_pub;
    std::deque<Eigen::VectorXf> joint_history;
    Eigen::VectorXf joint_positions;
    Eigen::Matrix<float, 5, 3> dh_parameters;
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud;
    Eigen::Matrix4f end_effector_T;
    Eigen::Matrix4f T01;
    Eigen::Matrix4f T02;
    Eigen::Matrix4f T03;
    ros::Time last_joint_time;
    ros::Time last_scan_time;
    int joint_count;
    int scan_count;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "spiral_node");

    // 클래스 객체 생성 후 실행
    Spiralpoint spiralpoint;
    ros::Rate loop_rate(100); // 속도 조절
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
