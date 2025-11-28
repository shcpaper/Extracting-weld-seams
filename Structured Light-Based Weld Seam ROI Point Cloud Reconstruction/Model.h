#ifndef CH7_STRUCTURELIGHTSTEREOTRIANGLE_MODEL_H
#define CH7_STRUCTURELIGHTSTEREOTRIANGLE_MODEL_H
#include <opencv2/opencv.hpp>
#include "utils.h"
using namespace std;



class Model {
    cv::Mat Q;
public:
    bool init(cv::Mat &Q){
        this->Q = Q;
        return true;
    }

    bool calcDistance(cv::Mat &dif_map, cv::Mat &depth_map, vector<pair<cv::Point2f, cv::Point2f>> &cps){
        // 将视差图、深度图初始化为0
        // https://blog.csdn.net/u011722133/article/details/79422942
        for (auto &cp: cps) {
            cv::Point2f l = cp.first, r = cp.second;
            float d = l.x - r.x;
            if (d <= 0){
                d = 0.0;
            }
            dif_map.at<float>(int(l.y), int(l.x)) = d;
        }
        // 可选，去除噪声点，但会降低准确度
        cv::medianBlur(dif_map, dif_map, 3);

        // depth_map: 映射后存储三维坐标的图像
        cv::reprojectImageTo3D(dif_map, depth_map, Q, false, CV_32FC1);
        return true;
    }

    bool saveXYZ(const string &filename, cv::Mat &depth_map, float min_z, float max_z){
        FILE* fp = fopen(filename.data(), "wt");
        // 对边界不进行扫描
        int edge = 5;
        for (int row = edge; row < depth_map.rows - edge; row++)
        {
            for (int col = edge; col < depth_map.cols - edge; col++)
            {
                float point_x = depth_map.at<cv::Vec3f>(row, col)[0];
                float point_y = depth_map.at<cv::Vec3f>(row, col)[1];
                float point_z = depth_map.at<cv::Vec3f>(row, col)[2];
                if (point_z > max_z || point_z < min_z)
                {
                    point_x = 0.;
                    point_y = 0.;
                    point_z = 0.;
                    depth_map.at<cv::Vec3f>(row, col)[0] = point_x;
                    depth_map.at<cv::Vec3f>(row, col)[1] = point_y;
                    depth_map.at<cv::Vec3f>(row, col)[2] = point_z;
                }
                fprintf(fp, "%f %f %f\n", point_x, point_y, point_z);
            }
        }
        fclose(fp);
        cout << "Write 3d point into file: \t" << filename << endl;
        return true;
    }

    // 定义颜色结构体
    struct PointColor {
        uint8_t r, g, b;
        PointColor(uint8_t r = 255, uint8_t g = 255, uint8_t b = 255) : r(r), g(g), b(b) {}
    };

    // 添加新的保存PCD的函数
    bool savePCD(const string &filename, cv::Mat &depth_map, float min_z, float max_z, 
                 int label = 0, const map<int, PointColor> &colorMap = {}) {
        // 首先计算有效点的数量
        int valid_points = 0;
        int edge = 5;
        for (int row = edge; row < depth_map.rows - edge; row++) {
            for (int col = edge; col < depth_map.cols - edge; col++) {
                float z = depth_map.at<cv::Vec3f>(row, col)[2];
                if (z >= min_z && z <= max_z) {
                    valid_points++;
                }
            }
        }

        // 打开文件
        FILE* fp = fopen(filename.data(), "w");
        if (!fp) return false;

        // 写入PCD文件头
        fprintf(fp, "# .PCD v0.7 - Point Cloud Data file format\n");
        fprintf(fp, "VERSION 0.7\n");
        fprintf(fp, "FIELDS x y z rgb label\n");  // 添加rgb和label字段
        fprintf(fp, "SIZE 4 4 4 4 4\n");
        fprintf(fp, "TYPE F F F U U\n");  // rgb和label使用无符号整型
        fprintf(fp, "COUNT 1 1 1 1 1\n");
        fprintf(fp, "WIDTH %d\n", valid_points);
        fprintf(fp, "HEIGHT 1\n");
        fprintf(fp, "VIEWPOINT 0 0 0 1 0 0 0\n");
        fprintf(fp, "POINTS %d\n", valid_points);
        fprintf(fp, "DATA ascii\n");

        // 获取当前标签对应的颜色，如果没有指定则使用默认白色
        PointColor color(255, 255, 255);
        if (!colorMap.empty()) {
            auto it = colorMap.find(label);
            if (it != colorMap.end()) {
                color = it->second;
            }
        }

        // 将RGB打包成一个32位整数
        uint32_t rgb = ((uint32_t)color.r << 16 | (uint32_t)color.g << 8 | (uint32_t)color.b);

        // 写入点云数据
        for (int row = edge; row < depth_map.rows - edge; row++) {
            for (int col = edge; col < depth_map.cols - edge; col++) {
                float x = depth_map.at<cv::Vec3f>(row, col)[0];
                float y = depth_map.at<cv::Vec3f>(row, col)[1];
                float z = depth_map.at<cv::Vec3f>(row, col)[2];
                
                if (z >= min_z && z <= max_z) {
                    fprintf(fp, "%f %f %f %u %d\n", x, y, z, rgb, label);
                }
            }
        }

        fclose(fp);
        cout << "Write PCD file with label " << label << ": \t" << filename << endl;
        return true;
    }
};


#endif //CH7_STRUCTURELIGHTSTEREOTRIANGLE_MODEL_H
