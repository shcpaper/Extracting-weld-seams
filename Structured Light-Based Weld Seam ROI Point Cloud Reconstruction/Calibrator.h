#ifndef CH7_STRUCTURELIGHTSTEREOTRIANGLE_CALIBRATOR_H  // 防止头文件重复包含的宏定义开始
#define CH7_STRUCTURELIGHTSTEREOTRIANGLE_CALIBRATOR_H  // 防止头文件重复包含的宏定义

// 包含所需的头文件
#include <map>        // 提供map容器
#include<iostream>    // 提供输入输出流功能
#include <regex>      // 提供正则表达式功能
#include <opencv2/opencv.hpp>  // OpenCV主要头文件
#include "utils.h"    // 项目自定义工具函数
//123

using namespace std;  // 使用标准命名空间

class Calibrator {    // 定义相机标定器类
public:
    cv::Mat Q;        // 视差到深度变换矩阵
    cv::Size imageSize;  // 图像尺寸

protected:
    cv::Mat
    KK_L, KK_R,                                     // 左右相机的内参矩阵
    RadialDistortion_L, RadialDistortion_R,         // 左右相机的径向畸变参数
    TangentialDistortion_L, TangentialDistortion_R, // 左右相机的切向畸变参数
    R, T, E, F,                                     // 旋转矩阵、平移向量、本质矩阵、基础矩阵
    error,                                          // 标定误差
    Dist_L, Dist_R;                                 // OpenCV格式的畸变系数

    cv::Mat rmap_L[2], rmap_R[2];                  // 左右相机的重映射矩阵
    cv::Mat R_L, R_R, P_L, P_R;                    // 立体校正后的旋转矩阵和投影矩阵
    cv::Rect validROI_L, validROI_R;               // 立体校正后的有效区域

    cv::Mat diag_scale;  // 添加新的成员变量存储对角矩阵(用于图片降采样)

public:
    // 从Matlab标定结果文件中加载参数
    bool calib(const string &calib_file, double scale_factor = 1.0){
        // 创建对角矩阵
        diag_scale = (cv::Mat_<double>(3,3) << 
            scale_factor, 0.0, 0.0,
            0.0, scale_factor, 0.0,
            0.0, 0.0, 1.0);
        // 打开标定文件
        ifstream in(calib_file);  // 打开标定文件，创建输入文件流对象in，用于读取标定参数文件
        if (in.fail()){
            throw logic_error("file not existed！");  // 文件不存在则抛出异常
            return false;
        }
        cout << "read calib file:" << calib_file << endl;
        
        regex p{".*[a-zA-Z]+.*"};  // 定义正则表达式，用于匹配包含字母的行
        string line, key;           // 存储读取的行和关键字
        int row, col;               // 矩阵的行列数
        vector<double> vec;         // 一维向量：存储一行数据
        vector<vector<double>> matrix{};     // 二维向量：存储多行数据（矩阵）
        
        // 逐行读取文件
        while (getline(in, line) && in.good()){
            // 如果是包含字母的行，表示新的参数开始
            if (regex_match(line, p)){
                key = line;         // 保存参数名
                matrix.clear();     // 清空之前的矩阵数据
            }
            else{
                // 如果成功解析数据行
                if (split(line, vec, "\t")){   // 将字符串按制表符\t分割为向量
                    matrix.emplace_back(vec);  // 将数据添加到矩阵(后面)
                }
                // 空行表示一个参数读取结束
                else{
                    // 将vector转换为OpenCV的Mat格式
                    row = matrix.size();
                    col = matrix[0].size();
                    cv::Mat M = cv::Mat_<double>(row, col);
                    for (int r = 0; r < row; ++r) {
                        for (int c = 0; c < col; ++c) {
                            M.at<double>(r, c) = matrix[r][c];
                        }
                    }
                    
                    // 根据参数名将数据赋值给相应的成员变量
                    cout << "###################" << endl;
                    if (key == "imageSize"){
                        imageSize = cv::Size2l(int(M.at<double>(1) * scale_factor), int(M.at<double>(0) * scale_factor));
                    }
                    else if (key == "KK_L"){
                        //KK_L = M.t();        // 转置是因为Matlab和OpenCV的矩阵存储方式不同
                        KK_L = M * diag_scale;  // 应用缩放
                        KK_L = KK_L.t();
                        cout << "KK_L:" << endl;
                        cout << KK_L << endl;
                    }
                    // ... 其他参数的赋值过程类似
                    else if (key == "KK_R"){
                        //KK_R = M.t();
                        KK_R = M * diag_scale;  // 应用缩放
                        KK_R = KK_R.t();
                        cout << "KK_R:" << endl;
                        cout << KK_R << endl;
                    }
                    else if (key == "RadialDistortion_L"){
                        RadialDistortion_L = M;
                        cout << "RadialDistortion_L:" << endl;
                        cout << RadialDistortion_L << endl;
                    }
                    else if (key == "RadialDistortion_R"){
                        RadialDistortion_R = M;
                        cout << "RadialDistortion_R:" << endl;
                        cout << RadialDistortion_R << endl;
                    }
                    else if (key == "TangentialDistortion_L"){
                        TangentialDistortion_L = M;
                        cout << "TangentialDistortion_L:" << endl;
                        cout << TangentialDistortion_L << endl;
                    }
                    else if (key == "TangentialDistortion_R"){
                        TangentialDistortion_R = M;
                        cout << "TangentialDistortion_R:" << endl;
                        cout << TangentialDistortion_R << endl;
                    }
                    else if (key == "R"){
                        R = M.t();
                        cout << "R" << endl;
                        cout << R << endl;
                    }
                    else if (key == "T"){
                        T = M.t();
                        cout << "T:" << endl;
                        cout << T << endl;
                    }
                    else if (key == "E"){    // 本质矩阵暂不处理
//                    E = M.t();
//                    cout << "E:" << endl;
//                    cout << E << endl;
                    }
                    else if (key == "F"){    // 基础矩阵暂不处理
//                    F = M.t();
//                    cout << "F:" << endl;
//                    cout << F << endl;
                    }
                    else if (key == "error"){
                        error = M;
                        cout << "error:" << endl;
                        cout << error << endl;
                    }
                    else{
                        cerr << key << endl;
                        cerr << "error key" << endl;
                        return false;
                    }
                }
            }
        }
        
        // 将Matlab格式的畸变参数转换为OpenCV格式
        // Matlab：k1, k2, k3, p1, p2
        // OpenCV：k1, k2, p1, p2, k3
        Dist_L = (cv::Mat_<double>(5, 1) <<
                RadialDistortion_L.at<double>(0),
                RadialDistortion_L.at<double>(1),
                TangentialDistortion_L.at<double>(0),
                TangentialDistortion_L.at<double>(1),
                0.
        );
        Dist_R = (cv::Mat_<double>(5, 1) <<
                RadialDistortion_R.at<double>(0),
                RadialDistortion_R.at<double>(1),
                TangentialDistortion_R.at<double>(0),
                TangentialDistortion_R.at<double>(1),
                0.
        );

        // 计算立体校正参数
        cv::stereoRectify(
                KK_L, Dist_L,           // 左相机内参和畸变系数
                KK_R, Dist_R,           // 右相机内参和畸变系数
                imageSize, R, T,         // 图像尺寸、旋转矩阵、平移向量
                R_L, R_R, P_L, P_R, Q,   // 输出：校正旋转矩阵、投影矩阵、视差转深度矩阵
                cv::CALIB_ZERO_DISPARITY, // 标定选项
                1, imageSize,            // 缩放因子和输出图像尺寸
                &validROI_L, &validROI_R // 输出：有效区域
        );
        
        // 计算重映射矩阵，用于后续的图像校正
        cv::initUndistortRectifyMap(KK_L, Dist_L, R_L, P_L, imageSize, CV_16SC2, rmap_L[0], rmap_L[1]);
        cv::initUndistortRectifyMap(KK_R, Dist_R, R_R, P_R, imageSize, CV_16SC2, rmap_R[0], rmap_R[1]);
        cout << "finish calib!" << endl;
        return true;
    }

    // 对输入的左右图像进行立体校正
    bool rectify(cv::Mat &imgL_src, cv::Mat &imgR_src, cv::Mat &imgL_des, cv::Mat &imgR_des){
        // 首先进行单目畸变校正
        cv::undistort(imgL_src, imgL_des, KK_L, Dist_L);
        cv::undistort(imgR_src, imgR_des, KK_R, Dist_R);
        // 然后进行立体校正（重映射）
        cv::remap(imgL_des, imgL_des, rmap_L[0], rmap_L[1], cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        cv::remap(imgR_des, imgR_des, rmap_R[0], rmap_R[1], cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        return true;
    }

    // 添加新方法：单独对左图进行校正
    bool rectifyLeft(cv::Mat &imgL_src, cv::Mat &imgL_des) {
        // 单目畸变校正
        cv::undistort(imgL_src, imgL_des, KK_L, Dist_L);
        // 立体校正（重映射）
        cv::remap(imgL_des, imgL_des, rmap_L[0], rmap_L[1], cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        return true;
    }
};

#endif //CH7_STRUCTURELIGHTSTEREOTRIANGLE_CALIBRATOR_H  // 防止头文件重复包含的宏定义结束
