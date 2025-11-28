#include <iostream>
#include <ctime>
#include "srcs/Calibrator.h"
#include "srcs/Phaser.h"
#include "srcs/Matcher.h"
#include "srcs/Model.h"

clock_t start_time, end_time;
using namespace std;


struct Config {
    const string sep = "/";
    const string root_dir = "D:/shuangmuText";
    const string data_dir = root_dir + sep + "data1";
    const string calib_dir = data_dir + sep + "calib";
    const string calib_file = calib_dir + sep + "stereoCalib1126.txt";

    const string project_dir = data_dir + sep + "protect";      // 投影
    const string simu_dir = data_dir + sep + "simulation";   // 仿真
    const string model_dir = data_dir + sep + "0.25";        // 测量

    const string output_dir = root_dir + sep + "outputs";
    const string output_dir_L = root_dir + sep + "outputs" + sep + "L";
    const string output_dir_R = root_dir + sep + "outputs" + sep + "R";
    const string save_file_point3d = output_dir + sep + "xyz1.txt";
    const string save_file_pcd = output_dir + sep + "point_cloud.pcd";

    bool         write = true; //写文件会很浪费时间
    bool         show = true;
    // 疑似振幅A、对比度B、频率数量N、图像宽度W、高度H以及三个不同的周期T1、T2、T3
    const double A = 130;
    const double B = 90;
    const int    N = 12;    // 相移步数(12步相移)
    const int    T = 3;     // 三频外差
    const double T1 = 28.;
    const double T2 = 26.;
    const double T3 = 24.;
    //const int    W = 4096;
    //const int    H = 3000;
    const int    W = 1024;
    const int    H = 750;
    const float  B_min = 5;

    const int    win = 3;
    const float  pd = 0.5;
    const float  min_z = 250; //限制深度的最小值和最大值
    const float  max_z = 600;
    const double scale_factor = 0.25; //缩放比例,图片也需要经过缩放才能使用,并且W和H也许需要相应缩放
};

// struct Config {
//     const string sep = "/";
//     const string root_dir = "D:/shuangmuText";
//     const string data_dir = root_dir + sep + "data1";
//     const string calib_dir = data_dir + sep + "calib";
//     const string calib_file = calib_dir + sep + "stereoCalib1126.txt";

//     const string project_dir = data_dir + sep + "protect";      // 投影
//     const string simu_dir = data_dir + sep + "simulation";   // 仿真
//     const string model_dir = data_dir + sep + "1";        // 测量

//     const string output_dir = root_dir + sep + "outputs";
//     const string output_dir_L = root_dir + sep + "outputs" + sep + "L";
//     const string output_dir_R = root_dir + sep + "outputs" + sep + "R";
//     const string save_file_point3d = output_dir + sep + "xyz123.txt";
//     const string save_file_pcd = output_dir + sep + "point_cloud.pcd";

//     bool         write = true;
//     bool         show = true;
//     // 疑似振幅A、对比度B、频率数量N、图像宽度W、高度H以及三个不同的周期T1、T2、T3
//     const double A = 130;
//     const double B = 90;
//     const int    N = 12;    // 相移步数(12步相移)
//     const int    T = 3;     // 三频外差
//     const double T1 = 28.;
//     const double T2 = 26.;
//     const double T3 = 24.;
//     const int    W = 4096;
//     const int    H = 3000;
//     // const int    W = 1024;
//     // const int    H = 750;
//     const float  B_min = 5;

//     const int    win = 3;
//     const float  pd = 0.5;
//     const float  min_z = 250; //限制深度的最小值和最大值
//     const float  max_z = 600;
//     const double scale_factor = 1; //缩放比例,图片也需要经过缩放才能使用,并且W和H也许需要相应缩放
// };



int main() {
    // 添加总计时器
    clock_t total_start = clock();
    clock_t stage_start;

    start_time = clock();

    /** ############# 01 配置文件 ############# **/
    Config config;
    Phaser phaser;
    Calibrator calibrator;

    /** ############# 02 相机标定 ############# **/
    stage_start = clock();
    calibrator.calib(config.calib_file, config.scale_factor);
    cout << "相机标定完成，耗时 " << (clock() - stage_start)/1000. << "S" << endl;

    /** ############# 03 生成相位 ############# **/
    stage_start = clock();
    if (config.write){
        //这三个频率组成的三频差,最多可以标记多宽的图像
        double T123 = phaser.calcT123(config.T1, config.T2, config.T3);
        // 用于写入投影仪 - 不写文件
        phaser.makePatterns(config.A, config.B, config.N, config.W, config.H, config.T1, config.T2, config.T3, config.project_dir, config.write);
        // 用于多频外差仿真 - 不写文件
        phaser.makePatterns(config.A, config.B, config.N, int(T123), 1, config.T1, config.T2, config.T3, config.simu_dir, config.write);
    }
    cout << "生成相位完成，耗时 " << (clock() - stage_start)/1000. << "S" << endl;

    /** ############# 04 解相位 ############# **/
    stage_start = clock();
    cv::Mat pha_L, pha_R;
    cv::Mat B_L, B_R;
    vector<string> files_L, files_R;
    vector<string> filesSimu;
    
    // 文件路径获取
    clock_t file_start = clock();
    globLR(config.T, config.N, config.model_dir, files_L, files_R);
    glob(config.T, config.N, config.simu_dir, filesSimu);
    clock_t file_end = clock();
    cout << "文件路径获取完成，耗时 " << (file_end - file_start)/1000. << "S" << endl;
    cout << "左相机图片数量: " << files_L.size() << ", 右相机图片数量: " << files_R.size() << ", 仿真图片数量: " << filesSimu.size() << endl;
    
    // 左相机解相位
    clock_t left_start = clock();
    phaser.calcPhase(files_L, filesSimu, config.N, config.T1, config.T2, config.T3, pha_L, B_L, config.output_dir_L, config.write);
    clock_t left_end = clock();
    cout << "左相机解相位完成，耗时 " << (left_end - left_start)/1000. << "S" << endl;
    
    // 右相机解相位
    clock_t right_start = clock();
    phaser.calcPhase(files_R, filesSimu, config.N, config.T1, config.T2, config.T3, pha_R, B_R, config.output_dir_R, config.write);
    clock_t right_end = clock();
    cout << "右相机解相位完成，耗时 " << (right_end - right_start)/1000. << "S" << endl;
    
    cout << "解相位完成，总耗时 " << (clock() - stage_start)/1000. << "S" << endl;

    /** ############# 05 相位滤波 ############# **/ 
    stage_start = clock();
    // 调制度滤波
    phaser.filterB(pha_L, B_L, config.B_min);
    phaser.filterB(pha_R, B_R, config.B_min);
    cout << "相位滤波完成，耗时 " << (clock() - stage_start)/1000. << "S" << endl;

    /** ############# 06 立体校正 ############# **/
    stage_start = clock();
    cv::Mat phaC_L, phaC_R;
    calibrator.rectify( pha_L, pha_R, phaC_L, phaC_R);
    // 梯度滤波
    phaser.filterGrad(phaC_L);
    phaser.filterGrad(phaC_R);
    cout << "立体校正完成，耗时 " << (clock() - stage_start)/1000. << "S" << endl;

    ///** ############# 06.5 保存文件 ############# **/
    //if (config.write) {
    //    cv::FileStorage fsL(config.output_dir_L + "/phaC_L.yml", cv::FileStorage::WRITE);
    //    fsL << "mat" << phaC_L;
    //    fsL.release();
    //    
    //    cv::FileStorage fsR(config.output_dir_R + "/phaC_R.yml", cv::FileStorage::WRITE);
    //    fsR << "mat" << phaC_R;
    //    fsR.release();
    //}

    //// 新增读取代码
    //cv::Mat loaded_phaC_L, loaded_phaC_R;
    //cv::FileStorage fsL_read("D:/shuangmuText/outputs/L/phaC_L.yml", cv::FileStorage::READ);
    //fsL_read["mat"] >> loaded_phaC_L;
    //fsL_read.release();
    //
    //cv::FileStorage fsR_read("D:/shuangmuText/outputs/R/phaC_R.yml", cv::FileStorage::READ);
    //fsR_read["mat"] >> loaded_phaC_R;
    //fsR_read.release();
    //cv::Mat phaC_L, phaC_R;
    //// 后续代码改用加载后的矩阵
    //phaC_L = loaded_phaC_L.clone();
    //phaC_R = loaded_phaC_R.clone();

    //// 可选：保存为可视化的PNG
    //cv::Mat vis_L, vis_R;
    //phaC_L.convertTo(vis_L, CV_8UC1, 255.0/(phaser.max_phase - phaser.min_phase));
    //phaC_R.convertTo(vis_R, CV_8UC1, 255.0/(phaser.max_phase - phaser.min_phase));
    //cv::imwrite(config.output_dir_L + "/phaC_L.png", vis_L);
    //cv::imwrite(config.output_dir_R + "/phaC_R.png", vis_R);



    /** ############# 07 相位匹配 ############# **/
    stage_start = clock();
    //// 创建掩码 - 示例：只匹配特定区域
    //cv::Mat left_mask_src = cv::imread("mask.bmp");
    //cv::Mat left_mask_rectified;
    //calibrator.rectifyLeft(left_mask_src, left_mask_rectified);
    //cv::Mat mask;
    //if (!left_mask_rectified.empty()) {
    //    mask = left_mask_rectified.clone();
    //    if (mask.channels() > 1) {
    //        cv::cvtColor(mask, mask, cv::COLOR_BGR2GRAY);  // 如果是彩色图像就转换为灰度图
    //    }
    //}
    //单独测试
    cv::Mat mask = cv::Mat::zeros(phaC_L.size(), CV_8UC1);
    // ... 在mask上标记需要匹配的区域，例如：
    mask.setTo(1);  // 将全部元素设置为1
    //cv::rectangle(mask, cv::Point(400, 400), cv::Point(500, 500), 255, -1);  // 矩形区域
    // 或者从其他处理结果得到mask
    vector<pair<cv::Point2f, cv::Point2f>> cps;
    Matcher matcher;
    matcher.init(config.win, config.pd);
    matcher.phaseMatch(phaC_L, phaC_R, mask, cps);
    cout << "相位匹配完成，耗时 " << (clock() - stage_start)/1000. << "S" << endl;

    /** ############# 08 计算点云 ############# **/
    stage_start = clock();
    // 定义颜色映射表
    map<int, Model::PointColor> colorMap = {
        {0, Model::PointColor(255, 0, 0)},    // 标签0使用红色
        {1, Model::PointColor(0, 255, 0)},    // 标签1使用绿色
        {2, Model::PointColor(0, 0, 255)}     // 标签2使用蓝色
    };

    cv::Mat dif_map = cv::Mat::zeros(phaC_L.size(), CV_32FC1);
    cv::Mat depth_map = cv::Mat::zeros(phaC_R.size(), CV_32FC1);
    Model model;
    model.init(calibrator.Q);
    model.calcDistance(dif_map,depth_map,cps);
    model.saveXYZ(config.save_file_point3d, depth_map, config.min_z, config.max_z);
    // 保存带标签和颜色的点云
    model.savePCD(config.save_file_pcd, depth_map, config.min_z, config.max_z, 1, colorMap);
    cout << "计算点云完成，耗时 " << (clock() - stage_start)/1000. << "S" << endl;

    /** ############# 09 查看结果 ############# **/
    cout << "\n总耗时:\t" << (clock() - total_start)/1000. << "S" << endl;
    end_time = clock();
    cout << "Cost time:\t" << (end_time - start_time) / 1000. << "S" << endl;
    if (config.write){
        saveMat(config.output_dir_L + "/pha.txt", pha_L);
        saveMat(config.output_dir_R + "/pha.txt", pha_R);
        saveMat(config.output_dir_L + "/phaC_L.txt", phaC_L);
        saveMat(config.output_dir_R + "/phaC_R.txt", phaC_R);

        cv::Mat depth_map_64;
        depth_map.convertTo(depth_map_64, CV_64FC1);
        saveMat(config.output_dir + "/depth.txt", depth_map_64);
    }
    if (config.show){
        showImagePair(pha_L, pha_R, "pha", 0.5, false, true);
        showImagePair(phaC_L, phaC_R, "phaC", 0.5, true, true);
    }
    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}
