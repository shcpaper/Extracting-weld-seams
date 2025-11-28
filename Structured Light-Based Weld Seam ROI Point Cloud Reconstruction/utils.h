#ifndef CH7_STRUCTURELIGHTSTEREOTRIANGLE_UTILS_H
#define CH7_STRUCTURELIGHTSTEREOTRIANGLE_UTILS_H
#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <fstream>
using namespace std;

// 分割字符串
bool split(const string &line, vector<double> &res, const string &pattern){
    if(line.empty())
        return false;
    res.clear();
    stringstream ss(line);
    string data;
    double v;
    char *ptr;
    while (getline(ss, data, '\t')){
        // 将读取的数据转换为
        v = strtod(data.c_str(), &ptr);
        res.emplace_back(v);
    }
    return !res.empty();
}

bool glob(int T, int N, const string &folder, vector<string> &files){
    files.clear();
    int num = T * N;
    for (int i = 1; i <= num; ++i) {
        string filename = folder + "/" + to_string(i) + ".bmp";
        files.emplace_back(filename);
    }
    return !files.empty();
}


/// <summary>
/// 生成左右图像的文件路径列表。
/// </summary>
/// <param name="T">序列数。</param>
/// <param name="N">每个序列的图像数。</param>
/// <param name="folder">基础文件夹路径。</param>
/// <param name="filesL">输出向量，存储左图像的文件路径。</param>
/// <param name="filesR">输出向量，存储右图像的文件路径。</param>
/// <returns>如果成功生成文件路径列表则返回 true，否则返回 false。</returns>
bool globLR(int T, int N, const string &folder, vector<string> &filesL, vector<string> &filesR){
    filesL.clear(); filesR.clear();
    string folderL = folder + "/L";
    string folderR = folder + "/R";
    int num = T * N;
    for (int i = 1; i <= num; ++i) {
        string filename_L = folderL + "/" + to_string(i) + ".bmp";
        string filename_R = folderR + "/" + to_string(i) + ".bmp";
        filesL.emplace_back(filename_L);
        filesR.emplace_back(filename_R);
    }
    return (!filesL.empty()) && (!filesR.empty());
}

void showImagePair(const cv::Mat &imgL, const cv::Mat &imgR, const string &title, float s, bool line, bool color){
    cv::Mat img_all;
    cv::hconcat(imgL, imgR, img_all);
    cv::resize(img_all, img_all, cv::Size(), s, s);
    double min_v, max_v;
    cv::Point min_p, max_p;
    cv::minMaxLoc(img_all, &min_v, &max_v, &min_p, &max_p);
    img_all = (img_all - min_v) * 255. / (max_v - min_v);
    img_all.convertTo(img_all, CV_8UC1);
    if (color){
        cv::applyColorMap(img_all, img_all, cv::COLORMAP_JET);
    }
    // 绘制平行线
    int h = img_all.rows;
    int w = img_all.cols;
    if (line){
        int num = 10;
        int p = h / num;
        for (int i = 0; i < num; ++i) {
            int y = i * p;
            cv::line(img_all, cv::Point(0, y), cv::Point(w, y), cv::Scalar(255, 255, 255), 1);
        }
    }
    cv::imshow(title, img_all);
    cv::waitKey(1);
}

void showImage(const cv::Mat &img, const string &title, float s, bool color){
    cv::Mat img_s;
    cv::resize(img, img_s, cv::Size(), s, s);
    double min_v, max_v;
    cv::Point min_p, max_p;
    cv::minMaxLoc(img_s, &min_v, &max_v, &min_p, &max_p);
    img_s = (img_s - min_v) * 255. / (max_v - min_v);
    img_s.convertTo(img_s, CV_8UC1);
    if (color){
        cv::applyColorMap(img_s, img_s, cv::COLORMAP_JET);
    }
    cv::imshow(title, img_s);
    cv::waitKey(1);
}

// void saveMat(const string &filename, cv::Mat &mat){
//     ofstream f;
//     f.open(filename);
//     cout << "Writing...:\t" << filename << endl;
//     for (int row = 0; row < mat.rows; ++row) {
//         for (int col = 0; col < mat.cols; ++col) {
//             if (col == mat.cols - 1){
//                 f << setprecision(32) << mat.at<double>(row, col) << endl;;
//             } else{
//                 f << mat.at<double>(row, col) << ",";
//             }
//         }
//     }
//     f.close();
//     cout << "Have Written into file:" << filename << endl;
// }
void saveMat(const string &filename, cv::Mat &mat){
    if (mat.empty()) {
        cout << "Matrix is empty, not writing to file: " << filename << endl;
        return;
    }
    if (mat.type() != CV_64F) {
        // For simplicity, this example only handles double matrices.
        // You could add conversions for other types if needed.
        mat.convertTo(mat, CV_64F);
    }

    FILE* pFile = nullptr;
#ifdef _WIN32
    errno_t err = fopen_s(&pFile, filename.c_str(), "w");
    if (err != 0 || pFile == nullptr) {
        cerr << "Error opening file: " << filename << endl;
        return;
    }
#else
    pFile = fopen(filename.c_str(), "w");
    if (pFile == nullptr) {
        cerr << "Error opening file: " << filename << endl;
        return;
    }
#endif

    cout << "Writing...:\t" << filename << endl;


    string buffer;
    buffer.reserve(16 * 1024 * 1024); 
    for (int i = 0; i < mat.rows; ++i) {
        const double* ptr = mat.ptr<double>(i);
        for (int j = 0; j < mat.cols; ++j) {
            buffer += std::to_string(ptr[j]);
            if (j < mat.cols - 1) {
                buffer += ",";
            }
        }
        buffer += "\n";

        if (buffer.size() > 1024 * 1024) { 
            fwrite(buffer.c_str(), 1, buffer.size(), pFile);
            buffer.clear();
        }
    }

    if (!buffer.empty()) {
        fwrite(buffer.c_str(), 1, buffer.size(), pFile);
    }

    fclose(pFile);
    cout << "Have Written into file:" << filename << endl;
}

/// <summary>
/// 将相位差限制在[0, 2π)范围内,如3π会变成π
/// </summary>
/// <param name="a">真实值/param>
/// <param name="m">最大值</param>
/// <returns></returns>
double mod(double a, double m){
    return a - m * floor(a / m);
}


#endif //CH7_STRUCTURELIGHTSTEREOTRIANGLE_UTILS_H
