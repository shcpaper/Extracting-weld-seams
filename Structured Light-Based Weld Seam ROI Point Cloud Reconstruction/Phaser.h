#ifndef CH7_STRUCTURELIGHTSTEREOTRIANGLE_PHASER
#define CH7_STRUCTURELIGHTSTEREOTRIANGLE_PHASER
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#ifdef _OPENMP
#include <omp.h>
#endif
using namespace std;

#define pi acos(-1)

class Phaser{
//在 OpenCV 中，布尔矩阵中的 true 值通常表示为 255（在 8 位无符号整数类型 CV_8U 中），而 false 值表示为 0。
//通过除以 255，将布尔值转换为浮点数 1.0 和 0.0。这样，mask1 矩阵中的每个元素将是 1.0 或 0.0，表示 pha123 中对应的值是否小于2π。

public:
    /// <summary>
    /// 输入参数,制作相移图,最后保存。
    /// </summary>
    /// <param name="A">序列数。</param>
    /// <param name="B">每个序列的图像数。</param>
    /// <param name="folder">基础文件夹路径。</param>
    /// <param name="filesL">输出向量，存储左图像的文件路径。</param>
    /// <param name="filesR">输出向量，存储右图像的文件路径。</param>
    /// <returns>如果成功生成文件路径列表则返回 true，否则返回 false。</returns>
    void makePatterns(double A, double B, int N, int W, int H, double T1, double T2, double T3, const string &save_dir, bool write_files = true){
        cv::Mat I = cv::Mat::zeros(cv::Size(W, 1), CV_64FC1);
        cv::Mat I_img = cv::Mat::zeros(cv::Size(W, H), CV_64FC1);
        int idx = 1;
        if (write_files) {
            cout << "\nsave phase patterns into folder:\t" << save_dir << "..." << endl;
        } else {
            cout << "\ngenerate phase patterns (not saving to disk)..." << endl;
        }
        
        ofstream f;
        if (write_files) {
            f.open(save_dir + "/" + "write.csv", ios::out);
        }
        
        for (auto T: {T1, T2, T3}) {
            for (int k = 0; k < N; ++k) {
                for (int w = 1; w <= W; ++w) {
                    double I_value = A + B * cos(w / double (T) * 2. * pi + 2 * double(k) / double(N) * pi);
                    I.at<double>(w - 1) = I_value;
                    // 写完了，换行符
                    if (write_files) {
                        if (w == W){
                            f << to_string(int(I_value)) << "\n";
                        }
                        else{
                            f << to_string(int(I_value)) << ",";
                        }
                    }
                }
                cv::repeat(I, H, 1, I_img);
                if (write_files) {
                    string save_file = save_dir + "\\" + to_string(idx) + ".bmp";
                    cv::imwrite(save_file, I_img);
                }
                idx += 1;
            }
        }
        
        if (write_files) {
            f.close();
        }
    }

    double calcT123(double T1, double T2, double T3){
        double T12 = calcT12(T1, T2);
        double  T23 = calcT12(T2, T3);
        return calcT12(T12, T23);
    }

    //解相位 - 优化的并行化版本
    void calcPhase(vector<string> &files, vector<string> &filesSimu,
                   int N, double T1, double T2, double T3,
                   cv::Mat &pha, cv::Mat &B,
                   const string &save_dir, bool write) {
        vector<vector<cv::Mat>> imgsSimu(3), imgs(3);
        
        // 01 串行加载图片（保持顺序正确性）
        clock_t load_start = clock();
        loadImages(N, filesSimu, files,
                   imgsSimu[0], imgsSimu[1], imgsSimu[2],
                   imgs[0], imgs[1], imgs[2]);
        clock_t load_end = clock();
        cout << "Image loading completed, time: " << (load_end - load_start) / 1000. << "S" << endl;

        if (imgs[0].empty() || imgs[1].empty() || imgs[2].empty()){
            cerr << "Image vectors are not properly filled. Aborting phase calculation." << endl;
            return;
        }

        // 02 准备数据
        cv::Size img_size = imgs[0][0].size();
        B = cv::Mat::zeros(img_size, CV_64FC1);
        vector<cv::Mat> phaSimu(3), pha_wrapped(3);
        vector<double> T = {T1, T2, T3};

        // 03 高效的并行计算包裹相位
        #ifdef _OPENMP
        #pragma omp parallel for num_threads(3) schedule(static, 1)
        #endif
        for (int i = 0; i < 3; ++i) {
            cv::Mat B_temp; // 临时背景强度
            calcWrappedPhase(imgsSimu[i], N, phaSimu[i], B_temp);
            calcWrappedPhase(imgs[i], N, pha_wrapped[i], B_temp);
            
            #pragma omp critical
            {
                B += B_temp; // 原子性地累加到总B中
            }
        }
        B /= 3.0; // 计算平均强度

        // 04 高效的并行多频外差合成
        cv::Mat phaSimu12, phaSimu23;
        cv::Mat pha12, pha23;
        double T12, T23;
        
        #ifdef _OPENMP
        #pragma omp parallel sections num_threads(2)
        {
            #pragma omp section
            {
                calcUnwrappedPhase(phaSimu[0], phaSimu[1], T[0], T[1], phaSimu12, T12);
                calcUnwrappedPhase(pha_wrapped[0], pha_wrapped[1], T[0], T[1], pha12, T12);
                toStdPhase(pha12, phaSimu12);
            }
            #pragma omp section
            {
                calcUnwrappedPhase(phaSimu[1], phaSimu[2], T[1], T[2], phaSimu23, T23);
                calcUnwrappedPhase(pha_wrapped[1], pha_wrapped[2], T[1], T[2], pha23, T23);
                toStdPhase(pha23, phaSimu23);
            }
        }
        #else
        calcUnwrappedPhase(phaSimu[0], phaSimu[1], T[0], T[1], phaSimu12, T12);
        calcUnwrappedPhase(pha_wrapped[0], pha_wrapped[1], T[0], T[1], pha12, T12);
        toStdPhase(pha12, phaSimu12);

        calcUnwrappedPhase(phaSimu[1], phaSimu[2], T[1], T[2], phaSimu23, T23);
        calcUnwrappedPhase(pha_wrapped[1], pha_wrapped[2], T[1], T[2], pha23, T23);
        toStdPhase(pha23, phaSimu23);
        #endif
        
        // 最终合成
        cv::Mat pha123, phaSimu123;
        double T123;
        calcUnwrappedPhase(phaSimu12, phaSimu23, T12, T23, phaSimu123, T123);
        calcUnwrappedPhase(pha12, pha23, T12, T23, pha123, T123);
        toStdPhase(pha123, phaSimu123);

        // 掩码计算与应用
        cv::Mat mask;
        cv::inRange(pha123, 0.0, 2 * pi, mask);
        mask.convertTo(mask, CV_64FC1, 1.0/255.0);
        pha = pha123.mul(mask);

        // 保存结果
        if (write) {
            saveMat(save_dir + "/" + "B.txt", B);
            saveMat(save_dir + "/" + "phaSimu12.txt", phaSimu12);
            saveMat(save_dir + "/" + "phaSimu23.txt", phaSimu23);
            saveMat(save_dir + "/" + "pha12.txt", pha12);
            saveMat(save_dir + "/" + "pha23.txt", pha23);
            saveMat(save_dir + "/" + "phaSimu123.txt", phaSimu123);
            saveMat(save_dir + "/" + "pha123.txt", pha123);
        }
    }

    /// <summary>
    /// 相位滤波
    /// </summary>
    /// <param name="pha">需要滤波的相位</param>
    /// <param name="B">需要滤波的啥</param>
    /// <param name="B_min">滤波依据</param>
    void filterB(cv::Mat &pha, cv::Mat &B, double B_min){
        cv::Mat B_mask = (B > B_min) / 255.;
        B_mask.convertTo(B_mask, CV_64FC1);
        pha = pha.mul(B_mask);
    }

    /// <summary>
    /// 梯度滤波(开不太懂)
    /// </summary>
    /// <param name="pha"></param>
    void filterGrad(cv::Mat &pha){
        cv::Mat dx, dy;
        cv::Sobel(pha, dx, pha.depth(), 1, 0, 1, 0.5, 0); // x方向
        cv::Sobel(pha, dy, pha.depth(), 0, 1, 1, 0.5, 0); // y方向

        double d_thr = 0.2;
        cv::Mat dx_mask = (cv::abs(dx) < d_thr) / 255.;
        cv::Mat dy_mask = (cv::abs(dy) < d_thr) / 255.;

        dx_mask.convertTo(dx_mask, CV_64FC1);
        dy_mask.convertTo(dy_mask, CV_64FC1);

        cv::Mat dxy_mask = dx_mask.mul(dy_mask);
        // 对掩模进行腐蚀（只要有黑的，就将整个变为黑色的）
        cv::erode(dxy_mask, dxy_mask, cv::Mat());
        
        // 安全的像素级并行化掩码应用
        #ifdef _OPENMP
        #pragma omp parallel for schedule(static, 1) num_threads(omp_get_max_threads())
        #endif
        for (int row = 0; row < dxy_mask.rows; ++row) {
            for (int col = 0; col < dxy_mask.cols; ++col) {
                if (dxy_mask.at<double>(row, col) < 0.5){
                    pha.at<double>(row, col) = 0;
                }
            }
        }
    }

protected:
    double calcT12(double T1, double T2){
        return (T1 * T2) / (T2 - T1);
    }

    /// <summary>
    /// 以灰度模式并行读取图像,并且对图片进行高斯模糊,将图片的灰度值的信息保存在矩阵中
    /// </summary>
    /// <param name="N">相移步数</param>
    /// <param name="filesSimu">模拟相位图片路径</param>
    /// <param name="files">真实图片路径</param>
    /// <param name="imgsSimu1">模拟相位图片1的灰度矩阵</param>
    /// <param name="imgsSimu2">模拟相位图片2的灰度矩阵</param>
    /// <param name="imgsSimu3">模拟相位图片3的灰度矩阵</param>
    /// <param name="imgs1">真实图片1的灰度矩阵</param>
    /// <param name="imgs2">真实图片2的灰度矩阵</param>
    /// <param name="imgs3">真实图片3的灰度矩阵</param>
    void loadImages(int N, vector<string> &filesSimu, vector<string> &files,
                    vector<cv::Mat> &imgsSimu1, vector<cv::Mat> &imgsSimu2, vector<cv::Mat> &imgsSimu3,
                    vector<cv::Mat> &imgs1, vector<cv::Mat> &imgs2, vector<cv::Mat> &imgs3){
        size_t num = files.size();

        cout << "Starting to load " << num << " images in parallel..." << endl;
        clock_t total_start = clock();
        
        // 预分配容器大小以避免动态重分配
        size_t per_group = num / 3;
        imgsSimu1.resize(per_group);
        imgsSimu2.resize(per_group);
        imgsSimu3.resize(per_group);
        imgs1.resize(per_group);
        imgs2.resize(per_group);
        imgs3.resize(per_group);

        atomic<bool> load_error(false);
        atomic<int> loaded_count(0);
        
        // 并行加载图像并直接放入目标位置
        #ifdef _OPENMP
        #pragma omp parallel for schedule(dynamic)
        #endif
        for (int i = 0; i < static_cast<int>(num); ++i) {
            if (load_error) {
                continue;
            }
            
            // 加载图像
            cv::Mat imgSimu = cv::imread(filesSimu[i], 0);
            cv::Mat imgModel = cv::imread(files[i], 0);

            if (imgSimu.empty() || imgModel.empty()){
                #pragma omp critical
                {
                    if (imgSimu.empty()) cerr << "unable to load image:\t" << filesSimu[i] << endl;
                    if (imgModel.empty()) cerr << "unable to load image:\t" << files[i] << endl;
                }
                load_error = true;
                continue;
            }

            // 图像预处理
            cv::GaussianBlur(imgSimu, imgSimu, cv::Size(3,3), 1, 1);
            cv::GaussianBlur(imgModel, imgModel, cv::Size(3,3), 1, 1);

            // 类型转换
            imgSimu.convertTo(imgSimu, CV_64FC1);
            imgModel.convertTo(imgModel, CV_64FC1);

            // 计算目标位置并直接存入
            int r = i / N;
            int j = i % N;

            if (r == 0) {
                imgsSimu1[j] = std::move(imgSimu);
                imgs1[j] = std::move(imgModel);
            } else if (r == 1) {
                imgsSimu2[j] = std::move(imgSimu);
                imgs2[j] = std::move(imgModel);
            } else {
                imgsSimu3[j] = std::move(imgSimu);
                imgs3[j] = std::move(imgModel);
            }

            // 报告进度
            int current_count = ++loaded_count;
            if (current_count % 10 == 0 || current_count == num) {
                #pragma omp critical
                cout << "Loaded " << current_count << "/" << num << " images" << endl;
            }
        }

        if (load_error) {
            cerr << "Failed to load some images. Aborting." << endl;
            // 如果加载失败，清空所有 vector，防止使用不完整的数据
            imgsSimu1.clear(); imgsSimu2.clear(); imgsSimu3.clear();
            imgs1.clear(); imgs2.clear(); imgs3.clear();
            return;
        }
        
        clock_t total_end = clock();
        cout << "Total image loading time: " << (total_end - total_start)/1000. << "S" << endl;
    }
    /// <summary>
    /// 计算包裹相位(数学原理查看md文件) - 向量化版本
    /// </summary>
    void calcWrappedPhase(vector<cv::Mat> &imgs, int N, cv::Mat &pha, cv::Mat &B) {
        if (imgs.empty()) return;
        cv::Size img_size = imgs[0].size();
        cv::Mat sin_sum = cv::Mat::zeros(img_size, CV_64FC1);
        cv::Mat cos_sum = cv::Mat::zeros(img_size, CV_64FC1);

        vector<double> sin_values(N), cos_values(N);
        for (int k = 0; k < N; k++) {
            double pk = 2. * k * pi / N;
            sin_values[k] = sin(pk);
            cos_values[k] = cos(pk);
        }

        for (int k = 0; k < N; k++) {
            cv::scaleAdd(imgs[k], sin_values[k], sin_sum, sin_sum);
            cv::scaleAdd(imgs[k], cos_values[k], cos_sum, cos_sum);
        }

        // 修正: cv::phase(Y, X) 计算 atan2(Y, X)
        // 原始逻辑是 -atan2(s, c), 等价于 atan2(-s, c)
        // 所以 Y 对应 -sin_sum, X 对应 cos_sum
        cv::phase(cos_sum, -sin_sum, pha, false); 
        cv::magnitude(sin_sum, cos_sum, B);
        B = B * (2.0 / N);
    }


    /// <summary>
    /// 从两幅包裹相位图（pha1 和 pha2）中计算出未包裹相位图（pha12） - 向量化版本
    /// </summary>
    void calcUnwrappedPhase(cv::Mat &pha1, cv::Mat &pha2, double T1, double T2, cv::Mat &pha12, double &T12){
        pha12 = cv::Mat::zeros(pha1.size(), CV_64FC1);
        T12 = T1 * T2 / (T2 - T1);
        
        // 安全的像素级并行化 - 每行独立处理
        #ifdef _OPENMP
        #pragma omp parallel for schedule(static, 1) num_threads(omp_get_max_threads())
        #endif
        for (int row = 0; row < pha1.rows; ++row) {
            for (int col = 0; col < pha1.cols; ++col) {
                double pha1_v = pha1.at<double>(row, col);
                double pha2_v = pha2.at<double>(row, col);
                double delta = mod(pha1_v - pha2_v, 2. * pi);
                pha12.at<double>(row, col) = pha1_v + round((delta * T12 / T1 - pha1_v) / (2. * pi)) * 2. * pi;
            }
        }
    }


    /// <summary>
    /// 相位图 pha 和模拟相位图 phaSimu 归一化到 [0, 2π) 范围内。- 优化版本
    /// </summary>
    void toStdPhase(cv::Mat &pha, cv::Mat &phaSimu) {
        double e = 1e-9; // 使用较小的epsilon避免除零
        double pha_min, pha_max;
        cv::minMaxLoc(phaSimu, &pha_min, &pha_max);

        double range = pha_max - pha_min;
        if (range < e) { // 如果范围接近于零，则不进行缩放
            // 可以选择将所有值设为0或保持原样
            pha = cv::Mat::zeros(pha.size(), pha.type());
            phaSimu = cv::Mat::zeros(phaSimu.size(), phaSimu.type());
            return;
        }

        double scale = (2. * pi) / range;
        
        pha = (pha - pha_min) * scale;
        phaSimu = (phaSimu - pha_min) * scale;
    }
};



#endif //CH7_STRUCTURELIGHTSTEREOTRIANGLE_PHASER
