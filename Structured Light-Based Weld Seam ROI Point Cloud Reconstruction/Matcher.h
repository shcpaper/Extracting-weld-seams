#ifndef CH7_STRUCTURELIGHTSTEREOTRIANGLE_MATCHER_H
#define CH7_STRUCTURELIGHTSTEREOTRIANGLE_MATCHER_H
#include <opencv2/opencv.hpp>
#include "Spline.h"
#include <omp.h>
#include <iomanip>
#include <algorithm>
#include <iostream>

using namespace std;

/**
 * @brief 相位匹配器类
 * 用于在结构光立体视觉中进行相位匹配
 */
class Matcher {
    cv::Size winSize;  // 匹配窗口大小
    int wh;           // 窗口半宽
    float pha_dif;    // 相位差阈值
    int maxDisparity = 750; // 默认值，将在init函数中根据图像尺寸动态调整
    int sample_step = 1;   // 采样步长，2表示每隔2个像素采样一次
    
    // 性能优化缓存变量
    int win_size_sq;      // 窗口面积缓存
    double pha_dif_sq;    // 相位差阈值的平方，用于快速比较

public:
    /**
     * @brief 初始化匹配器参数
     * @param win_size 匹配窗口大小
     * @param pha_dif 相位差阈值
     * @return 初始化是否成功
     */
    bool init(int win_size, float pha_dif) {
        winSize = cv::Size(win_size, win_size);
        wh = win_size / 2;
        this->pha_dif = pha_dif;
        
        // 预计算优化参数
        win_size_sq = winSize.width * winSize.height;
        pha_dif_sq = pha_dif * pha_dif;
        
        cout << "Matcher initialized: window=" << win_size << "x" << win_size 
             << ", phase_diff_threshold=" << pha_dif 
             << ", max_disparity=" << maxDisparity << endl;
        
        return true;
    }
    
    /**
     * @brief 根据图像尺寸动态调整最大视差范围
     * @param image_width 图像宽度
     */
    void adjustMaxDisparity(int image_width) {
        // 根据图像宽度动态计算最大视差，确保能处理大图像
        //maxDisparity = min(image_width / 4, 2000);
        maxDisparity = image_width;
        cout << "Adjusted maxDisparity to " << maxDisparity << " for image width " << image_width << endl;
    }
    
    /**
     * @brief 设置优化参数以提升性能但保持精度
     * @param max_disp 最大视差范围
     */
    void setOptimizationParams(int max_disp = 720) {
        maxDisparity = max(50, max_disp);
        cout << "Optimization params set: maxDisparity=" << maxDisparity << endl;
    }
    /**bool init(int win_size, float pha_dif, int max_disp = 320) {
    winSize = cv::Size(win_size, win_size);
    wh = win_size / 2;
    this->pha_dif = pha_dif;
    this->maxDisparity = max_disp;  // 新增参数
    return false;
    }*/ 

    /**
     * @brief 执行相位匹配（并行版本）
     * @param phaL 左图相位图
     * @param phaR 右图相位图
     * @param mask 掩码矩阵
     * @param cps 输出的对应点对
     * @return 是否找到匹配点对
     */
    bool phaseMatch(cv::Mat& phaL, cv::Mat& phaR, const cv::Mat& mask, vector<pair<cv::Point2f, cv::Point2f>>& cps) {
        //  从上到下，从左到右，进行搜索
        // cout << "=== 高精度稠密点云立体匹配开始 ===" << endl;
        cout << "=== High-precision dense point cloud stereo matching started ===" << endl;
        
        // 根据图像尺寸动态调整最大视差范围
        adjustMaxDisparity(phaL.cols);
        
        // 检查OpenMP是否启用
        int max_threads = omp_get_max_threads();
        cout << "OpenMP threads: " << max_threads << endl;
        cout << "Max disparity: " << maxDisparity << endl;
        cout << "Window size: " << winSize.width << "x" << winSize.height << endl;
        
        // 开始计时
        double start_time = omp_get_wtime();

        // 检查掩码是否为空,为空则创建全1掩码
        cv::Mat searchMask;
        if (mask.empty()) {
            searchMask = cv::Mat::ones(phaL.size(), CV_8UC1);
        }
        else {
            CV_Assert(mask.size() == phaL.size());
            searchMask = mask.clone();
        }

        // 预先计算图像尺寸以减少重复计算
        const int rows = phaL.rows;
        const int cols = phaL.cols;
        const int start_row = wh;
        const int end_row = rows - wh;
        const int start_col = wh;
        const int end_col = cols - wh;
        
        cout << "Processing dense point cloud: " << (end_row - start_row) << "x" << (end_col - start_col) << " pixels" << endl;
        
        // 使用线程安全的方式收集匹配结果
        vector<vector<pair<cv::Point2f, cv::Point2f>>> thread_results;
        thread_results.resize(max_threads);
        
        // 预分配内存 - 预估每行能匹配到的点数
        const int estimated_matches_per_thread = (end_row - start_row) * (end_col - start_col) / max_threads / 2;
        for (int i = 0; i < max_threads; ++i) {
            thread_results[i].reserve(estimated_matches_per_thread);
        }

        // 预计算掩码指针以加速访问
        const uchar* mask_data = searchMask.ptr<uchar>();
        const int mask_step = searchMask.step;

        // 优化的并行化策略：使用静态调度以减少同步开销
        const int total_rows = end_row - start_row;
        const int block_size = max(1, total_rows / (max_threads * 4)); // 每个线程处理多个块
        
        cout << "Parallel strategy: " << max_threads << " threads, block_size=" << block_size << endl;

#pragma omp parallel num_threads(max_threads)
        {
            int thread_id = omp_get_thread_num();
            vector<pair<cv::Point2f, cv::Point2f>>& my_results = thread_results[thread_id];
            
            // 每个线程处理指定的行范围
#pragma omp for schedule(static, block_size) nowait
            for (int Y_L = start_row; Y_L < end_row; Y_L++) {
                const uchar* mask_row = mask_data + Y_L * mask_step;
                
                // 批量处理行中的像素以提高缓存效率
                for (int X_L = start_col; X_L < end_col; X_L++) {
                    // 快速掩码检查
                    if (mask_row[X_L] == 0) {
                        continue;  // 跳过不在掩码区域内的点
                    }

                    // 01 在左图中找到相应的有效搜索框
                    cv::Rect roi_L(cv::Point(X_L - wh, Y_L - wh), winSize);
                    cv::Mat BOX_L = phaL(roi_L);

                    if (is_valid_box(BOX_L)) {
                        // 02 执行高度优化的相位搜索
                        int XR = phase_search(BOX_L, phaR, Y_L, wh);
                        
                        if (XR != -1) {
                            // 03 亚像素插值
                            double XR_new = search_sub_pixel(BOX_L, phaR, XR, Y_L);
                            cv::Point2f p_L, p_R;
                            p_L.x = float(X_L); p_L.y = float(Y_L);
                            p_R.x = float(XR_new); p_R.y = float(Y_L);
                            pair<cv::Point2f, cv::Point2f> cp(p_L, p_R);
                            my_results.emplace_back(cp);
                        }
                    }
                }
                
                // 每处理完一定数量的行，输出进度信息
                if (thread_id == 0 && (Y_L - start_row) % (total_rows / 10) == 0) {
                    double progress = double(Y_L - start_row) / total_rows * 100;
                    cout << "Progress: " << int(progress) << "%" << endl;
                }
            }
        }

        // 合并所有线程的结果
        cps.clear();
        for (const auto& thread_result : thread_results) {
            cps.insert(cps.end(), thread_result.begin(), thread_result.end());
        }
        
        // 结束计时和性能统计
        double end_time = omp_get_wtime();
        double elapsed_time = end_time - start_time;
        
        const int total_pixels = (end_row - start_row) * (end_col - start_col);
        const double match_rate = double(cps.size()) / total_pixels * 100;
        
        // cout << "=== 高精度稠密点云立体匹配完成 ===" << endl;
        // cout << "总执行时间:" << elapsed_time << "秒" << endl;
        // cout << "匹配点数量: " << cps.size() << " / " << total_pixels << " 像素" << endl;
        // cout << "匹配成功率: " << fixed << setprecision(2) << match_rate << "%" << endl;
        // cout << "处理速度: " << int(cps.size() / elapsed_time) << " 点/秒" << endl;
        // cout << "像素处理速度: " << int(total_pixels / elapsed_time) << " 像素/秒" << endl;
        // cout << "=== 高精度稠密点云立体匹配完成 ===" << endl;
        cout << "=== High-precision dense point cloud stereo matching completed ===" << endl;
        // cout << "总执行时间: " << elapsed_time << " seconds" << endl;
        // cout << "匹配点数量: " << cps.size() << " / " << total_pixels << " pixels" << endl;
        // cout << "匹配成功率: " << fixed << setprecision(2) << match_rate << "%" << endl;
        // cout << "处理速度: " << int(cps.size() / elapsed_time) << " points/sec" << endl;
        // cout << "像素处理速度: " << int(total_pixels / elapsed_time) << " pixels/sec" << endl;
        cout << "Total execution time: " << elapsed_time << " seconds" << endl;
        cout << "Matched points: " << cps.size() << " / " << total_pixels << " pixels" << endl;
        cout << "Match success rate: " << fixed << setprecision(2) << match_rate << "%" << endl;
        cout << "Processing speed: " << int(cps.size() / elapsed_time) << " points/sec" << endl;
        cout << "Pixel processing speed: " << int(total_pixels / elapsed_time) << " pixels/sec" << endl;
        
        if (elapsed_time > 0) {
            double speedup_estimate = 146.0 / elapsed_time; // 相对于原始146秒的加速比
            // cout << "相对于原始版本加速比: " << fixed << setprecision(1) << speedup_estimate << "x" << endl;
        cout << "Speedup compared to original version: " << fixed << setprecision(1) << speedup_estimate << "x" << endl;
        }
        
        return !cps.empty();
    }



protected:
    /**
     * @brief 检查搜索框是否有效
     * @param BOX 待检查的搜索框
     * @return 搜索框是否有效
     */
    bool is_valid_box(cv::Mat& BOX) {
        double min_v, max_v;
        cv::Point min_pt, max_pt;
        cv::minMaxLoc(BOX, &min_v, &max_v, &min_pt, &max_pt);
        if (min_v <= 0) {
            return false;
        }
        return true;
    }

    /**
     * @brief 高度优化的全局相位搜索（保持精度版本）
     * @param BOX_L 左图搜索框
     * @param phaR 右图相位图
     * @param Y_R 右图Y坐标
     * @param X_R_Start 搜索起始X坐标
     * @return 匹配点的X坐标，-1表示未找到
     */
    int phase_search(cv::Mat& BOX_L, cv::Mat& phaR, int Y_R, int X_R_Start) {
        double l = BOX_L.at<double>(wh, wh);
        double min_std = 1e6;
        int best_XR = -1;
        
        // 预计算数据指针和步长以加速访问
        const double* box_l_data = BOX_L.ptr<double>();
        const int box_step = BOX_L.step / sizeof(double);
        const int local_win_size_sq = win_size_sq; // 使用缓存的窗口面积
        
        // 边界检查和最大搜索范围
        int maxSearch = min(X_R_Start + maxDisparity, phaR.cols - wh - 1);
        if (Y_R - wh < 0 || Y_R + wh >= phaR.rows) return -1;
        
        // 预计算phaR的行指针
        vector<const double*> phaR_rows(winSize.height);
        for (int dy = -wh; dy <= wh; dy++) {
            phaR_rows[dy + wh] = phaR.ptr<double>(Y_R + dy);
        }
        
        // 分阶段搜索以提高效率
        int coarse_step = max(1, maxDisparity / 100); // 粗搜索步长
        
        // 第一阶段：粗搜索
        vector<pair<double, int>> candidates;
        for (int X_R = X_R_Start; X_R <= maxSearch; X_R += coarse_step) {
            // 相位差预筛选
            double r = phaR_rows[wh][X_R];
            if (abs(l - r) > pha_dif) continue;
            if (X_R - wh < 0) continue;
            
            // 快速计算代价（使用部分窗口）
            double sum_sq_diff = 0.0;
            bool valid = true;
            int sample_count = 0;
            
            // 只计算窗口中心的几个点来快速评估
            for (int dy = -wh/2; dy <= wh/2 && valid; dy += 2) {
                const double* phaR_ptr = phaR_rows[dy + wh];
                const double* boxL_ptr = box_l_data + (dy + wh) * box_step;
                
                for (int dx = -wh/2; dx <= wh/2; dx += 2) {
                    double r_val = phaR_ptr[X_R + dx];
                    double l_val = boxL_ptr[dx + wh];
                    
                    if (r_val <= 0 || l_val <= 0) {
                        valid = false;
                        break;
                    }
                    
                    double diff = r_val - l_val;
                    sum_sq_diff += diff * diff;
                    sample_count++;
                }
            }
            
            if (valid && sample_count > 0) {
                double cost = sum_sq_diff / sample_count; // 归一化代价
                candidates.emplace_back(cost, X_R);
            }
        }
        
        if (candidates.empty()) return -1;
        
        // 选择最好的几个候选进行精确搜索
        sort(candidates.begin(), candidates.end());
        int num_refine = min(5, (int)candidates.size());
        
        // 第二阶段：精确搜索
        for (int i = 0; i < num_refine; i++) {
            int center_X = candidates[i].second;
            int search_start = max(X_R_Start, center_X - coarse_step);
            int search_end = min(maxSearch, center_X + coarse_step);
            
            for (int X_R = search_start; X_R <= search_end; X_R++) {
                double r = phaR_rows[wh][X_R];
                if (abs(l - r) > pha_dif) continue;
                if (X_R - wh < 0) continue;
                
                // 完整窗口计算
                double sum_sq_diff = 0.0;
                bool valid = true;
                
                for (int dy = -wh; dy <= wh && valid; dy++) {
                    const double* phaR_ptr = phaR_rows[dy + wh];
                    const double* boxL_ptr = box_l_data + (dy + wh) * box_step;
                    
                    for (int dx = -wh; dx <= wh; dx++) {
                        double r_val = phaR_ptr[X_R + dx];
                        double l_val = boxL_ptr[dx + wh];
                        
                        if (r_val <= 0 || l_val <= 0) {
                            valid = false;
                            break;
                        }
                        
                        double diff = r_val - l_val;
                        sum_sq_diff += diff * diff;
                    }
                }
                
                if (valid) {
                    double std_val = sqrt(sum_sq_diff / local_win_size_sq);
                    if (std_val < min_std) {
                        min_std = std_val;
                        best_XR = X_R;
                    }
                }
            }
        }
        
        return best_XR;
    }

    /**
     * @brief 计算两个搜索框的标准差
     * @param box1 搜索框1
     * @param box2 搜索框2
     * @return 标准差值
     */
    double calc_std(cv::Mat& box1, cv::Mat& box2) {
        cv::Mat v_mat;
        cv::pow(box2 - box1, 2, v_mat);
        return sqrt(cv::sum(v_mat).val[0]);
    }

    /**
     * @brief 简化的亚像素级别搜索
     * @param BOX_L 左图搜索框
     * @param phaR 右图相位图
     * @param XR 右图X坐标
     * @param YR 右图Y坐标
     * @return 亚像素精度的X坐标
     */
    double search_sub_pixel(cv::Mat& BOX_L, cv::Mat& phaR, int XR, int YR) {
        // 边界检查，如果越界直接返回原始值
        if (XR - 1 < 0 || XR + 1 >= phaR.cols || YR - wh < 0 || YR + wh >= phaR.rows) {
            return double(XR);
        }

        // 使用简单的3点抛物线拟合进行亚像素精化
        double costs[3];
        const double* box_l_data = BOX_L.ptr<double>();
        const int box_step = BOX_L.step / sizeof(double);
        
        // 计算左中右三个位置的匹配代价
        for (int i = 0; i < 3; i++) {
            int x_pos = XR - 1 + i;
            double sum_sq_diff = 0.0;
            bool valid = true;
            
            for (int dy = -wh; dy <= wh && valid; dy++) {
                const double* phaR_ptr = phaR.ptr<double>(YR + dy);
                const double* boxL_ptr = box_l_data + (dy + wh) * box_step;
                
                for (int dx = -wh; dx <= wh; dx++) {
                    double r_val = phaR_ptr[x_pos + dx];
                    double l_val = boxL_ptr[dx + wh];
                    
                    if (r_val <= 0 || l_val <= 0) {
                        valid = false;
                        break;
                    }
                    
                    double diff = r_val - l_val;
                    sum_sq_diff += diff * diff;
                }
            }
            
            if (!valid) {
                return double(XR); // 如果计算失败，返回原始位置
            }
            
            costs[i] = sum_sq_diff;
        }
        
        // 抛物线拟合找最小值
        double a = costs[0] - 2 * costs[1] + costs[2];
        if (abs(a) < 1e-6) {
            return double(XR); // 如果是直线，返回原始位置
        }
        
        double b = (costs[2] - costs[0]) * 0.5;
        double offset = -b / a;
        
        // 限制偏移量在合理范围内
        offset = max(-0.5, min(0.5, offset));
        
        return XR + offset;
    }
};


#endif //CH7_STRUCTURELIGHTSTEREOTRIANGLE_MATCHER_H
