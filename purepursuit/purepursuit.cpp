#include <iostream>
#include <vector>
#include <cmath>
#include <tuple>
#include <algorithm>
#include <limits>

class Purepursuit {
public:
    Purepursuit(double wheelbase = 2.5, double k = 0.1, 
                double min_lookahead = 3.0, double max_lookahead = 15.0)
        : wheelbase_(wheelbase), k_(k), 
          min_lookahead_(min_lookahead), max_lookahead_(max_lookahead) {}

    void setReferenceLine(const std::vector<std::pair<double, double>>& ref_line) {
        ref_line_ = ref_line;
        segment_lengths_.clear();
        cumulative_distances_.clear();
        
        if (ref_line.size() > 1) {
            cumulative_distances_.push_back(0.0);
            for (size_t i = 0; i < ref_line.size() - 1; ++i) {
                double dx = ref_line[i+1].first - ref_line[i].first;
                double dy = ref_line[i+1].second - ref_line[i].second;
                double seg_len = std::hypot(dx, dy);
                segment_lengths_.push_back(seg_len);
                cumulative_distances_.push_back(cumulative_distances_.back() + seg_len);
            }
        }
    }

    std::vector<std::tuple<double, double, double, double>> simulate(
        double start_x, double start_y, double start_yaw, double start_velocity,
        double acceleration, double max_velocity, double dt = 0.1, double max_time = 60.0) {
        
        std::vector<std::tuple<double, double, double, double>> trajectory;
        trajectory.push_back(std::make_tuple(start_x, start_y, start_yaw, start_velocity));
        
        double current_x = start_x;
        double current_y = start_y;
        double current_yaw = start_yaw;
        double current_velocity = start_velocity;
        double time = 0.0;
        
        // 初始最近点索引
        int closest_index = findClosestPoint(current_x, current_y);
        int last_target_index = closest_index;

        while (time < max_time) {
            // 计算前瞻距离
            double lookahead = std::clamp(k_ * current_velocity, min_lookahead_, max_lookahead_);
            
            // 优化：从上一帧的目标点开始搜索
            closest_index = findClosestPoint(current_x, current_y, std::max(0, last_target_index - 5));
            
            // 沿参考线搜索目标点
            double accumulated_dist = 0.0;
            int target_index = closest_index;
            
            // 从最近点开始向后搜索
            for (int i = closest_index; i < static_cast<int>(ref_line_.size()) - 1; ++i) {
                accumulated_dist += segment_lengths_[i];
                if (accumulated_dist >= lookahead) {
                    target_index = i + 1;
                    break;
                }
            }
            
            // 如果到达终点
            if (target_index >= static_cast<int>(ref_line_.size()) - 1) {
                target_index = ref_line_.size() - 1;
            }
            
            last_target_index = target_index;
            
            // 计算目标点坐标
            double target_x = ref_line_[target_index].first;
            double target_y = ref_line_[target_index].second;
            
            // 计算转向角
            double dx = target_x - current_x;
            double dy = target_y - current_y;
            double target_angle = std::atan2(dy, dx);
            double alpha = target_angle - current_yaw;
            
            // 角度归一化
            alpha = std::fmod(alpha + M_PI, 2*M_PI);
            if (alpha < 0) alpha += 2*M_PI;
            alpha -= M_PI;
            
            // 计算曲率
            double curvature = 2.0 * std::sin(alpha) / lookahead;
            double steer_angle = std::atan(curvature * wheelbase_);
            
            // 运动学模型更新
            current_velocity += acceleration * dt;
            current_velocity = std::min(current_velocity, max_velocity);
            
            current_x += current_velocity * std::cos(current_yaw) * dt;
            current_y += current_velocity * std::sin(current_yaw) * dt;
            current_yaw += current_velocity * std::tan(steer_angle) / wheelbase_ * dt;
            
            // 角度归一化
            current_yaw = std::fmod(current_yaw + M_PI, 2*M_PI);
            if (current_yaw < 0) current_yaw += 2*M_PI;
            current_yaw -= M_PI;
            
            // 保存当前状态
            trajectory.push_back(std::make_tuple(current_x, current_y, current_yaw, current_velocity));
            time += dt;
            
            // 终点检查（距离终点小于0.5米）
            double end_x = ref_line_.back().first;
            double end_y = ref_line_.back().second;
            if (std::hypot(end_x - current_x, end_y - current_y) < 0.5) break;
        }
        
        return trajectory;
    }

private:
    // 优化后的最近点搜索函数
    int findClosestPoint(double x, double y, int start_index = 0) {
        if (ref_line_.empty()) return -1;
        
        int best_index = start_index;
        double min_dist_sq = std::numeric_limits<double>::max();
        const int window_size = 50;  // 搜索窗口大小
        
        // 确定搜索范围
        int start = std::max(0, start_index - window_size/2);
        int end = std::min(static_cast<int>(ref_line_.size()), start_index + window_size/2);
        
        // 在窗口内搜索最近点
        for (int i = start; i < end; ++i) {
            double dx = ref_line_[i].first - x;
            double dy = ref_line_[i].second - y;
            double dist_sq = dx*dx + dy*dy;
            
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                best_index = i;
            }
        }
        
        return best_index;
    }

    double wheelbase_;      // 车辆轴距
    double k_;              // 前瞻距离系数
    double min_lookahead_;  // 最小前瞻距离
    double max_lookahead_;  // 最大前瞻距离
    
    std::vector<std::pair<double, double>> ref_line_;
    std::vector<double> segment_lengths_;
    std::vector<double> cumulative_distances_;  // 累积距离
};
