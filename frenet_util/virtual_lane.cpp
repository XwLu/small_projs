#include <vector>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <algorithm>
#include <memory>

// 二维点类型
struct Point2D {
    double x, y;
    Point2D() : x(0), y(0) {}
    Point2D(double x, double y) : x(x), y(y) {}
};

// KD树节点
struct KDNode {
    Point2D point;
    int index;
    std::shared_ptr<KDNode> left;
    std::shared_ptr<KDNode> right;
    
    KDNode(Point2D p, int idx) 
        : point(p), index(idx), left(nullptr), right(nullptr) {}
};

// KD树类
class KDTree {
private:
    std::shared_ptr<KDNode> root_;
    
    std::shared_ptr<KDNode> buildTree(std::vector<Point2D>& points, 
                                     std::vector<int>& indices, 
                                     int start, int end, int depth) {
        if (start >= end) return nullptr;
        
        int axis = depth % 2;
        int mid = start + (end - start) / 2;
        
        // 按当前轴排序
        std::nth_element(indices.begin() + start, indices.begin() + mid, indices.begin() + end,
            [&](int a, int b) {
                return (axis == 0) ? (points[a].x < points[b].x) : (points[a].y < points[b].y);
            });
        
        auto node = std::make_shared<KDNode>(points[indices[mid]], indices[mid]);
        node->left = buildTree(points, indices, start, mid, depth + 1);
        node->right = buildTree(points, indices, mid + 1, end, depth + 1);
        
        return node;
    }
    
    void nearestNeighbor(const std::shared_ptr<KDNode> node, 
                         const Point2D& query, int depth, 
                         double& bestDist, int& bestIndex) const {
        if (!node) return;
        
        double dx = query.x - node->point.x;
        double dy = query.y - node->point.y;
        double distSq = dx*dx + dy*dy;
        
        if (distSq < bestDist) {
            bestDist = distSq;
            bestIndex = node->index;
        }
        
        int axis = depth % 2;
        double diff = (axis == 0) ? dx : dy;
        auto first = (diff < 0) ? node->left : node->right;
        auto second = (diff < 0) ? node->right : node->left;
        
        if (first) {
            nearestNeighbor(first, query, depth + 1, bestDist, bestIndex);
        }
        
        // 检查另一边是否有更近的点
        if (second && diff*diff < bestDist) {
            nearestNeighbor(second, query, depth + 1, bestDist, bestIndex);
        }
    }

public:
    KDTree(const std::vector<Point2D>& points) {
        if (points.empty()) return;
        
        // 创建索引数组
        std::vector<int> indices(points.size());
        for (int i = 0; i < points.size(); ++i) {
            indices[i] = i;
        }
        
        // 复制点集用于构建树（避免修改原始数据）
        std::vector<Point2D> points_copy = points;
        
        root_ = buildTree(points_copy, indices, 0, points.size(), 0);
    }
    
    int findNearest(const Point2D& query) const {
        double bestDist = std::numeric_limits<double>::max();
        int bestIndex = -1;
        nearestNeighbor(root_, query, 0, bestDist, bestIndex);
        return bestIndex;
    }
};

class VirtualLane {
private:
    std::vector<Point2D> ref_points_;  // 参考线点序列
    std::vector<double> s_vec_;        // 每个参考点的累计弧长
    std::vector<double> theta_vec_;    // 每个参考点的切线角度 (弧度)
    double total_s_;                   // 参考线总长度
    std::unique_ptr<KDTree> kd_tree_;  // KD树加速最近点搜索
    
    // 辅助函数：找到s所在的线段索引和插值比例
    void findSegment(double s, int& idx, double& t) const {
        if (s <= 0.0) {
            idx = 0;
            t = 0.0;
            return;
        }
        if (s >= total_s_) {
            idx = static_cast<int>(ref_points_.size()) - 2;
            t = 1.0;
            return;
        }
        
        // 二分查找s所在的线段
        auto it = std::lower_bound(s_vec_.begin(), s_vec_.end(), s);
        idx = static_cast<int>(std::distance(s_vec_.begin(), it)) - 1;
        if (idx < 0) idx = 0;
        if (idx >= static_cast<int>(s_vec_.size()) - 1) {
            idx = static_cast<int>(s_vec_.size()) - 2;
        }
        
        t = (s - s_vec_[idx]) / (s_vec_[idx+1] - s_vec_[idx]);
        t = std::max(0.0, std::min(1.0, t));
    }

    // 计算点到线段的投影
    void projectToSegment(const Point2D& p, int seg_idx, 
                         double& t, Point2D& proj, double& dist_sq) const {
        const Point2D& p0 = ref_points_[seg_idx];
        const Point2D& p1 = ref_points_[seg_idx+1];
        
        double dx_seg = p1.x - p0.x;
        double dy_seg = p1.y - p0.y;
        double seg_length_sq = dx_seg*dx_seg + dy_seg*dy_seg;
        
        // 处理零长度线段
        if (seg_length_sq < 1e-12) {
            t = 0.0;
            proj = p0;
            double dx = p.x - p0.x;
            double dy = p.y - p0.y;
            dist_sq = dx*dx + dy*dy;
            return;
        }
        
        // 计算投影比例
        double dx = p.x - p0.x;
        double dy = p.y - p0.y;
        t = (dx*dx_seg + dy*dy_seg) / seg_length_sq;
        
        // 约束到线段上
        t = std::max(0.0, std::min(1.0, t));
        proj.x = p0.x + t * dx_seg;
        proj.y = p0.y + t * dy_seg;
        
        dx = p.x - proj.x;
        dy = p.y - proj.y;
        dist_sq = dx*dx + dy*dy;
    }

public:
    // 构造函数：输入一系列笛卡尔坐标点构建参考线
    VirtualLane(const std::vector<std::array<double, 2>>& points) {
        if (points.size() < 2) {
            throw std::invalid_argument("At least two points are required.");
        }
        
        // 复制点并构建参考线
        ref_points_.reserve(points.size());
        for (const auto& p : points) {
            ref_points_.emplace_back(p[0], p[1]);
        }
        
        int n = ref_points_.size();
        s_vec_.resize(n);
        theta_vec_.resize(n);
        
        // 计算累计弧长和切线角度
        s_vec_[0] = 0.0;
        for (int i = 1; i < n; ++i) {
            double dx = ref_points_[i].x - ref_points_[i-1].x;
            double dy = ref_points_[i].y - ref_points_[i-1].y;
            double ds = std::hypot(dx, dy);
            s_vec_[i] = s_vec_[i-1] + ds;
            theta_vec_[i-1] = std::atan2(dy, dx);  // 线段[i-1, i]的角度
        }
        theta_vec_[n-1] = theta_vec_[n-2];  // 终点角度与前一线段相同
        total_s_ = s_vec_.back();
        
        // 构建KD树加速最近点搜索
        kd_tree_ = std::make_unique<KDTree>(ref_points_);
    }

    // 接口1: 笛卡尔坐标转Frenet坐标 (返回[s, d])
    std::array<double, 2> cartesianToFrenet(const std::array<double, 2>& cart_point) const {
        Point2D query(cart_point[0], cart_point[1]);
        
        // 使用KD树找到最近参考点
        int nearest_idx = kd_tree_->findNearest(query);
        if (nearest_idx == -1) {
            throw std::runtime_error("KD tree search failed");
        }
        
        // 确定搜索窗口 (前后各2个线段)
        int start_idx = std::max(0, nearest_idx - 2);
        int end_idx = std::min(static_cast<int>(ref_points_.size()) - 1, nearest_idx + 3);
        
        double min_dist_sq = std::numeric_limits<double>::max();
        int best_idx = -1;
        double best_t = 0.0;
        Point2D best_proj;
        
        // 在搜索窗口内查找最近投影点
        for (int i = start_idx; i < end_idx; ++i) {
            double t;
            Point2D proj;
            double dist_sq;
            
            projectToSegment(query, i, t, proj, dist_sq);
            
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                best_idx = i;
                best_t = t;
                best_proj = proj;
            }
        }
        
        // 计算投影点的s坐标
        double s_proj = s_vec_[best_idx] + best_t * (s_vec_[best_idx+1] - s_vec_[best_idx]);
        
        // 计算d坐标（带符号距离）
        double dist = std::sqrt(min_dist_sq);
        double tx = std::cos(theta_vec_[best_idx]);
        double ty = std::sin(theta_vec_[best_idx]);
        double cross = (query.x - best_proj.x) * ty - (query.y - best_proj.y) * tx;
        double d = (cross >= 0) ? -dist : dist;
        
        return {s_proj, d};
    }

    // 接口2: Frenet坐标转笛卡尔坐标
    std::array<double, 2> frenetToCartesian(double s, double d) const {
        int idx;
        double t;
        findSegment(s, idx, t);
        
        // 插值参考点
        const Point2D& p0 = ref_points_[idx];
        const Point2D& p1 = ref_points_[idx+1];
        double x_ref = p0.x + t * (p1.x - p0.x);
        double y_ref = p0.y + t * (p1.y - p0.y);
        
        // 计算法向量（逆时针旋转90度）
        double nx = -std::sin(theta_vec_[idx]);
        double ny = std::cos(theta_vec_[idx]);
        
        // 计算笛卡尔坐标
        double x = x_ref + d * nx;
        double y = y_ref + d * ny;
        
        return {x, y};
    }

    // 接口3: 获取s位置的路点角度（弧度）
    double getAngleAtS(double s) const {
        int idx;
        double t;
        findSegment(s, idx, t);
        return theta_vec_[idx];  // 线段上角度恒定
    }

    // 获取参考线总长度
    double getTotalS() const {
        return total_s_;
    }
};
