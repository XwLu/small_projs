int main() {
    // 创建参考线
    std::vector<std::pair<double, double>> ref_line;
    for (int i = 0; i < 100; ++i) {
        ref_line.push_back({i * 0.5, std::sin(i*0.1)}); // 正弦曲线更真实
    }
    
    // 创建Pure Pursuit控制器
    Purepursuit pp_controller(2.5, 0.1, 3.0, 15.0);
    pp_controller.setReferenceLine(ref_line);
    
    // 运行仿真
    auto trajectory = pp_controller.simulate(
        0.0, 0.0, 0.0,    // 初始位置(x,y), 初始朝向(弧度)
        5.0,               // 初始速度(m/s)
        0.5,               // 加速度(m/s²)
        15.0               // 最大速度(m/s)
    );
    
    // 输出轨迹点
    for (const auto& point : trajectory) {
        auto [x, y, yaw, v] = point;
        std::cout << "Position: (" << x << ", " << y << ") "
                  << "Yaw: " << yaw << " Velocity: " << v << std::endl;
    }
    
    return 0;
}
