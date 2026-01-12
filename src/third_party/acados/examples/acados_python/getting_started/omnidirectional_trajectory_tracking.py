#
# Copyright (c) The acados authors.
#
# This file is part of acados.
#
# The 2-Clause BSD License
#

"""
全向轮移动机器人 MPC 轨迹跟踪控制器 (升级版)

本脚本实现了基于微分平坦特性的多项式轨迹跟踪 MPC 控制器。
相比基础版本，主要改进：
1. 支持8字形(Lissajous)和圆形轨迹生成
2. 利用微分平坦特性计算参考控制量（前馈）
3. 动态更新MPC预测视界内的参考轨迹
4. 可视化对比理论轨迹与实际跟踪轨迹

微分平坦特性说明：
==================
全向轮机器人是微分平坦系统，平坦输出为 (x, y, θ)。
给定轨迹 x(t), y(t), θ(t)，可解析计算控制量：
    ẋ = v_x*cos(θ) - v_y*sin(θ)
    ẏ = v_x*sin(θ) + v_y*cos(θ)
    
逆解得：
    v_x =  ẋ*cos(θ) + ẏ*sin(θ)
    v_y = -ẋ*sin(θ) + ẏ*cos(θ)
    ω   = θ̇

作者: Jinbo Liu
日期: 2025.12.20
"""

import numpy as np
import matplotlib.pyplot as plt
import casadi as ca
from acados_template import AcadosOcp, AcadosOcpSolver
from typing import Tuple, Literal

# 导入全向轮模型
from omnidirectional_model import export_omnidirectional_dynamic_model


# =============================================================================
# 轨迹生成器类
# =============================================================================
class TrajectoryGenerator:
    """
    基于微分平坦特性的轨迹生成器
    
    支持的轨迹类型：
    - 'figure8': 8字形轨迹 (Lissajous曲线)
    - 'circle': 圆形轨迹
    
    核心功能：
    - 生成位置轨迹 (x, y, θ)
    - 利用微分平坦特性解析计算参考控制量 (v_x, v_y, ω)
    """
    
    def __init__(self, 
                 trajectory_type: Literal['figure8', 'circle'] = 'figure8',
                 scale: float = 1.0,
                 period: float = 10.0,
                 center: Tuple[float, float] = (0.0, 0.0)):
        """
        初始化轨迹生成器
        
        Args:
            trajectory_type: 轨迹类型 ('figure8' 或 'circle')
            scale: 轨迹缩放因子 [m]
            period: 完成一个周期的时间 [s]
            center: 轨迹中心点 (x_c, y_c) [m]
        """
        self.trajectory_type = trajectory_type
        self.scale = scale
        self.period = period
        self.center = center
        self.omega_traj = 2 * np.pi / period  # 轨迹角频率 [rad/s]
        
        print(f"[轨迹生成器] 类型: {trajectory_type}, 缩放: {scale}m, 周期: {period}s")
    
    def get_position(self, t: float) -> Tuple[float, float, float]:
        """
        获取 t 时刻的位置和航向
        
        Args:
            t: 时间 [s]
            
        Returns:
            (x, y, theta): 位置和航向角
        """
        omega = self.omega_traj
        
        if self.trajectory_type == 'figure8':
            # 8字形轨迹 (Lissajous曲线): x = A*sin(ωt), y = A*sin(2ωt)/2
            x = self.center[0] + self.scale * np.sin(omega * t)
            y = self.center[1] + self.scale * np.sin(2 * omega * t) / 2
            
            # 计算速度方向作为航向角
            dx = self.scale * omega * np.cos(omega * t)
            dy = self.scale * omega * np.cos(2 * omega * t)
            theta = np.arctan2(dy, dx)
            
        elif self.trajectory_type == 'circle':
            # 圆形轨迹: x = R*cos(ωt), y = R*sin(ωt)
            x = self.center[0] + self.scale * np.cos(omega * t)
            y = self.center[1] + self.scale * np.sin(omega * t)
            
            # 航向角为速度方向（切线方向）
            theta = omega * t + np.pi / 2  # 切线方向
            
        else:
            raise ValueError(f"未知轨迹类型: {self.trajectory_type}")
        
        return x, y, theta
    
    def get_velocity(self, t: float) -> Tuple[float, float, float]:
        """
        获取 t 时刻的世界坐标系速度和角速度
        
        Args:
            t: 时间 [s]
            
        Returns:
            (x_dot, y_dot, theta_dot): 世界坐标系下的速度
        """
        omega = self.omega_traj
        
        if self.trajectory_type == 'figure8':
            # 8字形轨迹的速度
            x_dot = self.scale * omega * np.cos(omega * t)
            y_dot = self.scale * omega * np.cos(2 * omega * t)
            
            # 计算航向角变化率 (数值微分或解析)
            # θ = atan2(ẏ, ẋ) => θ̇ = (ẋÿ - ẏẍ) / (ẋ² + ẏ²)
            x_ddot = -self.scale * omega**2 * np.sin(omega * t)
            y_ddot = -2 * self.scale * omega**2 * np.sin(2 * omega * t)
            
            denom = x_dot**2 + y_dot**2
            if denom > 1e-6:
                theta_dot = (x_dot * y_ddot - y_dot * x_ddot) / denom
            else:
                theta_dot = 0.0
                
        elif self.trajectory_type == 'circle':
            # 圆形轨迹的速度
            x_dot = -self.scale * omega * np.sin(omega * t)
            y_dot = self.scale * omega * np.cos(omega * t)
            theta_dot = omega  # 匀速旋转
            
        else:
            raise ValueError(f"未知轨迹类型: {self.trajectory_type}")
        
        return x_dot, y_dot, theta_dot
    
    def get_body_velocity(self, t: float) -> Tuple[float, float, float]:
        """
        利用微分平坦特性，计算 t 时刻的本体坐标系控制量
        
        这是核心函数！根据全向轮的运动学逆解：
            v_x =  ẋ*cos(θ) + ẏ*sin(θ)
            v_y = -ẋ*sin(θ) + ẏ*cos(θ)
            ω   = θ̇
        
        Args:
            t: 时间 [s]
            
        Returns:
            (v_x, v_y, omega): 本体坐标系下的控制量
        """
        # 获取位置和航向
        _, _, theta = self.get_position(t)
        
        # 获取世界坐标系速度
        x_dot, y_dot, theta_dot = self.get_velocity(t)
        
        # 微分平坦逆解：世界速度 -> 本体速度
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        
        v_x = x_dot * cos_theta + y_dot * sin_theta
        v_y = -x_dot * sin_theta + y_dot * cos_theta
        omega = theta_dot
        
        return v_x, v_y, omega
    
    def get_ref(self, t: float) -> np.ndarray:
        """
        获取 t 时刻的完整参考向量（用于MPC）
        
        Args:
            t: 时间 [s]
            
        Returns:
            y_ref: [x, y, theta, v_x, v_y, omega] 参考向量
        """
        # 位置参考
        x, y, theta = self.get_position(t)
        
        # 控制量参考（前馈）
        v_x, v_y, omega = self.get_body_velocity(t)
        
        return np.array([x, y, theta, v_x, v_y, omega])
    
    def get_ref_state(self, t: float) -> np.ndarray:
        """
        获取 t 时刻的状态参考（仅位置，用于终端代价）
        
        Args:
            t: 时间 [s]
            
        Returns:
            y_ref_e: [x, y, theta] 状态参考向量
        """
        x, y, theta = self.get_position(t)
        return np.array([x, y, theta])
    
    def generate_full_trajectory(self, t_start: float, t_end: float, 
                                  dt: float = 0.01) -> Tuple[np.ndarray, np.ndarray]:
        """
        生成完整的参考轨迹（用于可视化）
        
        Args:
            t_start: 起始时间 [s]
            t_end: 结束时间 [s]
            dt: 时间步长 [s]
            
        Returns:
            (t_array, ref_array): 时间数组和参考轨迹数组
        """
        t_array = np.arange(t_start, t_end, dt)
        ref_array = np.array([self.get_ref(t) for t in t_array])
        return t_array, ref_array


# =============================================================================
# OCP 设置函数
# =============================================================================
def setup_ocp() -> AcadosOcp:
    """
    设置最优控制问题 (Optimal Control Problem, OCP)
    
    与基础版本相同，但权重矩阵经过调优以适应轨迹跟踪任务。
    """
    ocp = AcadosOcp()

    # 加载全向轮机器人模型
    model = export_omnidirectional_model()
    ocp.model = model

    nx = model.x.rows()  # 状态维度: 3
    nu = model.u.rows()  # 控制维度: 3

    print(f"\n[OCP 设置] 状态维度 nx = {nx}, 控制维度 nu = {nu}")

    # =========================================================================
    # 预测时域参数
    # =========================================================================
    N = 80        # 预测步数
    Tf = 2.0      # 预测总时间 [s]
    dt = Tf / N

    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = Tf

    print(f"[OCP 设置] 预测步数 N = {N}, 预测时域 Tf = {Tf}s, 步长 dt = {dt}s")

    # =========================================================================
    # 代价函数 (针对轨迹跟踪优化)
    # =========================================================================
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.model.cost_y_expr = ca.vertcat(model.x, model.u)

    # 状态权重：位置跟踪精度要求高
    Q = np.diag([
        200.0,   # x 位置误差权重
        200.0,   # y 位置误差权重
        50.0     # theta 航向误差权重
    ])

    # 控制权重：惩罚与参考控制量的偏差（而非绝对值）
    # 由于有前馈，这里的权重可以适当降低
    R = np.diag([
        10.0,    # v_x 偏差权重
        10.0,    # v_y 偏差权重
        5.0      # omega 偏差权重
    ])

    W = np.block([
        [Q, np.zeros((nx, nu))],
        [np.zeros((nu, nx)), R]
    ])
    ocp.cost.W = W
    ocp.cost.yref = np.zeros(nx + nu)

    print(f"[OCP 设置] 路径代价权重 W 维度: {W.shape}")

    # 终端代价
    ocp.cost.cost_type_e = 'NONLINEAR_LS'
    ocp.model.cost_y_expr_e = model.x
    Q_e = np.diag([300.0, 300.0, 80.0])
    ocp.cost.W_e = Q_e
    ocp.cost.yref_e = np.zeros(nx)

    print(f"[OCP 设置] 终端代价权重 W_e 维度: {Q_e.shape}")

    # =========================================================================
    # 约束条件
    # =========================================================================
    v_max = 2.0       # 最大线速度 [m/s]
    omega_max = 0.7   # 最大角速度 [rad/s]

    ocp.constraints.lbu = np.array([-v_max, -v_max, -omega_max])
    ocp.constraints.ubu = np.array([v_max, v_max, omega_max])
    ocp.constraints.idxbu = np.array([0, 1, 2])

    ocp.constraints.x0 = np.array([0.0, 0.0, 0.0])

    print(f"[OCP 设置] 控制约束: v_x, v_y ∈ [-{v_max}, {v_max}], omega ∈ [-{omega_max}, {omega_max}]")

    # =========================================================================
    # 求解器选项
    # =========================================================================
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.nlp_solver_max_iter = 1
    ocp.solver_options.print_level = 0

    print(f"[OCP 设置] QP求解器: {ocp.solver_options.qp_solver}")
    print(f"[OCP 设置] NLP求解器: {ocp.solver_options.nlp_solver_type}")

    return ocp


# =============================================================================
# 闭环仿真函数 (升级版)
# =============================================================================
def run_trajectory_tracking_simulation(
    ocp_solver: AcadosOcpSolver,
    traj_gen: TrajectoryGenerator,
    x0: np.ndarray,
    T_sim: float,
    dt: float
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    运行基于轨迹跟踪的闭环 MPC 仿真
    
    核心改进：
    1. 在每个仿真步，动态更新MPC预测视界内的参考轨迹
    2. 参考轨迹包含位置和速度前馈信息
    
    Args:
        ocp_solver: acados OCP 求解器
        traj_gen: 轨迹生成器实例
        x0: 初始状态 [x, y, theta]
        T_sim: 仿真总时间 [s]
        dt: 仿真时间步长 [s]
        
    Returns:
        (x_traj, u_traj, t_traj, ref_traj, errors): 
        实际轨迹、控制轨迹、时间向量、参考轨迹、位置误差
    """
    # 获取维度和参数
    nx = ocp_solver.acados_ocp.dims.nx
    nu = ocp_solver.acados_ocp.dims.nu
    N = ocp_solver.acados_ocp.solver_options.N_horizon
    dt_mpc = ocp_solver.acados_ocp.solver_options.tf / N
    
    N_sim = int(T_sim / dt)
    
    # 初始化存储数组
    x_traj = np.zeros((N_sim + 1, nx))      # 实际状态轨迹
    u_traj = np.zeros((N_sim, nu))           # 实际控制轨迹
    ref_traj = np.zeros((N_sim + 1, nx + nu)) # 参考轨迹 (用于绘图)
    errors = np.zeros(N_sim + 1)             # 位置跟踪误差
    solve_times = np.zeros(N_sim)
    
    # 设置初始状态
    x_current = x0.copy()
    x_traj[0, :] = x_current
    
    # 记录初始参考和误差
    ref_traj[0, :] = traj_gen.get_ref(0.0)
    errors[0] = np.linalg.norm(x_current[:2] - ref_traj[0, :2])
    
    print("\n" + "=" * 70)
    print("开始轨迹跟踪 MPC 仿真")
    print("=" * 70)
    print(f"初始状态: x0 = [{x0[0]:.2f}, {x0[1]:.2f}, {x0[2]:.2f}]")
    print(f"轨迹类型: {traj_gen.trajectory_type}, 周期: {traj_gen.period}s")
    print(f"仿真时间: {T_sim}s, 步数: {N_sim}, 步长: {dt}s")
    print("-" * 70)
    
    # 主仿真循环
    for i in range(N_sim):
        t_current = i * dt
        
        # -----------------------------------------------------------------
        # 步骤 1: 设置初始状态约束
        # -----------------------------------------------------------------
        ocp_solver.set(0, "lbx", x_current)
        ocp_solver.set(0, "ubx", x_current)
        
        # -----------------------------------------------------------------
        # 步骤 2: 动态更新预测视界内的参考轨迹 (核心改进!)
        # -----------------------------------------------------------------
        for k in range(N):
            # 计算预测时刻
            t_pred = t_current + k * dt_mpc
            
            # 获取该时刻的完整参考 (位置 + 速度前馈)
            y_ref_k = traj_gen.get_ref(t_pred)
            
            # 设置到求解器
            ocp_solver.set(k, "yref", y_ref_k)
        
        # 设置终端参考 (仅状态)
        t_terminal = t_current + N * dt_mpc
        y_ref_e = traj_gen.get_ref_state(t_terminal)
        ocp_solver.set(N, "yref", y_ref_e)
        
        # -----------------------------------------------------------------
        # 步骤 3: 求解 OCP
        # -----------------------------------------------------------------
        status = ocp_solver.solve()
        solve_times[i] = ocp_solver.get_stats("time_tot")
        
        if status != 0:
            print(f"[警告] 第 {i} 步求解失败, 状态码: {status}")
        
        # -----------------------------------------------------------------
        # 步骤 4: 获取并应用最优控制
        # -----------------------------------------------------------------
        u_optimal = ocp_solver.get(0, "u")
        u_traj[i, :] = u_optimal
        
        # -----------------------------------------------------------------
        # 步骤 5: 系统状态更新 (欧拉积分)
        # -----------------------------------------------------------------
        theta = x_current[2]
        v_x, v_y, omega = u_optimal
        
        x_dot = v_x * np.cos(theta) - v_y * np.sin(theta)
        y_dot = v_x * np.sin(theta) + v_y * np.cos(theta)
        theta_dot = omega
        
        x_current = x_current + dt * np.array([x_dot, y_dot, theta_dot])
        
        # 保存数据
        x_traj[i + 1, :] = x_current
        ref_traj[i + 1, :] = traj_gen.get_ref((i + 1) * dt)
        errors[i + 1] = np.linalg.norm(x_current[:2] - ref_traj[i + 1, :2])
        
        # 打印进度
        if (i + 1) % 50 == 0 or i == 0:
            print(f"Step {i+1:4d}/{N_sim}: t={t_current:.2f}s, "
                  f"pos=[{x_current[0]:6.3f}, {x_current[1]:6.3f}], "
                  f"error={errors[i+1]*1000:.2f}mm, "
                  f"solve={solve_times[i]*1000:.2f}ms")
    
    # 统计信息
    avg_error = np.mean(errors) * 1000  # mm
    max_error = np.max(errors) * 1000
    avg_solve_time = np.mean(solve_times) * 1000
    
    print("-" * 70)
    print(f"仿真完成!")
    print(f"平均位置误差: {avg_error:.2f}mm, 最大位置误差: {max_error:.2f}mm")
    print(f"平均求解时间: {avg_solve_time:.3f}ms")
    print("=" * 70)
    
    t_traj = np.arange(N_sim + 1) * dt
    
    return x_traj, u_traj, t_traj, ref_traj, errors


# =============================================================================
# 可视化函数 (升级版)
# =============================================================================
def plot_tracking_results(
    x_traj: np.ndarray,
    u_traj: np.ndarray,
    t_traj: np.ndarray,
    ref_traj: np.ndarray,
    errors: np.ndarray,
    traj_gen: TrajectoryGenerator,
    save_path: str = None
):
    """
    绘制轨迹跟踪仿真结果 (升级版)
    
    新增功能：
    1. 同时显示理论参考轨迹和实际跟踪轨迹
    2. 增加位置误差随时间变化的子图
    
    Args:
        x_traj: 实际状态轨迹
        u_traj: 实际控制轨迹
        t_traj: 时间向量
        ref_traj: 参考轨迹
        errors: 位置误差
        traj_gen: 轨迹生成器 (用于生成完整参考曲线)
        save_path: 图片保存路径
    """
    # 禁用 LaTeX，设置字体
    plt.rcParams['text.usetex'] = False
    
    import matplotlib.font_manager as fm
    chinese_fonts = [
        'WenQuanYi Micro Hei', 'WenQuanYi Zen Hei', 'Noto Sans CJK SC',
        'Source Han Sans SC', 'SimHei', 'Microsoft YaHei', 'DejaVu Sans'
    ]
    available_fonts = [f.name for f in fm.fontManager.ttflist]
    selected_font = next((f for f in chinese_fonts if f in available_fonts), 'DejaVu Sans')
    
    use_chinese = selected_font != 'DejaVu Sans'
    if use_chinese:
        plt.rcParams['font.family'] = selected_font
        plt.rcParams['axes.unicode_minus'] = False
    
    # 生成完整的理论轨迹用于绘图
    t_ref_full, ref_full = traj_gen.generate_full_trajectory(0, t_traj[-1], 0.01)
    
    # 创建 2x4 子图布局
    fig, axes = plt.subplots(2, 4, figsize=(20, 10))
    
    title = '全向轮机器人 MPC 轨迹跟踪结果' if use_chinese else 'Omnidirectional Robot MPC Trajectory Tracking'
    fig.suptitle(title, fontsize=16)
    
    # 标签字典
    L = {
        'actual': '实际轨迹' if use_chinese else 'Actual',
        'ref': '参考轨迹' if use_chinese else 'Reference',
        'start': '起点' if use_chinese else 'Start',
        'time': '时间 [s]' if use_chinese else 'Time [s]',
        'error': '位置误差 [mm]' if use_chinese else 'Position Error [mm]',
        'x_pos': 'X 位置' if use_chinese else 'X Position',
        'y_pos': 'Y 位置' if use_chinese else 'Y Position',
        'heading': '航向角' if use_chinese else 'Heading',
        'vx': 'v_x 控制' if use_chinese else 'v_x Control',
        'vy': 'v_y 控制' if use_chinese else 'v_y Control',
        'omega': 'ω 控制' if use_chinese else 'ω Control',
        '2d': '2D 轨迹对比' if use_chinese else '2D Trajectory Comparison',
        'tracking_error': '跟踪误差' if use_chinese else 'Tracking Error',
    }
    
    # ----- 子图 1: 2D 轨迹对比 (大图) -----
    ax1 = axes[0, 0]
    ax1.plot(ref_full[:, 0], ref_full[:, 1], 'r--', linewidth=1.5, alpha=0.7, label=L['ref'])
    ax1.plot(x_traj[:, 0], x_traj[:, 1], 'b-', linewidth=2, label=L['actual'])
    ax1.plot(x_traj[0, 0], x_traj[0, 1], 'go', markersize=10, label=L['start'])
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_title(L['2d'])
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # ----- 子图 2: 位置误差 -----
    ax2 = axes[0, 1]
    ax2.plot(t_traj, errors * 1000, 'b-', linewidth=2)  # 转换为 mm
    ax2.axhline(y=np.mean(errors) * 1000, color='r', linestyle='--', 
                label=f'Mean: {np.mean(errors)*1000:.1f}mm')
    ax2.set_xlabel(L['time'])
    ax2.set_ylabel(L['error'])
    ax2.set_title(L['tracking_error'])
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # ----- 子图 3: X 位置对比 -----
    ax3 = axes[0, 2]
    ax3.plot(t_traj, ref_traj[:, 0], 'r--', linewidth=1.5, alpha=0.7, label=L['ref'])
    ax3.plot(t_traj, x_traj[:, 0], 'b-', linewidth=2, label=L['actual'])
    ax3.set_xlabel(L['time'])
    ax3.set_ylabel('X [m]')
    ax3.set_title(L['x_pos'])
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # ----- 子图 4: Y 位置对比 -----
    ax4 = axes[0, 3]
    ax4.plot(t_traj, ref_traj[:, 1], 'r--', linewidth=1.5, alpha=0.7, label=L['ref'])
    ax4.plot(t_traj, x_traj[:, 1], 'b-', linewidth=2, label=L['actual'])
    ax4.set_xlabel(L['time'])
    ax4.set_ylabel('Y [m]')
    ax4.set_title(L['y_pos'])
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    # ----- 子图 5: 航向角对比 -----
    ax5 = axes[1, 0]
    ax5.plot(t_traj, np.rad2deg(ref_traj[:, 2]), 'r--', linewidth=1.5, alpha=0.7, label=L['ref'])
    ax5.plot(t_traj, np.rad2deg(x_traj[:, 2]), 'b-', linewidth=2, label=L['actual'])
    ax5.set_xlabel(L['time'])
    ax5.set_ylabel('θ [deg]')
    ax5.set_title(L['heading'])
    ax5.legend()
    ax5.grid(True, alpha=0.3)
    
    # ----- 子图 6: v_x 控制量 -----
    t_u = t_traj[:-1]
    ax6 = axes[1, 1]
    ax6.plot(t_u, ref_traj[:-1, 3], 'r--', linewidth=1.5, alpha=0.7, label=L['ref'])
    ax6.plot(t_u, u_traj[:, 0], 'b-', linewidth=2, label=L['actual'])
    ax6.set_xlabel(L['time'])
    ax6.set_ylabel('v_x [m/s]')
    ax6.set_title(L['vx'])
    ax6.legend()
    ax6.grid(True, alpha=0.3)
    
    # ----- 子图 7: v_y 控制量 -----
    ax7 = axes[1, 2]
    ax7.plot(t_u, ref_traj[:-1, 4], 'r--', linewidth=1.5, alpha=0.7, label=L['ref'])
    ax7.plot(t_u, u_traj[:, 1], 'b-', linewidth=2, label=L['actual'])
    ax7.set_xlabel(L['time'])
    ax7.set_ylabel('v_y [m/s]')
    ax7.set_title(L['vy'])
    ax7.legend()
    ax7.grid(True, alpha=0.3)
    
    # ----- 子图 8: ω 控制量 -----
    ax8 = axes[1, 3]
    ax8.plot(t_u, ref_traj[:-1, 5], 'r--', linewidth=1.5, alpha=0.7, label=L['ref'])
    ax8.plot(t_u, u_traj[:, 2], 'b-', linewidth=2, label=L['actual'])
    ax8.set_xlabel(L['time'])
    ax8.set_ylabel('ω [rad/s]')
    ax8.set_title(L['omega'])
    ax8.legend()
    ax8.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"\n[INFO] 图片已保存至: {save_path}")
    
    plt.show()


# =============================================================================
# 主函数
# =============================================================================
def main():
    """
    主函数：运行全向轮机器人轨迹跟踪 MPC 仿真
    """
    print("\n" + "=" * 70)
    print("全向轮移动机器人 MPC 轨迹跟踪控制器 (升级版)")
    print("基于微分平坦特性的多项式轨迹跟踪")
    print("=" * 70)
    
    # =========================================================================
    # 步骤 1: 创建轨迹生成器
    # =========================================================================
    print("\n[步骤 1] 创建轨迹生成器...")
    
    # 选择轨迹类型: 'figure8' (8字形) 或 'circle' (圆形)
    traj_gen = TrajectoryGenerator(
        trajectory_type='figure8',  # 可改为 'circle'
        scale=1.5,                  # 轨迹大小 [m]
        period=10.0,                # 周期 [s]
        center=(0.0, 0.0)           # 轨迹中心
    )
    
    # =========================================================================
    # 步骤 2: 设置 OCP 并创建求解器
    # =========================================================================
    print("\n[步骤 2] 配置最优控制问题 (OCP)...")
    ocp = setup_ocp()
    
    print("\n[步骤 3] 创建 acados 求解器...")
    ocp_solver = AcadosOcpSolver(ocp)
    print("[INFO] C 代码已生成至: c_generated_code/")
    
    # =========================================================================
    # 步骤 3: 设置仿真参数
    # =========================================================================
    # 初始状态：从轨迹起点开始
    x0 = traj_gen.get_ref_state(0.0)
    print(f"\n[INFO] 初始状态 (轨迹起点): [{x0[0]:.3f}, {x0[1]:.3f}, {x0[2]:.3f}]")
    
    # 仿真参数
    T_sim = 15.0    # 仿真总时间 [s] (1.5个周期)
    dt = 0.05       # 仿真步长 [s]
    
    # =========================================================================
    # 步骤 4: 运行轨迹跟踪仿真
    # =========================================================================
    print("\n[步骤 4] 运行轨迹跟踪 MPC 仿真...")
    x_traj, u_traj, t_traj, ref_traj, errors = run_trajectory_tracking_simulation(
        ocp_solver, traj_gen, x0, T_sim, dt
    )
    
    # =========================================================================
    # 步骤 5: 绘制结果
    # =========================================================================
    print("\n[步骤 5] 绘制仿真结果...")
    plot_tracking_results(
        x_traj, u_traj, t_traj, ref_traj, errors, traj_gen,
        save_path='omnidirectional_trajectory_tracking.png'
    )
    
    # 清理
    del ocp_solver
    
    print("\n[完成] 轨迹跟踪 MPC 仿真已完成!")
    print("=" * 70)


if __name__ == '__main__':
    main()
