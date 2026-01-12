#
# Copyright (c) The acados authors.
#
# This file is part of acados.
#
# The 2-Clause BSD License
#

"""
全向轮移动机器人 MPC 轨迹跟踪控制器

本脚本实现了一个基于 acados 的 MPC（模型预测控制）轨迹跟踪控制器，
用于控制全向轮（Omnidirectional/Holonomic）移动机器人从起点移动到目标点。

功能特性:
=========
1. 使用非线性最小二乘 (NONLINEAR_LS) 代价函数进行轨迹跟踪
2. 支持控制输入约束（速度限制）
3. 自动生成可移植的 C 代码
4. 闭环仿真演示
5. 结果可视化

使用方法:
=========
直接运行此脚本:
    python omnidirectional_ocp.py

生成的文件:
===========
    - c_generated_code/           : 包含可移植的 C 代码
    - omnidirectional_trajectory.png : 仿真轨迹图

作者: Jinbo Liu
日期: 2025.12.20
"""

import numpy as np
import matplotlib.pyplot as plt
import casadi as ca
from acados_template import AcadosOcp, AcadosOcpSolver

# 导入我们定义的全向轮模型
from omnidirectional_model import export_omnidirectional_model


def setup_ocp() -> AcadosOcp:
    """
    设置最优控制问题 (Optimal Control Problem, OCP)

    本函数配置 MPC 的所有参数，包括：
    - 系统模型
    - 代价函数
    - 约束条件
    - 求解器选项

    Returns:
        AcadosOcp: 配置好的 OCP 对象
    """

    # =========================================================================
    # 1. 创建 OCP 对象并设置模型
    # =========================================================================
    ocp = AcadosOcp()

    # 加载全向轮机器人模型
    model = export_omnidirectional_model()
    ocp.model = model

    # 获取状态和控制的维度
    nx = model.x.rows()   # 状态维度: 3 (x, y, theta)
    nu = model.u.rows()   # 控制维度: 3 (v_x, v_y, omega)

    print(f"\n[OCP 设置] 状态维度 nx = {nx}, 控制维度 nu = {nu}")

    # =========================================================================
    # 2. 设置预测时域参数
    # =========================================================================
    N = 20        # 预测步数 (prediction horizon steps)
    Tf = 1.0      # 预测总时间 [s]
    dt = Tf / N   # 单步时间间隔 [s]

    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = Tf

    print(f"[OCP 设置] 预测步数 N = {N}, 预测时域 Tf = {Tf}s, 步长 dt = {dt}s")

    # =========================================================================
    # 3. 设置代价函数 (Cost Function)
    # =========================================================================
    # 使用非线性最小二乘 (NONLINEAR_LS) 代价函数
    #
    # 代价函数形式:
    #   J = sum_{k=0}^{N-1} ||y_k - y_ref||^2_W + ||y_N - y_ref_e||^2_W_e
    #
    # 其中:
    #   y_k = [x, y, theta, v_x, v_y, omega]^T  (路径代价输出)
    #   y_N = [x, y, theta]^T                    (终端代价输出)

    # ----- 路径代价 (Stage Cost) -----
    ocp.cost.cost_type = 'NONLINEAR_LS'

    # 定义代价输出表达式 y = [状态; 控制]
    # 这意味着我们同时惩罚状态偏差和控制输入
    ocp.model.cost_y_expr = ca.vertcat(model.x, model.u)

    # 状态权重矩阵 Q: 惩罚位置和航向偏差
    # [x, y, theta] 的权重
    Q = np.diag([
        100.0,   # x 位置误差权重（较大，希望精确跟踪）
        100.0,   # y 位置误差权重（较大，希望精确跟踪）
        10.0     # theta 航向误差权重（中等）
    ])

    # 控制权重矩阵 R: 惩罚控制输入大小（避免过大的控制动作）
    # [v_x, v_y, omega] 的权重
    R = np.diag([
        1.0,    # v_x 权重（较小，允许较大速度）
        1.0,    # v_y 权重（较小，允许较大速度）
        0.1     # omega 权重（更小，允许灵活转向）
    ])

    # 组合权重矩阵 W = diag(Q, R)，维度为 (nx + nu) x (nx + nu)
    W = np.block([
        [Q, np.zeros((nx, nu))],
        [np.zeros((nu, nx)), R]
    ])
    ocp.cost.W = W

    # 路径参考值初始化（将在仿真循环中更新）
    # y_ref = [x_ref, y_ref, theta_ref, v_x_ref, v_y_ref, omega_ref]
    ocp.cost.yref = np.zeros(nx + nu)

    print(f"[OCP 设置] 路径代价权重 W 维度: {W.shape}")

    # ----- 终端代价 (Terminal Cost) -----
    ocp.cost.cost_type_e = 'NONLINEAR_LS'

    # 终端代价只包含状态（不包含控制）
    ocp.model.cost_y_expr_e = model.x

    # 终端权重矩阵（通常设置较大，强调到达目标）
    Q_e = np.diag([
        200.0,   # x 终端位置权重
        200.0,   # y 终端位置权重
        20.0     # theta 终端航向权重
    ])
    ocp.cost.W_e = Q_e

    # 终端参考值初始化
    ocp.cost.yref_e = np.zeros(nx)

    print(f"[OCP 设置] 终端代价权重 W_e 维度: {Q_e.shape}")

    # =========================================================================
    # 4. 设置约束条件 (Constraints)
    # =========================================================================

    # ----- 控制输入约束 (Box Constraints on u) -----
    # 定义控制输入的上下界
    #   v_x, v_y ∈ [-1.0, 1.0] m/s
    #   omega ∈ [-2.0, 2.0] rad/s

    v_max = 2.0     # 最大线速度 [m/s]
    omega_max = 0.7  # 最大角速度 [rad/s]

    # 下界 (lower bounds)
    ocp.constraints.lbu = np.array([-v_max, -v_max, -omega_max])

    # 上界 (upper bounds)
    ocp.constraints.ubu = np.array([v_max, v_max, omega_max])

    # 指定哪些控制变量受约束（这里是所有 3 个）
    ocp.constraints.idxbu = np.array([0, 1, 2])

    print(f"[OCP 设置] 控制约束: v_x, v_y ∈ [-{v_max}, {v_max}], omega ∈ [-{omega_max}, {omega_max}]")

    # ----- 初始状态约束 (Initial State Constraint) -----
    # 初始状态将在仿真循环中设置，这里先初始化为零
    x0 = np.array([0.0, 0.0, 0.0])
    ocp.constraints.x0 = x0

    print(f"[OCP 设置] 初始状态约束: x0 = {x0}")

    # =========================================================================
    # 5. 设置求解器选项 (Solver Options)
    # =========================================================================

    # QP 求解器选择
    # PARTIAL_CONDENSING_HPIPM: 高性能内点法，适合中等规模问题
    # 其他选项: FULL_CONDENSING_QPOASES, FULL_CONDENSING_HPIPM
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'

    # Hessian 近似方法
    # GAUSS_NEWTON: 适用于最小二乘代价函数，稳定且高效
    # EXACT: 精确 Hessian，计算量更大
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'

    # 积分器类型
    # ERK: 显式 Runge-Kutta (适用于非刚性系统，全向轮运动学是非刚性的)
    # IRK: 隐式 Runge-Kutta (适用于刚性系统)
    ocp.solver_options.integrator_type = 'ERK'

    # 积分器阶数 (ERK4 是经典的 4 阶 Runge-Kutta)
    ocp.solver_options.sim_method_num_stages = 4

    # NLP 求解器类型
    # SQP: 完整的序列二次规划（精确但可能较慢）
    # SQP_RTI: 实时迭代 SQP（每个时间步只做一次 QP 迭代，适合实时控制）
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'

    # 最大迭代次数（对于 SQP_RTI 通常设为 1）
    ocp.solver_options.nlp_solver_max_iter = 1

    # 打印级别 (0: 无输出, 1: 简要输出, 2: 详细输出)
    ocp.solver_options.print_level = 0

    print(f"[OCP 设置] QP求解器: {ocp.solver_options.qp_solver}")
    print(f"[OCP 设置] 积分器: {ocp.solver_options.integrator_type}")
    print(f"[OCP 设置] NLP求解器: {ocp.solver_options.nlp_solver_type}")

    return ocp


def run_closed_loop_simulation(ocp_solver: AcadosOcpSolver, x0: np.ndarray,
                                x_goal: np.ndarray, N_sim: int, dt: float):
    """
    运行闭环 MPC 仿真

    Args:
        ocp_solver: acados OCP 求解器对象
        x0: 初始状态 [x, y, theta]
        x_goal: 目标状态 [x, y, theta]
        N_sim: 仿真步数
        dt: 仿真时间步长 [s]

    Returns:
        tuple: (状态轨迹, 控制轨迹, 时间向量, 求解时间列表)
    """

    # 获取维度
    nx = ocp_solver.acados_ocp.dims.nx
    nu = ocp_solver.acados_ocp.dims.nu
    N = ocp_solver.acados_ocp.dims.N

    # 初始化存储数组
    x_traj = np.zeros((N_sim + 1, nx))   # 状态轨迹
    u_traj = np.zeros((N_sim, nu))        # 控制轨迹
    solve_times = np.zeros(N_sim)         # 求解时间

    # 设置初始状态
    x_current = x0.copy()
    x_traj[0, :] = x_current

    # 设置参考轨迹（这里使用简单的点到点控制，参考固定为目标点）
    # 路径参考: [x_ref, y_ref, theta_ref, v_x_ref, v_y_ref, omega_ref]
    y_ref = np.concatenate([x_goal, np.zeros(nu)])

    # 终端参考: [x_ref, y_ref, theta_ref]
    y_ref_e = x_goal

    print("\n" + "=" * 60)
    print("开始闭环 MPC 仿真")
    print("=" * 60)
    print(f"初始状态: x0 = [{x0[0]:.2f}, {x0[1]:.2f}, {x0[2]:.2f}]")
    print(f"目标状态: x_goal = [{x_goal[0]:.2f}, {x_goal[1]:.2f}, {x_goal[2]:.2f}]")
    print(f"仿真步数: {N_sim}, 总时间: {N_sim * dt:.2f}s")
    print("-" * 60)

    # 主仿真循环
    for i in range(N_sim):
        # ---------------------------------------------------------------------
        # 步骤 1: 设置初始状态约束
        # ---------------------------------------------------------------------
        ocp_solver.set(0, "lbx", x_current)
        ocp_solver.set(0, "ubx", x_current)

        # ---------------------------------------------------------------------
        # 步骤 2: 设置参考轨迹
        # ---------------------------------------------------------------------
        # 对于每个预测步，设置参考值
        for k in range(N):
            ocp_solver.set(k, "yref", y_ref)

        # 设置终端参考
        ocp_solver.set(N, "yref", y_ref_e)

        # ---------------------------------------------------------------------
        # 步骤 3: 求解 OCP
        # ---------------------------------------------------------------------
        status = ocp_solver.solve()

        # 记录求解时间
        solve_times[i] = ocp_solver.get_stats("time_tot")

        # 检查求解状态
        if status != 0:
            print(f"[警告] 第 {i} 步求解失败, 状态码: {status}")

        # ---------------------------------------------------------------------
        # 步骤 4: 获取最优控制输入（第一步）
        # ---------------------------------------------------------------------
        u_optimal = ocp_solver.get(0, "u")
        u_traj[i, :] = u_optimal

        # ---------------------------------------------------------------------
        # 步骤 5: 应用控制并更新状态（简单的欧拉积分用于仿真）
        # ---------------------------------------------------------------------
        # 实际应用中，这一步由真实系统/仿真器完成
        # 这里使用简化的欧拉积分来模拟系统响应

        theta = x_current[2]
        v_x, v_y, omega = u_optimal

        # 全向轮运动学方程
        x_dot = v_x * np.cos(theta) - v_y * np.sin(theta)
        y_dot = v_x * np.sin(theta) + v_y * np.cos(theta)
        theta_dot = omega

        # 欧拉积分更新状态
        x_current = x_current + dt * np.array([x_dot, y_dot, theta_dot])

        # 保存状态
        x_traj[i + 1, :] = x_current

        # 打印进度（每 20 步打印一次）
        if (i + 1) % 20 == 0 or i == 0:
            error = np.linalg.norm(x_current[:2] - x_goal[:2])
            print(f"Step {i+1:4d}: pos=[{x_current[0]:6.3f}, {x_current[1]:6.3f}], "
                  f"theta={x_current[2]:6.3f}, error={error:.4f}m, "
                  f"solve_time={solve_times[i]*1000:.2f}ms")

    # 计算统计信息
    avg_solve_time = np.mean(solve_times) * 1000  # 转换为毫秒
    max_solve_time = np.max(solve_times) * 1000
    final_error = np.linalg.norm(x_traj[-1, :2] - x_goal[:2])

    print("-" * 60)
    print(f"仿真完成!")
    print(f"最终位置: [{x_traj[-1, 0]:.4f}, {x_traj[-1, 1]:.4f}], 航向: {x_traj[-1, 2]:.4f}")
    print(f"最终位置误差: {final_error:.4f}m")
    print(f"平均求解时间: {avg_solve_time:.3f}ms, 最大求解时间: {max_solve_time:.3f}ms")
    print("=" * 60)

    # 生成时间向量
    t_traj = np.arange(N_sim + 1) * dt

    return x_traj, u_traj, t_traj, solve_times


def plot_results(x_traj: np.ndarray, u_traj: np.ndarray, t_traj: np.ndarray,
                 x_goal: np.ndarray, save_path: str = None):
    """
    绘制仿真结果

    Args:
        x_traj: 状态轨迹 (N_sim+1, nx)
        u_traj: 控制轨迹 (N_sim, nu)
        t_traj: 时间向量 (N_sim+1,)
        x_goal: 目标状态
        save_path: 图片保存路径（可选）
    """

    # 禁用 LaTeX 渲染，避免依赖问题
    plt.rcParams['text.usetex'] = False

    # 设置中文字体支持
    # 尝试使用系统中可用的中文字体
    import matplotlib.font_manager as fm
    
    # Linux 系统常见中文字体列表（按优先级排序）
    chinese_fonts = [
        'WenQuanYi Micro Hei',      # 文泉驿微米黑
        'WenQuanYi Zen Hei',        # 文泉驿正黑
        'Noto Sans CJK SC',         # Google Noto 字体
        'Noto Sans CJK TC',
        'Source Han Sans SC',       # 思源黑体
        'SimHei',                   # 黑体
        'Microsoft YaHei',          # 微软雅黑
        'DejaVu Sans',              # 回退到默认字体
    ]
    
    # 查找可用的中文字体
    available_fonts = [f.name for f in fm.fontManager.ttflist]
    selected_font = None
    for font in chinese_fonts:
        if font in available_fonts:
            selected_font = font
            break
    
    if selected_font and selected_font != 'DejaVu Sans':
        plt.rcParams['font.family'] = selected_font
        plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题
    else:
        # 如果没有找到中文字体，使用英文标签
        print("[警告] 未找到中文字体，将使用英文标签")

    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    
    # 根据是否有中文字体选择标题
    if selected_font and selected_font != 'DejaVu Sans':
        fig.suptitle('全向轮机器人 MPC 轨迹跟踪结果', fontsize=14)
    else:
        fig.suptitle('Omnidirectional Robot MPC Trajectory Tracking', fontsize=14)

    # 定义中英文标签
    use_chinese = selected_font and selected_font != 'DejaVu Sans'
    labels = {
        'trajectory': '实际轨迹' if use_chinese else 'Trajectory',
        'start': '起点' if use_chinese else 'Start',
        'goal': '目标点' if use_chinese else 'Goal',
        'target': '目标' if use_chinese else 'Target',
        'time': '时间 [s]' if use_chinese else 'Time [s]',
        '2d_traj': '2D 轨迹' if use_chinese else '2D Trajectory',
        'x_pos': 'X 位置' if use_chinese else 'X Position',
        'y_pos': 'Y 位置' if use_chinese else 'Y Position',
        'heading': '航向角' if use_chinese else 'Heading',
        'heading_unit': '航向角 [deg]' if use_chinese else 'Heading [deg]',
        'lin_vel': '线速度控制' if use_chinese else 'Linear Velocity Control',
        'velocity': '速度 [m/s]' if use_chinese else 'Velocity [m/s]',
        'ang_vel': '角速度控制' if use_chinese else 'Angular Velocity Control',
        'ang_vel_unit': '角速度 [rad/s]' if use_chinese else 'Angular Velocity [rad/s]',
        'limit': '限制' if use_chinese else 'Limit',
    }

    # ----- 子图 1: 2D 轨迹 -----
    ax1 = axes[0, 0]
    ax1.plot(x_traj[:, 0], x_traj[:, 1], 'b-', linewidth=2, label=labels['trajectory'])
    ax1.plot(x_traj[0, 0], x_traj[0, 1], 'go', markersize=10, label=labels['start'])
    ax1.plot(x_goal[0], x_goal[1], 'r*', markersize=15, label=labels['goal'])
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_title(labels['2d_traj'])
    ax1.legend()
    ax1.grid(True)
    ax1.axis('equal')

    # ----- 子图 2: X 位置 vs 时间 -----
    ax2 = axes[0, 1]
    ax2.plot(t_traj, x_traj[:, 0], 'b-', linewidth=2)
    ax2.axhline(y=x_goal[0], color='r', linestyle='--', label=labels['target'])
    ax2.set_xlabel(labels['time'])
    ax2.set_ylabel('X [m]')
    ax2.set_title(labels['x_pos'])
    ax2.legend()
    ax2.grid(True)

    # ----- 子图 3: Y 位置 vs 时间 -----
    ax3 = axes[0, 2]
    ax3.plot(t_traj, x_traj[:, 1], 'b-', linewidth=2)
    ax3.axhline(y=x_goal[1], color='r', linestyle='--', label=labels['target'])
    ax3.set_xlabel(labels['time'])
    ax3.set_ylabel('Y [m]')
    ax3.set_title(labels['y_pos'])
    ax3.legend()
    ax3.grid(True)

    # ----- 子图 4: 航向角 vs 时间 -----
    ax4 = axes[1, 0]
    ax4.plot(t_traj, np.rad2deg(x_traj[:, 2]), 'b-', linewidth=2)
    ax4.axhline(y=np.rad2deg(x_goal[2]), color='r', linestyle='--', label=labels['target'])
    ax4.set_xlabel(labels['time'])
    ax4.set_ylabel(labels['heading_unit'])
    ax4.set_title(labels['heading'])
    ax4.legend()
    ax4.grid(True)

    # ----- 子图 5: 线速度控制输入 -----
    t_u = t_traj[:-1]  # 控制时间
    ax5 = axes[1, 1]
    ax5.plot(t_u, u_traj[:, 0], 'b-', linewidth=2, label='v_x')
    ax5.plot(t_u, u_traj[:, 1], 'g-', linewidth=2, label='v_y')
    ax5.axhline(y=1.0, color='r', linestyle='--', alpha=0.5, label=labels['limit'])
    ax5.axhline(y=-1.0, color='r', linestyle='--', alpha=0.5)
    ax5.set_xlabel(labels['time'])
    ax5.set_ylabel(labels['velocity'])
    ax5.set_title(labels['lin_vel'])
    ax5.legend()
    ax5.grid(True)

    # ----- 子图 6: 角速度控制输入 -----
    ax6 = axes[1, 2]
    ax6.plot(t_u, u_traj[:, 2], 'b-', linewidth=2, label='omega')
    ax6.axhline(y=2.0, color='r', linestyle='--', alpha=0.5, label=labels['limit'])
    ax6.axhline(y=-2.0, color='r', linestyle='--', alpha=0.5)
    ax6.set_xlabel(labels['time'])
    ax6.set_ylabel(labels['ang_vel_unit'])
    ax6.set_title(labels['ang_vel'])
    ax6.legend()
    ax6.grid(True)

    plt.tight_layout()

    # 保存图片
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"\n[INFO] 图片已保存至: {save_path}")

    plt.show()


def main():
    """
    主函数：设置并运行全向轮机器人 MPC 仿真
    """

    print("\n" + "=" * 60)
    print("全向轮移动机器人 MPC 轨迹跟踪控制器")
    print("基于 acados Python 接口")
    print("=" * 60)

    # =========================================================================
    # 步骤 1: 设置 OCP 并创建求解器
    # =========================================================================
    print("\n[步骤 1] 配置最优控制问题 (OCP)...")
    ocp = setup_ocp()

    print("\n[步骤 2] 创建 acados 求解器 (这将生成 C 代码)...")
    ocp_solver = AcadosOcpSolver(ocp)

    print("\n[INFO] C 代码已生成至: c_generated_code/")

    # =========================================================================
    # 步骤 2: 设置仿真参数
    # =========================================================================
    # 初始状态: 机器人从原点开始，航向为 0
    x0 = np.array([0.0, 0.0, 0.0])

    # 目标状态: 移动到 (2, 2)，航向旋转到 90 度 (pi/2)
    x_goal = np.array([2.0, 2.0, np.pi / 2])

    # 仿真参数
    N_sim = 100     # 仿真步数
    dt = 0.05       # 仿真时间步长 [s]，与 MPC 采样时间一致 (Tf/N = 1.0/20 = 0.05)

    # =========================================================================
    # 步骤 3: 运行闭环仿真
    # =========================================================================
    print("\n[步骤 3] 运行闭环 MPC 仿真...")
    x_traj, u_traj, t_traj, solve_times = run_closed_loop_simulation(
        ocp_solver, x0, x_goal, N_sim, dt
    )

    # =========================================================================
    # 步骤 4: 绘制结果
    # =========================================================================
    print("\n[步骤 4] 绘制仿真结果...")
    plot_results(x_traj, u_traj, t_traj, x_goal,
                 save_path='omnidirectional_trajectory.png')

    # =========================================================================
    # 步骤 5: 清理资源
    # =========================================================================
    # 释放求解器资源（可选，Python 会自动垃圾回收）
    del ocp_solver

    print("\n[完成] 全向轮 MPC 仿真已完成!")
    print("=" * 60)


if __name__ == '__main__':
    main()
