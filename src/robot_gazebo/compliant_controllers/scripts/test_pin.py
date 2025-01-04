from pathlib import Path
import pinocchio
import numpy as np
import sys

def compute_separated_C_matrix(model, data, q):
    """
    通过数值方法计算分离的科氏力矩矩阵 C(q)。
    """
    nv = model.nv
    C_matrix = np.zeros((nv, nv))
    dq_unit = np.zeros(nv)  # 单位速度向量

    for i in range(nv):
        dq_unit[i] = 1.0  # 设置第 i 个速度为 1
        tau_plus = pinocchio.rnea(model, data, q, dq_unit, np.zeros(nv))  # τ(q, [1, 0, ..., 0], 0)
        dq_unit[i] = 0.0  # 重置为 0

        C_matrix[:, i] = tau_plus  # 提取对应列

    return C_matrix

def main():
    # 设置 URDF 文件路径
    urdf_filename = "/home/xiaomeng/code/robot_ws/src/franka_ros2/franka_description/robots/fr3/fr3.urdf"

    # 检查 URDF 文件是否存在
    if not Path(urdf_filename).is_file():
        print(f"URDF 文件未找到: {urdf_filename}")
        sys.exit(1)

    # 加载模型
    model = pinocchio.buildModelFromUrdf(urdf_filename)
    print(f"模型名称: {model.name}")

    # 创建数据结构
    data = model.createData()

    # 随机关节位置和速度
    q = pinocchio.randomConfiguration(model)
    v = np.random.rand(model.nv)  # 随机关节速度
    print(f"关节位置 q:\n{q}")
    print(f"关节速度 v:\n{v}")

    # 1. 计算惯性矩阵 M(q)
    M = pinocchio.crba(model, data, q)
    # 修正惯性矩阵的对称性
    M = M + M.T - np.diag(M.diagonal())
    print("\n惯性矩阵 M(q):")
    print(M)

    # 2. 计算完整的科氏力矩项 C * v
    C_full = pinocchio.rnea(model, data, q, v, np.zeros(model.nv)) - pinocchio.rnea(model, data, q, np.zeros(model.nv), np.zeros(model.nv))
    print("\n完整科氏力矩项 C(q, v) * v:")
    print(C_full)

    # 3. 计算分离的科氏力矩矩阵 C(q)
    C_matrix = compute_separated_C_matrix(model, data, q)
    print("\n分离的科氏力矩矩阵 C(q):")
    print(C_matrix)

    # 4. 计算重力项 G(q)
    g = pinocchio.computeGeneralizedGravity(model, data, q)
    print("\n重力项 G(q):")
    print(g)

if __name__ == "__main__":
    main()
