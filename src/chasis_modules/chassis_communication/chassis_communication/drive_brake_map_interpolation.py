# 						abs介入	abs介入
# 初始速度	37	33	38	40	40	43	38	kph
# 刹车开度	10	20	30	40	50	60	70	%
# 刹车压力	10	20	30	40	50	60	70	bar
# 加速度	-2	-3.15	-4.4	-5.5	-6.5	-7.7	-7.4	mps2


# 加速度（m/s2）	当前速度（k/h）
# 油门开度      0       10      20      30      40      50      55      60
# 10        0.67     0.05
# 20	    0.75	 0.32	 0.27
# 30	     1.4	  0.7	  0.6	 0.47	  0.3
# 40	    1.85	  1.2	 0.98	 0.85	 0.75	 0.55
# 50	     1.9	 1.65	 1.46	 1.35	 1.27	 0.99	  0.8
# 60	     2.4	    2	 1.89	 1.82	 1.65	  1.5	 1.44	  1.4
# 70	     2.5	 2.25	 2.16	  2.1	 1.93	 1.88	 1.78	  1.6
# 80	    2.72	 2.67	 2.55	  2.5	  2.4	 2.29	 2.28	 2.27
# 90	  缓慢上升	   3.3	  2.87	    2.8	   2.73	   2.7	   2.64	    2.6
# 100	  缓慢上升	  3.37	  3.35	    3.3	   3.27	   3.25	   3.23	   3.22

import pylab
import numpy as np
from matplotlib import cm
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

break_command = [10, 20, 30, 40, 50, 60, 70]
deceleration = [-2, -3.15, -4.4, -5.5, -6.5, -7.7, -7.4]

# 表格中的0表示不可达范围
acceleration_under_different_velocity = np.array([[0.67, 0.05, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00],
                                                  [0.75, 0.32, 0.27, 0.00, 0.00, 0.00, 0.00, 0.00],
                                                  [1.40, 0.70, 0.60, 0.47, 0.30, 0.00, 0.00, 0.00],
                                                  [1.85, 1.20, 0.98, 0.85, 0.75, 0.55, 0.00, 0.00],
                                                  [1.90, 1.65, 1.46, 1.35, 1.27, 0.99, 0.80, 0.00],
                                                  [2.40, 2.00, 1.89, 1.82, 1.65, 1.50, 1.44, 1.40],
                                                  [2.50, 2.25, 2.16, 2.10, 1.93, 1.88, 1.78, 1.60],
                                                  [2.72, 2.67, 2.55, 2.50, 2.40, 2.29, 2.28, 2.27],
                                                  [0.00, 3.30, 2.87, 2.80, 2.73, 2.70, 2.64, 2.60],
                                                  [0.00, 3.37, 3.35, 3.30, 3.27, 3.25, 3.23, 3.22]])

# 曲面可视化
# figure = plt.figure()
# ax = Axes3D(figure)
# X = np.array([0, 10, 20, 30, 40, 50, 55, 60])
# Y = np.arange(10, 110, 10)
# print(X, Y )
# X,Y = np.meshgrid(X, Y)
# ax.plot_surface(X, Y, acceleration_under_different_velocity,rstride=1,cstride=1,cmap='rainbow')
# plt.show()

# ---------------------------------------------------------------------------------- surface_fit --------------------------------------------------------------------------------- #


# def fun(x):  # 处理符号问题
#     round(x, 2)
#     if x >= 0:
#         return '+' + str(x)
#     else:
#         return str(x)


# # 主函数
# if __name__ == '__main__':

#     X = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 20, 20, 20, 20, 20, 20, 20, 20, 20,
#          30, 30, 30, 30, 30, 30, 30, 30, 40, 40, 40, 40, 40, 40, 40, 40, 50, 50, 50, 50, 50, 50, 50, 55, 55, 55, 55, 55, 55,
#          60, 60, 60, 60, 60, ]

#     Z = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 20, 30, 40, 50, 60, 70, 80, 90, 100, 30, 40, 50, 60, 70, 80, 90, 100,
#          30, 40, 50, 60, 70, 80, 90, 100, 40, 50, 60, 70, 80, 90, 100, 50, 60, 70, 80, 90, 100, 60, 70, 80, 90, 100, ]

#     Y = [0.67, 0.75, 1.4, 1.85, 1.9, 2.4, 2.5, 2.72, 2.73, 2.74, 0.05, 0.32, 0.7, 1.2, 1.65, 2, 2.25, 2.67, 3.3, 3.37,
#          0.27, 0.6, 0.98, 1.46, 1.89, 2.16, 2.55, 2.87, 3.35, 0.47, 0.85, 1.35, 1.82, 2.1, 2.5, 2.8, 3.3,
#          0.3, 0.75, 1.27, 1.65, 1.93, 2.4, 2.73, 3.27, 0.55, 0.99, 1.5, 1.88, 2.29, 2.7, 3.25, 0.8, 1.44, 1.78, 2.28, 2.64, 3.23,
#          1.4, 1.6, 2.27, 2.6, 3.22]

#     # print(X)
#     # print(len(X))
#     # print(Y)
#     # print(len(Y))
#     # print(Z)
#     # print(len(Z))

#     # 求方程系数
#     sigma_x = 0
#     for i in X:
#         sigma_x += i
#     sigma_y = 0
#     for i in Y:
#         sigma_y += i
#     sigma_z = 0
#     for i in Z:
#         sigma_z += i
#     sigma_x2 = 0
#     for i in X:
#         sigma_x2 += i * i
#     sigma_y2 = 0
#     for i in Y:
#         sigma_y2 += i * i
#     sigma_x3 = 0
#     for i in X:
#         sigma_x3 += i * i * i
#     sigma_y3 = 0
#     for i in Y:
#         sigma_y3 += i * i * i
#     sigma_x4 = 0
#     for i in X:
#         sigma_x4 += i * i * i * i
#     sigma_y4 = 0
#     for i in Y:
#         sigma_y4 += i * i * i * i
#     sigma_x_y = 0
#     for i in range(len(X)):
#         sigma_x_y += X[i] * Y[i]
#     # print(sigma_xy)
#     sigma_x_y2 = 0
#     for i in range(len(X)):
#         sigma_x_y2 += X[i] * Y[i] * Y[i]
#     sigma_x_y3 = 0
#     for i in range(len(X)):
#         sigma_x_y3 += X[i] * Y[i] * Y[i] * Y[i]
#     sigma_x2_y = 0
#     for i in range(len(X)):
#         sigma_x2_y += X[i] * X[i] * Y[i]
#     sigma_x2_y2 = 0
#     for i in range(len(X)):
#         sigma_x2_y2 += X[i] * X[i] * Y[i] * Y[i]
#     sigma_x3_y = 0
#     for i in range(len(X)):
#         sigma_x3_y += X[i] * X[i] * X[i] * Y[i]
#     sigma_z_x2 = 0
#     for i in range(len(X)):
#         sigma_z_x2 += Z[i] * X[i] * X[i]
#     sigma_z_y2 = 0
#     for i in range(len(X)):
#         sigma_z_y2 += Z[i] * Y[i] * Y[i]
#     sigma_z_x_y = 0
#     for i in range(len(X)):
#         sigma_z_x_y += Z[i] * X[i] * Y[i]
#     sigma_z_x = 0
#     for i in range(len(X)):
#         sigma_z_x += Z[i] * X[i]
#     sigma_z_y = 0
#     for i in range(len(X)):
#         sigma_z_y += Z[i] * Y[i]

#     # 给出对应方程的矩阵形式
#     a = np.array([[sigma_x4, sigma_x3_y, sigma_x2_y2, sigma_x3, sigma_x2_y, sigma_x2],
#                   [sigma_x3_y, sigma_x2_y2, sigma_x_y3, sigma_x2_y, sigma_x_y2, sigma_x_y],
#                   [sigma_x2_y2, sigma_x_y3, sigma_y4, sigma_x_y2, sigma_y3, sigma_y2],
#                   [sigma_x3, sigma_x2_y, sigma_x_y2, sigma_x2, sigma_x_y, sigma_x],
#                   [sigma_x2_y, sigma_x_y2, sigma_y3, sigma_x_y, sigma_y2, sigma_y],
#                   [sigma_x2, sigma_x_y, sigma_y2, sigma_x, sigma_y, len(X)]])
#     b = np.array([sigma_z_x2, sigma_z_x_y, sigma_z_y2, sigma_z_x, sigma_z_y, sigma_z])
#     # 高斯消元解线性方程
#     res = np.linalg.solve(a, b)
#     # print(a)
#     # print(b)
#     # print(x)
#     # print("-----------------------")

#     # 输出方程形式，根据已经标定的结果，使用一个二元二次方程来拟合标定表，打印拟合结果
#     print("z={}*x^2{}*xy{}*y^2{}*x{}*y{}".format(fun(res[0]), fun(res[1]), fun(res[2]), fun(res[3]), fun(res[4]), fun(res[5])))

#     current_velocity = 60
#     acceleration_request = 1.4
#     print(res[0] * current_velocity * current_velocity + res[1] * current_velocity * acceleration_request + res[2]
#           * acceleration_request * acceleration_request + res[3] * current_velocity + res[4] * acceleration_request + res[5])

#     # # 画曲面图和离散点
#     # fig = plt.figure()  # 建立一个空间
#     # ax = fig.add_subplot(111, projection='3d')  # 3D坐标

#     # n = 256
#     # u = np.linspace(0, 100, n)  # 创建一个等差数列
#     # u1 = np.linspace(0, 60, n)
#     # u2 = np.linspace(0, 3.37, n)
#     # x, y = np.meshgrid(u1, u2)  # 转化成矩阵
#     # # x = 30
#     # # y = 20
#     # # 给出方程
#     # z = res[0] * x * x + res[1] * x * y + res[2] * y * y + res[3] * x + res[4] * y + res[5]
#     # # print(z)
#     # # 画出曲面
#     # ax.plot_surface(z, x, y, cmap=cm.jet)
#     # # 画出点
#     # # ax.scatter(X, Y, Z, c='r')
#     # plt.show()


if __name__ == "__main__":
    y = np.array([10, 20, 30, 40, 50, 60])
    x = np.array([-2, -3.5, -4.4, -5.5, -6.5, -7.7])

    z1 = np.polyfit(x, y, 1)              # 曲线拟合，返回值为多项式的各项系数
    p1 = np.poly1d(z1)                    # 返回值为多项式的表达式，也就是函数式子
    print(p1)
    y_pred = p1(-4.4)                        # 根据函数的多项式表达式，求解 y
    print(y_pred)
    # print(np.polyval(p1, 29))             根据多项式求解特定 x 对应的 y 值
    # print(np.polyval(z1, 29))             根据多项式求解特定 x 对应的 y 值

    # # 绘图
    # plot1 = pylab.plot(x, y, '*', label='original values')
    # plot2 = pylab.plot(x, y_pred, 'r', label='fit values')
    # pylab.title('')
    # pylab.xlabel('')
    # pylab.ylabel('')
    # pylab.legend(loc=3, borderaxespad=0., bbox_to_anchor=(0, 0))
    # pylab.show()
    # pylab.savefig('p1.png', dpi=200, bbox_inches='tight')
