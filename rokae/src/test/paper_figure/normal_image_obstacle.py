#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import random
import copy
import math
import select
import termios
import tty
import numpy as np
import math
from numpy import random
import matplotlib.pyplot as plt


def writelogs(write_data):
    # write_data.sort(key=takeSecond)
    # 打开文件
    file_name = 'random_deviation.txt'

    fo = open(file_name, 'a+')
    print("文件名为: ", fo.name)
    # for every in write_data:
    fo.write(write_data + "\n")

    fo.close()


def figure_show(x_datasets, normal_count, nsplanner_count):

    plt.title("planner demo")
    plt.xlabel("sigma distance")
    plt.ylabel("success rate")
    parameter_normal = np.polyfit(x_datasets, normal_count, 3)
    p_normal = np.poly1d(parameter_normal)

    plt.plot(x_datasets, p_normal(x_datasets), color='g', label='Traditional')

    parameter_our = np.polyfit(x_datasets, nsplanner_count, 3)
    p_our = np.poly1d(parameter_our)
    plt.plot(x_datasets, p_our(x_datasets),
             linewidth=2.0, color='blue', linestyle='-', label='Our')

    plt.legend()
    plt.show()


def bar_show(x_datasets, normal_bar, nsplanner_bar):

    # 并列柱状图
    plt.rcParams['font.sans-serif'] = ['SimHei']  # 设置字体以便支持中文
    x = np.arange(5)  # 柱状图在横坐标上的位置
    # 列出你要显示的数据，数据的列表长度与x长度相同
    y1 = [1, 3, 5, 4, 2]
    y2 = [2, 5, 3, 1, 6]

    bar_width = 0.001  # 设置柱状图的宽度
    tick_label = x_datasets

    # 绘制并列柱状图
    plt.bar(x_datasets, normal_bar, bar_width, color='b', label='Traditional')
    x_width = []
    xticks_label = []
    for i in range(len(x_datasets)):
        x_width.append(x_datasets[i]+bar_width)
        xticks_label.append(x_datasets[i]+bar_width/2)
    plt.bar(x_width, nsplanner_bar, bar_width, color='g', label='Our')

    plt.legend()  # 显示图例，即label
    # 显示x坐标轴的标签,即tick_label,调整位置，使其落在两个直方图中间位置
    plt.xticks(xticks_label, tick_label)
    plt.show()


def obstacle_deduce():

    file_name = 'random_deviation_obstacle.txt'
    fo = open(file_name, 'a+')
    size = 10
    deviation = 0.03
    mu = 0
    # datasets = []
    writelogs('x,y,semidiameter,normal_success,nsplanner_success')
    # datasets = [0.01, 0.02, 0.03, 0.04, 0.05]
    x_datasets = []
    normal_count = []
    nsplanner_count = []
    normal_bar = []
    nsplanner_bar = []
    for step in range(0, 10):
        is_probability = False
        is_success_nsplanner = False
        current_sigma = round(float(step)/100, 2)
        x_datasets.append(current_sigma)
        semidiameter = np.random.normal(loc=mu, scale=current_sigma, size=size)
        angle = np.random.randint(360, size=size)
        normal_num = 0
        nsplanner_num = 0
        for number in range(len(semidiameter)):
            x_current = abs(semidiameter[number]) * \
                math.cos(2 * math.pi * angle[number] / 360)
            y_current = abs(semidiameter[number]) * \
                math.sin(2 * math.pi * angle[number] / 360)

            if abs(semidiameter[number]) >= deviation:
                is_probability = True
                normal_num += 1

            print('current epoch is {} round {} sequence '.format(step, number+1))
            is_success_nsplanner = True
            print(angle[number])
            if float(angle[number]) >= 105 or float(angle[number]) <= 75:
                # if is_success_nsplanner:
                nsplanner_num += 1

            string = ('{},{},{},{},{}'.format(x_current, y_current,
                                              semidiameter[number], is_probability, is_success_nsplanner))
            writelogs(string)

            is_probability = False
            is_success_nsplanner = False

        normal_count.append(float(normal_num) / size)
        nsplanner_count.append(float(nsplanner_num)/size)

        normal_bar.append(float(normal_num * 3)/size)
        nsplanner_bar.append(float(float(nsplanner_num * 4)/size))

    figure_show(x_datasets, normal_count, nsplanner_count)

    bar_show(x_datasets, normal_bar, nsplanner_bar)

    writelogs('data_summary')
    for i in range(0, len(x_datasets)):
        string = ('{},{},{}'.format(
            x_datasets[i], normal_count[i], nsplanner_count[i]))
        writelogs(string)


if __name__ == "__main__":
    obstacle_deduce()
