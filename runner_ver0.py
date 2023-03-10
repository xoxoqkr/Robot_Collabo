# -*- coding: utf-8 -*-
##'Instance_random','Instance_cluster'
import time

ITE_NUM = 1
driver_speeds = [3]
robot_speeds = [1.5]
robot_nums = [0,30]
driver_nums = [20]

scenarios  = []
for i in driver_speeds:
    for j in robot_speeds:
        for k in robot_nums:
            for l in driver_nums:
                scenarios .append([i,j,k,l])
init = 0

for sc_info in scenarios:
    for ite in range(ITE_NUM):
        start_time = time.time()
        dir = '_instance__ite_' + str(ite) + '.txt'
        exec(open('Simulator_v1.py', encoding='UTF8').read(),
             globals().update(rider_speed = sc_info[0], robot_speed= sc_info[1],
                              robot_num = sc_info[2],rider_num = sc_info[3], ite = ite, dir = dir))
        end_time = time.time()
        f = open('C:/Users/박태준/PycharmProjects/Robot_Collabo/exp_log.txt','a')
        if init == 0:
            con = ';driver_speed;robot_speed;robot_num;driver_num;start;computation_time; \n'
            f.write(con)
        tm = time.localtime(time.time())
        tm_info = time.strftime('%Y-%m-%d-%I-%M-%S-%p', tm)
        con = ';{};{};{};{};{};{}; \n'.format(sc_info[0],sc_info[1],sc_info[2],sc_info[3],tm_info,(end_time - start_time)/60)
        f.write(con)
        f.close()
        init += 1