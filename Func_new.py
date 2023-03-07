# -*- coding: utf-8 -*-
#.py 설명 :
# -> LinearizedASP_gurobi.py : gurobipy를 이용해 보조금 할당 문제를 풀이하는 함수

import random
import numpy
import math
import time
from MIP_Model import LinearizedCollaboProblem
import operator

def distance(p1_x, p1_y, p2_x,p2_y):
    """
    Calculate 4 digit rounded euclidean distance between p1 and p2
    :para
    m p1:
    :param p2:
    :return: 4 digit rounded euclidean distance between p1 and p2
    """
    """
    counter('distance1')
    if rider_count == 'rider':
        counter('distance2')
    elif rider_count == 'xgboost':
        counter('distance3')
    else:
        pass
    """
    euc_dist = math.sqrt((p1_x - p2_x)**2 + (p1_y - p2_y)**2)
    return euc_dist


def CalculateS_ijm(driver_set, customers_set, middle_point_set, rider_names, customer_names):
    res = numpy.zeros((len(rider_names),len(customer_names),len(middle_point_set)))
    print( 'CalculateS_ijm',rider_names, customer_names)
    #print(res.shape)
    #input('shape 확인')
    i_index = 0
    for i in rider_names:
        rider = driver_set[i]
        j_index = 0
        for j in customer_names:
            customer = customers_set[j]
            for m in range(len(res[i_index][j_index])):
                #print(i_index,j_index,m)
                #print(len(rider_names),len(customer_names))
                #print(len(res[i_index][j_index]))
                #input('길이 확인')
                middle = middle_point_set[m]
                #rider_last_node = rider.visited_route[-1][2]
                rider_last_node = rider.exp_end_location
                org = distance(rider_last_node[0],rider_last_node[1],customer.store_loc[0],customer.store_loc[1]) + distance(customer.store_loc[0],customer.store_loc[1],customer.location[0],customer.location[1])
                rev = distance(rider_last_node[0],rider_last_node[1],middle[0],middle[1]) + distance(middle[0],middle[1],customer.location[0],customer.location[1])
                res[i_index,j_index,m] = round(org - rev ,4)
            j_index  += 1
        i_index += 1
    return res

def CalculateV_ijm(driver_set, customers_set, middle_point_set, rider_names, customer_names, r = 0.9):
    #r = 로봇 사용에 대한 할인율
    res = numpy.zeros((len(rider_names),len(customer_names),len(middle_point_set)))
    print('CalculateV_ijm', rider_names, customer_names)
    i_index = 0
    for i in rider_names:
        rider = driver_set[i]
        j_index = 0
        for j in customer_names:
            customer = customers_set[j]
            for m in range(len(res[i_index][j_index])):
                middle = middle_point_set[m]
                #rider_last_node = rider.visited_route[-1][2]
                rider_last_node = rider.exp_end_location
                dist = distance(rider_last_node[0],rider_last_node[1],middle[0],middle[1]) + distance(middle[0],middle[1],customer.location[0],customer.location[1])
                val = customer.fee*r - 150*dist/rider.speed
                res[i_index,j_index,m] = round(val ,4)
            j_index += 1
        i_index += 1
    return res

def CalculateZeroY(driver_set, customers_set, middle_point_set,robots, rider_names, customer_names, robot_names, now_time = 0, thres = 5):
    zero_x = []
    zero_y = []
    zero_r = []
    j_index = 0
    for j in customer_names:
        customer = customers_set[j]
        for m in range(len(middle_point_set)):
            middle = middle_point_set[m]
            r_index = 0
            for r in robot_names:
                robot = robots[r]
                i_index = 0
                for i in rider_names:
                    rider = driver_set[i]
                    robot_arrive = now_time + (distance(robot.loc[0],robot.loc[1],customer.store_loc[0],customer.store_loc[0]) + distance(customer.store_loc[0],customer.store_loc[0],middle[0],middle[1]))/robot.speed
                    rider_arrive = rider.exp_end_time + (distance(rider.visited_route[-1][2][0],rider.visited_route[-1][2][0],middle[0],middle[1]))/rider.speed
                    if robot_arrive - rider_arrive> thres:
                        zero_x.append([i_index,j_index])
                        zero_y.append([j_index,m])
                        zero_r.append([r_index,j_index])
                    i_index += 1
                r_index += 1
        j_index += 1
    return [zero_x, zero_y, zero_r]

def CalculateRo(riders,rider_names):
    times = []
    res = []
    count = 0
    for rider_name in rider_names:
        rider = riders[rider_name]
        times.append([count, rider_name, rider.exp_end_time])
        res.append(0)
        count += 1
    times.sort(key=operator.itemgetter(2))
    count = 1
    for info in times:
        res[info[0]] = count
        count += 1
    return res

def CalculateTargetNames(driver_set, customers_set, robots, platforms, t_now, interval= 5):
    rider_names = []
    customer_names = []
    robot_names = []
    for rider_name in driver_set:
        driver = driver_set[rider_name]
        if driver.exp_end_time < t_now + interval and len(driver.picked_orders) < driver.max_order_num:
            rider_names.append(rider_name)
    for customer_name in customers_set:
        if customers_set[customer_name].time_info[1] == None and customers_set[customer_name].cancel == False:
            customer_names.append(customer_name)
    #for task_name in platforms.platforms:
    #    task = platforms.platforms[task_name]
    #    if task.picked == False:
    #        customer_names.append(task_name)
    for robot_name in robots:
        if robots[robot_name].idle == True:
            robot_names.append(robot_name)
    return rider_names, customer_names, robot_names


def Platform_process5(env, platform, orders, riders, robots, interval = 5, end_t = 100, warm_up_time = 20):
    f = open("loop시간정보.txt", 'a')
    locat_t = time.localtime(time.time())
    f.write('시간;{};라이더수;{};'.format(locat_t,len(riders)))
    f.write('\n')
    f.close()
    yield env.timeout(warm_up_time) #warm-up time
    #middle point 설정(지금은 격자로 설정)
    middle_point_set = []
    for i in list(range(1, 50, 6)):
        for j in list(range(1, 50, 6)):
            middle_point_set.append([i,j])
    while env.now <= end_t:
        #1 문제 풀이
        t_now = env.now
        rider_names, customer_names, robot_names = CalculateTargetNames(riders, orders, robots, platform, t_now, interval=interval)
        ro = CalculateRo(riders, rider_names)
        input_s = CalculateS_ijm(riders, orders, middle_point_set,rider_names, customer_names)
        input_v = CalculateV_ijm(riders, orders, middle_point_set,rider_names, customer_names)
        print('확인 shape',input_s.shape, input_v.shape)
        zero_info = []
        #zero_info = CalculateZeroY(riders, orders, middle_point_set, robots, rider_names, customer_names, robot_names,now_time=t_now, thres=30)
        print(ro)
        #input('확인')
        if len(rider_names) > 0 and len(robot_names) > 0 :
            feasibility, solution = LinearizedCollaboProblem(rider_names, customer_names, robot_names, middle_point_set, ro, input_v,input_s, zero_info, print_gurobi = True)
            #2 task에 middle point 할당
            if feasibility == True:
                print('Solved')
                count = 0
                for info1 in solution[2]:
                    for info2 in solution[1]:
                        if info1[1] == info2[0]:
                            middle_point = middle_point_set[info2[1]]
                            break
                    robot = robots[robot_names[info1[0]]]
                    print(info1, middle_point)
                    if count < len(solution[1]):
                        print(solution[1][count])
                    else:
                        print(count,solution[1],solution[2] )
                        input('error')
                        continue
                    target_order = orders[customer_names[info1[1]]]
                    target_order.robot = True
                    #target_order.fee += 3000
                    target_order.middle_point = middle_point
                    for task_name in platform.platform:
                        task = platform.platform[task_name]
                        if task.customers[0] == target_order.name:
                            robot.run_process = env.process(robot.JobAssign(target_order, task))
                    count += 1
            else:
                print('infeasible')
            #input('!!!! Check!!!!!')
        robo_count = 0
        for task_name in platform.platform:
            task = platform.platform[task_name]
            if task.robot == True and task.robot_t == env.now:
                print(task.index, task.robot_t, task.route[0][2])
                robo_count += 1
        if robo_count > 0:
            print('T:', env.now, 'robot check')
        yield env.timeout(interval)
        for task_name in platform.platform:
            task = platform.platform[task_name]
            if task.freeze == True and task.gen_t < env.now:
                task.freeze = False


