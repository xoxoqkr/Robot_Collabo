# -*- coding: utf-8 -*-
#.py 설명 : 재배치 의사 결정 코드
import numpy
import math
import operator
import random


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
    try:
        euc_dist = math.sqrt((round(p1_x,2) - round(p2_x,2))**2 + (round(p1_y,2) - round(p2_y,2))**2)
        return euc_dist
    except:
        print(p1_x, p2_x,p1_y,p2_y)
        input('Euc dist error')



def RobotRelocate(method, env, robot_names, robots, stores, used_robots, now_t = 0):
    store_order_count = []
    for store_name in stores:
        store_order_count.append(stores[store_name].got)
    ave_num = sum(store_order_count)/len(store_order_count)
    #1 로봇 점수 계산
    for robot_name in robot_names:  # 현재 대기 중인 로봇 중 idle 상태(=마지막 위치가 m)인 로봇을 다시 가게 근처로 재 배치
        robot = robots[robot_name]
        # if robot.idle == True and robot.relocate == False and robot.visited_nodes[-1][2] == 'm':
        if robot.idle == True and robot.relocate == False and robot.name not in used_robots and robot.run_process == None:
            # store_name = random.choice(stores)
            # store = stores[store_name]
            store_scores = []  # 가장 가까운 가게로 로봇 재배치 #현재 가게라면, 더이상 움직이지 X?
            ct_num = []
            if robot.relocate_info[1] > now_t:  # 향하던 가게가 있다면, 해당 가게로 다시 출발 시키자
                store_scores.append([robot.relocate_info[3],0, 0,0])
            else:  # 현재 가게나 중간 지점에 있다면, 다른 가게로 보내 보자.
                for store_name in stores:
                    store = stores[store_name]
                    store_scores.append(
                        [store_name, distance(store.location[0], store.location[1], robot.loc[0], robot.loc[1]),
                         len(store.received_orders),0])
                    ct_num.append(len(store.received_orders))
            if method == 'dist':
                store_scores.sort(key=operator.itemgetter(1))
            elif method == '#ct':  # 현재 주문이 가장 많은 가게로 이동
                store_scores.sort(key=operator.itemgetter(2), reverse=True)
            elif method == 'pareto':
                pareto_scores = []
                index1 = 0
                for store_score1 in store_scores:
                    tem = [store_score1[0], 0]
                    for store_score2 in store_scores:
                        if store_score1 != store_score2:
                            if store_score1[1] < store_score2[1] and store_score1[2] > store_score2[2]:
                                tem[1] += 1
                    pareto_scores.append(tem)
                    store_scores[index1][3] += tem[1]
                    index1 += 1
                pareto_scores.sort(key=operator.itemgetter(1), reverse=True)
                store_scores.sort(key=operator.itemgetter(3), reverse=True)
                #store_scores = [pareto_scores[0]]
            elif method == 'random':
                store_name = random.choice(list(range(len(stores))))
                store_scores = [[stores[store_name].name]]
            elif method == 'None':
                continue
            else:
                input('robot_relocate_rule error : current rule : {}'.format(method))
            for info in store_scores:
                store_name = info[0]
                if store_order_count[store_name] >= ave_num:
                    store = stores[store_name]
                    store_order_count[store_name] -= 1
                    robot.run_process = env.process(robot.RobotReLocate(store))  # robot에 작업 할당
                    break

def PavoneInputCalculator(stores, robots, drivers, now_t = 0):
    c = numpy.zeros((len(stores),len(stores)))
    v = []
    for i in stores:
        store1 = stores[i].location
        for j in stores:
            store2 = stores[j].location
            if i < j:
                dist = distance(store1[0], store1[1], store2[0], store2[1])
                c[i,j] = dist
                c[j, i] = dist
            elif i == j:
                pass
            else:
                pass
    store_infos = []
    for i in stores:
        store = stores[i]
        tem = [store.name, 0,0, store.got, 0]
        for j in drivers:
            driver = drivers[j]
            if store.name in driver.onhand:
                tem[4] += 1
        for j in robots:
            robot = robots[j]
            #print('PavoneInputCalculator',robot,robot.name, robot.relocate_info)
            if robot.relocate_info[3] == store.name:
                if robot.relocate_info[1] > now_t:
                    tem[2] += 1
                elif robot.idle == True:
                    tem[1] += 1
            #if robot.relocate_info[-1][1] > now_t and robot.relocate_info[-1][3] == store.name:
            #    tem[2] += 1
            #if robot.current_store == store.name:
            #if robot.relocate_info[-1][1] <= now_t and robot.relocate_info[-1][3] == store.name:
            #    tem[1] += 1
        store_infos.append(tem)
    for info in store_infos:
        s_z = min(info[2] + info[3] - info[4],0)
        v.append(max(info[1] + s_z , 0))
    return c, v

def RelocateAlgo1(env, x, robots, stores, now_t = 0):
    tmp = []
    for i in stores:
        for j in stores:
            if x[i,j] > 0:
                tmp.append([i,j])
    idle_robots = []
    for robot_name in robots:
        robot = robots[robot_name]
        if robot.relocate_info[1] < now_t and robot.idle == True:
            idle_robots.append(robot.name)
    random.shuffle(tmp)
    for _ in range(len(idle_robots)):
        rs = random.choice(tmp)
        r_dist = []
        for robot_name in idle_robots:
            robot_loc = robot.relocate_info[-1][2]
            store_loc = stores[rs[1]].location
            r_dist.append([robot_name, distance(store_loc[0],store_loc[1],robot_loc[0],robot_loc[1])])
        if len(r_dist) > 0:
            r_dist.sort(kep = operator.itemgetter(1))
            robot = robots[r_dist[0][0]]
            store = stores[rs[1]]
            robot.run_process = env.process(robot.RobotReLocate(store))  # robot에 작업 할당
            print('로봇 {} 가게 {}로 재배치 {}'.format(robot.name, store.name, robot.visited_nodes[-1]))

def RelocateAlgo2(env, x, robots, stores, robot_names): #pavone 2에 맞춘 작업
    """

    :param env:
    :param x:
    :param robots:
    :param stores:
    :param robot_names:
    :return:
    """
    """
    tmp = []
    for i in range(len(robot_names)):
        for j in range(len(stores)):
            print('ij확인',i, j)
            if x[i,j] > 0:
                tmp.append([i,j])    
    """
    #for info in tmp:
    for info in x:
        robot = robots[robot_names[info[0]]]
        store = stores[info[1]]
        robot.run_process = env.process(robot.RobotReLocate(store))  # robot에 작업 할당
        print('로봇 {} 가게 {}로 재배치 {}'.format(robot.name, store.name, robot.visited_nodes[-1]))


def PavoneInputCalculator2(stores, robots, robot_names):
    v = [] #가게에 필요한 주문 수 list index = 가게 이름
    r = [] #현대 주문을 idle인 로봇 수
    w = [] #가중치
    for robot_name in robot_names:
        robot = robots[robot_name]
        if robot.idle == True:
            end_node = robot.RobotCurrentLoc()
            if end_node == None:
                end_node = robot.visited_nodes[-1][1]
            r.append([robot.name, end_node])
            #print(end_node)
            #input('현 위치')
    c = numpy.zeros((len(r), len(stores)))  # index(i,j) = index(j,i) ; 로봇 i 현재 위치 ~ 가게 j 거리
    count1 = 0
    for robot_info in r:
        count2 = 0
        for store_name in stores:
            store = stores[store_name]
            c[count1,count2] = distance(robot_info[1][0], robot_info[1][1], store.location[0], store.location[1])
            count2 += 1
        count1 += 1
    for store_name in stores:
        store = stores[store_name]
        val = store.got #todo : 0405 더 detail한 계한 필요. 연구 노트 참고.
        v.append(val)
    sum_v = sum(v)
    weights = []
    for val in v:
        w.append(int(len(r)*(val/sum_v)))
        weights.append(val/sum_v)
    diff = len(r) - sum(w)
    if diff > 0:
        sampled = random.choices(range(len(v)), weights= weights, k = diff)#todo : 0405 더 detail한 계한 필요. 연구 노트 참고.
        for index in sampled:
            w[index] += 1
    return c, w