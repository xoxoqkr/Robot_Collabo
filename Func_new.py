# -*- coding: utf-8 -*-
#.py 설명 :
# -> LinearizedASP_gurobi.py : gurobipy를 이용해 보조금 할당 문제를 풀이하는 함수
import copy
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
    sync_value = numpy.zeros((len(rider_names),len(customer_names),len(middle_point_set),len(robot_names)))
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
                    #rider_arrive = rider.exp_end_time + (distance(rider.route[-1][2][0],rider.route[-1][2][0],middle[0],middle[1]))/rider.speed
                    rider_arrive = rider.exp_end_time + (distance(rider.exp_end_location[0], rider.exp_end_location[1], middle[0], middle[1])) / rider.speed
                    if robot_arrive - rider_arrive> thres:
                        zero_x.append([i_index,j_index])
                        zero_y.append([j_index,m])
                        zero_r.append([r_index,j_index])
                    sync_value[i_index,j_index,m,r_index] = max(0,robot_arrive - rider_arrive)
                    i_index += 1
                r_index += 1
        j_index += 1
    return [zero_x, zero_y, zero_r, sync_value]

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


def Platform_process5(env, platform, orders, riders, robots, stores, interval = 5, end_t = 100, warm_up_time = 20,solver_time_limit = 1000):
    f = open("loop시간정보.txt", 'a')
    locat_t = time.localtime(time.time())
    f.write('시간;{};라이더수;{};'.format(locat_t,len(riders)))
    f.write('\n')
    f.close()
    yield env.timeout(warm_up_time) #warm-up time
    #middle point 설정(지금은 격자로 설정)
    middle_point_set = []
    for i in list(range(1, 50, 4)):
        for j in list(range(1, 50, 4)):
            middle_point_set.append([i,j])
    while env.now <= end_t:
        #1 문제 풀이
        t_now = env.now
        #로봇의 relocating을 멈출 것
        for robot_name in robots:
            robot = robots[robot_name]
            reloc = robot.RobotCurrentLoc()
            if reloc != None and robot.relocate == True: #relocat 중지하기.
                robot.relocate = False
                robot.loc = reloc
                robot.visited_nodes.append([env.now, reloc, 's'])
        rider_names, customer_names, robot_names = CalculateTargetNames(riders, orders, robots, platform, t_now, interval=interval)
        ro = CalculateRo(riders, rider_names)
        input_s = CalculateS_ijm(riders, orders, middle_point_set,rider_names, customer_names)
        input_v = CalculateV_ijm(riders, orders, middle_point_set,rider_names, customer_names)
        print('확인 shape',input_s.shape, input_v.shape, len(robot_names))
        zero_info = CalculateZeroY(riders, orders, middle_point_set, robots, rider_names, customer_names, robot_names,now_time=t_now, thres=30)

        try:
            sync_t = copy.deepcopy(zero_info[3])
            print('확인 shape2', sync_t.shape)
        except:
            sync_t = []
        sync_t = []
        zero_info = []
        print(ro)
        #input('확인')
        if len(rider_names) > 0 and len(robot_names) > 0 :
            feasibility, solution = LinearizedCollaboProblem(rider_names, customer_names, robot_names, middle_point_set, ro,
                                                             input_v,input_s, zero_info, print_gurobi = True, timelimit=solver_time_limit
                                                             ,add_obj = sync_t)
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
                        #input('error')
                        continue
                    target_order = orders[customer_names[info1[1]]]
                    target_order.robot = True
                    #target_order.fee += 3000
                    target_order.middle_point = middle_point
                    for task_name in platform.platform:
                        task = platform.platform[task_name]
                        if task.customers[0] == target_order.name:
                            store_name = orders[task.customers[0]].store
                            stores[store_name].got -= 1
                            robot.run_process = env.process(robot.JobAssign(target_order, task))
                    count += 1
                print('rider_Select', ro)
                #print('order_Select', solution[3])
                print('order_Select', sorted(solution[3][:min(len(solution[3])-1,len(ro)+1)]))
            else:
                print('infeasible')
            #input('!!!! Check!!!!!')
        for robot_name in robots: #현재 대기 중인 로봇 중 마지막 위치가 m인 로봇을 다시 가게 근처로 재 배치
            robot = robots[robot_name]
            if robot.idle == True and robot.relocate == False and robot.visited_nodes[-1][2] == 'm':
                #store_name = random.choice(stores)
                #store = stores[store_name]
                store_dist = [] #가장 가까운 가게로 로봇 재배치 #현재 가게라면, 더이상 움직이지 X?
                for store_name in stores:
                    store = stores[store_name]
                    store_dist.append([store_name,distance(store.location[0],store.location[1],robot.loc[0],robot.loc[1])])
                store_dist.sort(key = operator.itemgetter(1))
                store = stores[store_dist[0][0]]
                robot.RobotReLocate(store)
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

def InstanceSave(stores, customers, riders, robots, title_info='',ite = 1, root = ''):
    store_title = root + 'store_instance_{}_ite_{}.txt'.format(title_info,ite)
    f = open(store_title, 'a')
    f.write('#;x;y;\n')
    for store_name in stores:
        store = stores[store_name]
        content = '{};{};{};\n'.format(store.name,store.location[0], store.location[1])
        f.write(content)
    f.close()
    customer_title = root + 'customer_instance_{}_ite_{}.txt'.format(title_info, ite)
    f = open(customer_title, 'a')
    f.write('#;gent_t;x;y;s#;s_x;s_y;\n')
    for customer_name in customers:
        customer = customers[customer_name]
        content = '{};{};{};{};{};{};{}; \n'.format(customer.name,customer.time_info[0],customer.location[0], customer.location[1],customer.store,customer.store_loc[0], customer.store_loc[1])
        f.write(content)
    f.close()
    driver_title = root + 'driver_instance_{}_ite_{}.txt'.format(title_info, ite)
    f = open(driver_title, 'a')
    f.write('#;x;y;\n')
    for rider_name in riders:
        rider = riders[rider_name]
        content = '{};{};{}; \n'.format(rider.name,rider.visited_route[0][2][0],rider.visited_route[0][2][1])
        f.write(content)
    f.close()
    robot_title = root + 'robot_instance_{}_ite_{}.txt'.format(title_info, ite)
    f = open(robot_title, 'a')
    f.write('#;x;y;\n')
    for robot_name in robots:
        robot = robots[robot_name]
        content = '{};{};{}; \n'.format(robot.name,robot.visited_nodes[0][1][0],robot.visited_nodes[0][1][1])
        f.write(content)
    f.close()

def ResultPrint2(customers, drivers, robots):
    #고객 부
    d_lead_time = []
    d_pick_up_time = []
    r_lead_time = []
    r_pick_up_time = []
    served_num = 0
    p_sync_time = []
    n_sync_time = []
    for customer_name in customers:
        customer = customers[customer_name]
        if customer.time_info[4] != None:
            served_num += 1
            if customer.robot_t != None:
                r_lead_time.append(customer.time_info[4] - customer.time_info[0])
                r_pick_up_time.append(customer.time_info[1] - customer.time_info[0])
                sync_t = customer.time_info[1] - customer.robot_t
                if sync_t < 0:
                    n_sync_time.append(sync_t)
                else:
                    p_sync_time.append(sync_t)
            else:
                d_lead_time.append(customer.time_info[4] - customer.time_info[0])
                d_pick_up_time.append(customer.time_info[1] - customer.time_info[0])
    customer_res = [d_lead_time,d_pick_up_time, r_lead_time ,r_pick_up_time,p_sync_time,n_sync_time]
    #라이더 부
    driver_incomes = []
    driver_dists = []
    for driver_name in drivers:
        driver = drivers[driver_name]
        driver_incomes.append(driver.income)
        dist = 0
        for index in range(1,len(driver.route)):
            bf = driver.route[index - 1][2]
            af = driver.route[index][2]
            dist += distance(bf[0],bf[1],af[0],af[1])
        driver_dists.append([driver.name, driver.income, driver.robot_use, dist])

    #로봇 부
    robot_dists = []
    for robot_name in robots:
        robot = robots[robot_name]
        dist = 0
        for index in range(1,len(robot.visited_nodes)):
            bf = robot.visited_nodes[index - 1][1]
            af = robot.visited_nodes[index][1]
            dist += distance(bf[0],bf[1],af[0],af[1])
        robot_dists.append([robot.name, robot.income, dist])
    return customer_res, driver_dists, robot_dists


def ResultSave2(customers, drivers, robots, saved_title = '',dir_root= 'C:/Users/박태준/PycharmProjects/Robot_Collabo/res'):
    #고객 부
    d_lead_time = []
    d_pick_up_time = []
    r_lead_time = []
    r_pick_up_time = []
    served_num = 0
    p_sync_time = []
    n_sync_time = []
    customer_saves = []
    for customer_name in customers:
        customer = customers[customer_name]
        if customer.time_info[4] != None:
            served_num += 1
            if customer.robot_t != None:
                r_lead_time.append(customer.time_info[4] - customer.time_info[0])
                r_pick_up_time.append(customer.time_info[1] - customer.time_info[0])
                sync_t = customer.time_info[1] - customer.robot_t
                if sync_t < 0:
                    n_sync_time.append(sync_t)
                else:
                    p_sync_time.append(sync_t)
            else:
                d_lead_time.append(customer.time_info[4] - customer.time_info[0])
                d_pick_up_time.append(customer.time_info[1] - customer.time_info[0])
            content = [customer.name] + customer.time_info[:5] + [customer.fee] + [customer.robot_t,customer.middle_point_arrive_t, customer.v_middle_point_arrive_t, customer.distance]
            customer_saves.append(content)
    customer_res = [d_lead_time,d_pick_up_time, r_lead_time ,r_pick_up_time,p_sync_time,n_sync_time]
    #라이더 부
    driver_incomes = []
    driver_dists = []
    driver_saves = []
    for driver_name in drivers:
        driver = drivers[driver_name]
        driver_incomes.append(driver.income)
        dist = 0
        for index in range(1,len(driver.visited_route)):
            bf = driver.visited_route[index - 1][2]
            af = driver.visited_route[index][2]
            dist += distance(bf[0],bf[1],af[0],af[1])
        driver_dists.append([driver.name, driver.income, driver.robot_use, dist])
        driver_saves.append([driver.name, driver.income, len(driver.served),driver.robot_use, driver.visited_route])
    #로봇 부
    robot_dists = []
    robot_saves = []
    for robot_name in robots:
        robot = robots[robot_name]
        dist = 0
        for index in range(1,len(robot.visited_nodes)):
            bf = robot.visited_nodes[index - 1][1]
            af = robot.visited_nodes[index][1]
            dist += distance(bf[0],bf[1],af[0],af[1])
        robot_dists.append([robot.name, dist])
        robot_saves.append([robot.name, robot.visited_nodes])
    save_type = ['/customers/','/drivers/','/robots/']
    save_headers = ['name;gen_t;pickup_t;store_t;store_dep_t;arrive_t;fee;robot_t;r_m_arrive_t;v_m_arrive_t;OD_distance;\n','name;income;served#;robot_use;route;\n','name;route;\n']
    save_res = [customer_saves, driver_saves, robot_saves]
    sub_info = ';R;{};V;{};C;{};Vs;{};'.format(len(robots),len(drivers),len(customers),drivers[0].speed)
    if len(robots) > 0:
        sub_info += 'Rs;{};'.format(robots[0].speed)
    else:
        sub_info += 'Rs;0;'
    tm = time.localtime(time.time())
    tm_info = time.strftime('%Y-%m-%d-%I-%M-%S-%p', tm)
    for save_count in range(3):
        f = open(dir_root + save_type[save_count] + saved_title + tm_info + sub_info + '.txt', 'a')
        f.write(save_headers[save_count])
        for line in save_res[save_count]:
            con = ''
            for info in line:
                con += str(info) + ';'
            con += ';\n'
            f.write(con)
        f.close()
    return customer_res, driver_dists, robot_dists