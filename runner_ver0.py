# -*- coding: utf-8 -*-
##'Instance_random','Instance_cluster'
import time
run_time = 120
ITE_Start = 0
ITE_End = 1
driver_speeds = [2.5]
robot_speeds = [1.5,2]
robot_nums = [31]
driver_nums = [21]
robot_relocate_rules = ['pavone']#['dist','#ct','pareto','random','None'] #['dist']#['dist','#ct','pareto','random','None']
ins_types = ['Songpa','Dongjak']
scenarios  = []


"""
for ins in ins_types:
    for i in driver_speeds:
        for l in driver_nums:
            for j in robot_speeds:
                if ins == 'Sogpa':
                    if j == 1.5:
                        r_speed = 
                    elif j == 2:
                else:
                #robot_nums = [31]
                #for _ in range(1,2):
                #    robot_nums.append(l + 5*_)
                for k in robot_nums:
                    #if l < k <= l + 9:
                    for u in robot_relocate_rules:
                        scenarios.append([i,j,k,l,u,ins])
"""

#송파 만들기
for i in [1.875]: #driver_speeds:
    for l in driver_nums:
        for j in [1.25,1.56]: #robot_speeds:
            for k in robot_nums:
                for u in robot_relocate_rules:
                    scenarios.append([i,j,k,l,u,'Songpa'])
                    if k == 0:
                        break
#동작 만들기
"""
for i in [2.14]: #driver_speeds:
    for l in driver_nums:
        for j in [1.42,1.78]: #robot_speeds:
            for k in robot_nums:
                for u in robot_relocate_rules:
                    scenarios.append([i,j,k,l,u,'Dongjak'])
                    if k == 0:
                        break
"""
for i in scenarios:
    print(i)
input('확인')

init = 0
print(scenarios)
print(len(scenarios)*(ITE_End))
input('개수 확인')
for sc_info in scenarios:
    for ite in range(ITE_Start, ITE_End):
        start_time = time.time()
        dir = '_instance__ite_' + str(ite) + '.txt'
        exec(open('Simulator_v1.py', encoding='UTF8').read(),
             globals().update(rider_speed = sc_info[0], robot_speed= sc_info[1],
                              robot_num = sc_info[2],rider_num = sc_info[3], ite = ite, dir = dir,
                              robot_relocate_rule = sc_info[4], ins_type = sc_info[5] , run_time = run_time))
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