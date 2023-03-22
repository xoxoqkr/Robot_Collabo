# -*- coding: utf-8 -*-
#.py 설명 :
# -> LinearizedASP_gurobi.py : gurobipy를 이용해 보조금 할당 문제를 풀이하는 함수

import random
import simpy
from re_A1_class import Platform_pool
from A1_BasicFunc import GenerateStoreByCSVStressTest, RiderGenerator,Ordergenerator, RobotGenerator
from Func_new import Platform_process6, ResultPrint2, ResultSave2




global ite
global rider_speed
global robot_speed
global robot_num
global rider_num
global dir
global robot_relocate_rule
global run_time
"""
ite = 0
rider_speed = 3
robot_speed = 5
robot_num = 20
rider_num = 10
"""

instance_type = 'Instance_random'
store_num = 50
customer_num = 400
Rider_dict = {}
Orders = {}
Platform2 = Platform_pool()
Store_dict = {}
#run_time = 120
csv_dir = ''
Robot_dict = {}
warm_up_time = 20
solver_time_limit = 1000
root = 'C:/Users/박태준/PycharmProjects/Robot_Collabo/datas/'
# run
env = simpy.Environment()
GenerateStoreByCSVStressTest(env, store_num, Platform2, Store_dict, store_type=instance_type, ITE=ite, dir = root+'store'+dir, customer_pend= False, warm_up_time = warm_up_time)
RobotGenerator(env, Robot_dict, Store_dict, robot_speed = robot_speed, robot_num = robot_num + 1, dir = root+'robot'+dir) #robot 0은 더미 로봇임
#RobotGenerator(env, Robot_dict, Store_dict, robot_speed = robot_speed, robot_num = robot_num, dir = 'store')
env.process(RiderGenerator(env, Rider_dict, Platform2, Store_dict, Orders, Robot_dict, capacity=1,speed=rider_speed, working_duration=120, interval=0.01,gen_num=rider_num, dir = root+'driver'+dir))
env.process(Ordergenerator(env, Orders, Store_dict,  customer_num,  Platform2, rider_speed = rider_speed, lamda= 0.5, warm_up_time = warm_up_time, dir = root+'customer'+dir, task_push= 'store'))
#print('로봇 수',robot_num)
#input('확인')

env.process(Platform_process6(env, Platform2, Orders, Rider_dict, Robot_dict, Store_dict,interval = 5, end_t = run_time, warm_up_time = warm_up_time,
                              solver_time_limit = solver_time_limit, robot_relocate_rule = robot_relocate_rule))

env.run(run_time)
#결과 저장 파트
print(len(Store_dict),len(Robot_dict),len(Orders))
#InstanceSave(Store_dict, Orders, Rider_dict, Robot_dict, title_info='',ite = ite, root = 'C:/Users/박태준/PycharmProjects/Robot_Collabo/datas/')


ResultSave2(Orders, Rider_dict, Robot_dict, saved_title = 'ite;'+str(ite) + ';r_rule;' + robot_relocate_rule + ';')
res_c, res_d, res_r = ResultPrint2(Orders, Rider_dict, Robot_dict)



