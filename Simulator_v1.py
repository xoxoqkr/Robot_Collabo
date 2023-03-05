# -*- coding: utf-8 -*-
#.py 설명 :
# -> LinearizedASP_gurobi.py : gurobipy를 이용해 보조금 할당 문제를 풀이하는 함수

import random
import simpy
from re_A1_class import Platform_pool, robot
from A1_BasicFunc import GenerateStoreByCSVStressTest, RiderGenerator,Ordergenerator
from Func_new import Platform_process5
instance_type = 'Instance_random'
ite = 0
store_num = 50
customer_num = 400
rider_speed = 3
robot_speed = 2
Rider_dict = {}
Orders = {}
Platform2 = Platform_pool()
Store_dict = {}
run_time = 120
csv_dir = ''
Robot_dict = {}

# run
env = simpy.Environment()
GenerateStoreByCSVStressTest(env, store_num, Platform2, Store_dict, store_type=instance_type, ITE=ite)
for i in range(20):
    store = Store_dict[random.choice(Store_dict).name]
    Robot_dict[i] = robot(env, i, speed = robot_speed, init_loc= store.location)

env.process(RiderGenerator(env, Rider_dict, Platform2, Store_dict, Orders, capacity=1,speed=rider_speed, working_duration=120, interval=0.01,gen_num=50))
env.process(Ordergenerator(env, Orders, Store_dict,  customer_num,  Platform2, rider_speed = rider_speed))
env.process(Platform_process5(env, Platform2, Orders, Rider_dict, Robot_dict, interval = 5, end_t = run_time))
env.run(run_time)