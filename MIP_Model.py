# -*- coding: utf-8 -*-
#.py 설명 :
# -> LinearizedASP_gurobi.py : gurobipy를 이용해 보조금 할당 문제를 풀이하는 함수

import gurobipy as gp
from gurobipy import GRB
#import random
#import numpy



def LinearizedCollaboProblem(driver_set, customers_set, robot_set, middle_point_set,ro, v_value,  s_value, zero_infos, solver=-1, print_gurobi = False):
    """
    선형화된 버전의 보조금 문제
    :param driver_set: 가능한 라이더 수
    :param customers_set: 가능한 고객 수
    :param value:
    :param ro: 라이더 선택 순서
    :param print_gurobi: Gurobi 실행 내용 print문
    :param solver: gurobi의 solver engine 선택 [-1,0,1,2]
    :return: 해
    """
    drivers = list(range(len(driver_set)))
    customers = list(range(len(customers_set)))
    robots = list(range(len(robot_set)))
    m_points = list(range(len(middle_point_set)))
    driver_num = len(driver_set)
    customer_num = len(customers_set)
    sum_i = sum(ro)
    large_M = 10000
    #print('parameters',drivers,customers, ro, driver_num, customer_num, sum_i)
    # D.V. and model set.
    print(len(drivers),len(customers),len(m_points),len(robots))
    m = gp.Model("mip1")
    x = m.addVars(len(drivers), len(customers), vtype=GRB.BINARY, name="x")
    y = m.addVars(len(customers), len(m_points), vtype=GRB.BINARY, name="y")
    z = m.addVars(len(robots), len(customers), vtype=GRB.BINARY, name="z")
    cso = m.addVars(len(customers), vtype=GRB.INTEGER, name="c" )
    #선형화를 위한 변수
    h = m.addVars(len(drivers), len(customers),len(m_points), vtype=GRB.BINARY, name="h")
    b = m.addVars(len(drivers), len(customers), vtype=GRB.CONTINUOUS, name="b") #크시
    #print("Priority Customer", rev_sp)
    # Set objective #Eq(1)
    print('s_value',s_value.shape)
    print(len(h))
    #input('확인')
    m.setObjective(gp.quicksum(s_value[i,j,m]*h[i,j,m] for i in drivers for j in customers for m in m_points), GRB.MAXIMIZE)
    m.addConstrs(h[i,j,m] <= x[i, j] for i in drivers for j in customers for m in m_points)
    m.addConstrs(h[i, j, m] <= y[j, m] for i in drivers for j in customers for m in m_points)
    m.addConstrs(h[i, j, m] >= x[i, j] + y[j, m] - 1 for i in drivers for j in customers for m in m_points)
    #Eq(2)
    m.addConstrs(gp.quicksum(x[i, j] for i in drivers) <= 1 for j in customers)
    #Eq(3)
    m.addConstrs(gp.quicksum(x[i, j] for j in customers) <= 1 for i in drivers)
    #Eq(4)
    m.addConstrs(gp.quicksum(b[i, j] for j in customers) == ro[i] for i in drivers)
    m.addConstrs(b[i,j] - cso[j] <= driver_num*(1 - x[i,j]) for i in drivers for j in customers)
    m.addConstrs(cso[j] - b[i, j]<= driver_num*(1 - x[i, j]) for i in drivers for j in customers)
    m.addConstrs(b[i, j] <= (driver_num)*x[i,j] for i in drivers for j in customers)
    #Eq(5)
    m.addConstrs(cso[j] <= driver_num for j in customers)
    #Eq(6)
    m.addConstr(gp.quicksum(cso[j] for j in customers) == sum_i + (driver_num) * (customer_num - driver_num))
    #Eq(7)
    m.addConstrs(gp.quicksum(x[i, l]*v_value[i,l,m] for l in customers) + large_M*gp.quicksum(x[i, l] for l in customers) + large_M >= v_value[i,j,m]*x[i,j]  for i in drivers for j in customers for m in m_points)
    #Eq(8)
    m.addConstrs(gp.quicksum(y[j, m] for m in m_points) <= gp.quicksum(x[i, j] for i in drivers) for j in customers)
    #Eq(9)
    m.addConstrs(gp.quicksum(y[j, m] for m in m_points) == gp.quicksum(z[r, j] for r in robots) for j in customers)
    #Eq(10)
    m.addConstrs(gp.quicksum(z[r, j] for r in robots) <= 1 for j in customers)
    #Eq(11) -> input과 결합
    for info in zero_infos[0]:
        m.addConstr(x[info[0],info[1]] == 0)
    for info in zero_infos[1]:
        m.addConstr(y[info[0], info[1]] == 0)
    for info in zero_infos[2]:
        m.addConstr(z[info[0], info[1]] == 0)
    #출력 설정
    if print_gurobi == False:
        m.setParam(GRB.Param.OutputFlag, 0)
    m.Params.method = solver  # -1은 auto dedection이며, 1~5에 대한 차이.
    m.optimize()
    """
    res = ASP.printer(m.getVars(), [], len(drivers), len(customers))
    print('Obj val: %g' % m.objVal, "Solver", solver)
    c_list = []
    x_list = []
    y_list = []
    for val in m.getVars():
        if val.VarName[0] == 'c':
            c_list.append(int(val.x))
        elif val.VarName[0] == 'x':
            x_list.append(int(val.x))
        elif val.VarName[0] == 'y':
            y_list.append(int(val.x))
        else:
            pass
    print("CSO")
    print(c_list)
    c_list.sort()
    print(c_list)
    x_list = np.array(x_list)
    x_list = x_list.reshape(driver_num, customer_num)
    print("X")
    print(x_list)
    print("Y")
    y_list = np.array(y_list)
    y_list = y_list.reshape(driver_num, customer_num)
    print(y_list)
    """
    try:
        print('Obj val: %g' % m.objVal, "Solver", solver)
        res_x = solution_var(m,'x')
        res_y = solution_var(m,'y')
        res_z = solution_var(m,'z')
        return True, [res_x,res_y,res_z]
    except:
        #print('Infeasible')
        #res = printer(m.getVars(), [], len(drivers), len(customers))
        return False, []

def solution_var(model,name):
    res = []
    count = 0
    for i in model.getVarByName(name): #이름이 제대로 풀력 되는지 확인 필요
        if i.X == 1:
            res.append(count)
        count += 1
    return res