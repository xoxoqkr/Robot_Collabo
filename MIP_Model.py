# -*- coding: utf-8 -*-
#.py 설명 :
# -> LinearizedASP_gurobi.py : gurobipy를 이용해 보조금 할당 문제를 풀이하는 함수

import gurobipy as gp
from gurobipy import GRB
#import random
#import numpy



def LinearizedCollaboProblem(driver_set, customers_set, robot_set, middle_point_set,ro, v_value,  s_value, zero_infos, solver=-1, print_gurobi = False, timelimit = 1000, add_obj = None):
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
    g = m.addVars(len(drivers), len(customers), vtype=GRB.BINARY, name="g") #y
    #l = m.addVars(len(drivers), len(customers), vtype=GRB.CONTINUOUS, name="l") #z

    #print("Priority Customer", rev_sp)
    # Set objective #Eq(1)
    print('s_value',s_value.shape)
    print(len(h))
    #input
    if len(add_obj) > 0:
        m.setObjective(gp.quicksum((s_value[i, j, m] - add_obj[i,j,m,r])*(h[i, j, m]*z[r,j]) for i in drivers for j in customers for m in m_points for r in robots),GRB.MAXIMIZE)
    else:
        m.setObjective(gp.quicksum(s_value[i,j,m]*h[i,j,m] for i in drivers for j in customers for m in m_points), GRB.MAXIMIZE)
    m.addConstrs(h[i,j,m] <= x[i, j] for i in drivers for j in customers for m in m_points)
    m.addConstrs(h[i, j, m] <= y[j, m] for i in drivers for j in customers for m in m_points)
    m.addConstrs(h[i, j, m] >= x[i, j] + y[j, m] - 1 for i in drivers for j in customers for m in m_points)
    #Eq(2)
    m.addConstrs(gp.quicksum(x[i, j] for i in drivers) <= 1 for j in customers)
    #Eq(3)
    #m.addConstrs(gp.quicksum(x[i, j] for j in customers) == 1 for i in drivers)
    m.addConstrs(gp.quicksum(x[i, j] for j in customers) <= 1 for i in drivers)
    #Eq(4)
    m.addConstrs(gp.quicksum(b[i, j] for j in customers) == ro[i] for i in drivers)
    m.addConstrs(b[i,j] - cso[j] <= driver_num*(1 - x[i,j]) for i in drivers for j in customers)
    m.addConstrs(cso[j] - b[i, j]<= driver_num*(1 - x[i, j]) for i in drivers for j in customers)
    m.addConstrs(b[i, j] <= (driver_num)*x[i,j] for i in drivers for j in customers)
    #Eq(5)
    m.addConstrs(cso[j] <= driver_num + 1 for j in customers)
    #Eq(6)
    m.addConstr(gp.quicksum(cso[j] for j in customers) == sum_i + (driver_num) * (customer_num - driver_num))
    #Eq(7)
    #m.addConstrs(gp.quicksum(x[i, l]*v_value[i,l,m] for l in customers) + large_M*gp.quicksum(x[i, l] for l in customers) + large_M >= v_value[i,j,m]*x[i,j]  for i in drivers for j in customers for m in m_points)
    #m.addConstrs(gp.quicksum(x[i, l]*v_value[i,l,m] for l in customers) - large_M*gp.quicksum(x[i, e] for e in customers) + large_M >= v_value[i,j,m]*g[i,j]  for i in drivers for j in customers for m in m_points)
    m.addConstrs(gp.quicksum(x[i, l]*v_value[i,l,m] - large_M*x[i, l] for l in customers) + large_M >= v_value[i,j,m]*g[i,j]  for i in drivers for j in customers for m in m_points)

    #Eq(8)
    m.addConstrs(gp.quicksum(y[j, m] for m in m_points) <= gp.quicksum(x[i, j] for i in drivers) for j in customers)
    #Eq(9)
    m.addConstrs(gp.quicksum(y[j, m] for m in m_points) == gp.quicksum(z[r, j] for r in robots) for j in customers)
    #Eq(10)
    m.addConstrs(gp.quicksum(z[r, j] for r in robots) <= 1 for j in customers)
    #Eq(11)
    m.addConstrs(gp.quicksum(z[r, j] for j in customers) <= 1 for r in robots)
    #Eq(12) -> input과 결합
    if zero_infos != []:
        for info in zero_infos[0]:
            m.addConstr(x[info[0],info[1]] == 0)
        for info in zero_infos[1]:
            m.addConstr(y[info[0], info[1]] == 0)
        for info in zero_infos[2]:
            m.addConstr(z[info[0], info[1]] == 0)
    #13
    m.addConstrs(cso[j] >= ro[i]*g[i,j] for i in drivers for j in customers)
    #14
    #m.addConstrs(cso[j] <= (ro[i])*(1 - g[i,j]) + (driver_num + 1)*g[i,j] for i in drivers for j in customers)
    #출력 설정
    if print_gurobi == False:
        m.setParam(GRB.Param.OutputFlag, 0)
    m.Params.method = solver  # -1은 auto dedection이며, 1~5에 대한 차이.
    m.setParam("TimeLimit", timelimit)
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

    #res_x = solution_var(m, 'x')
    #print(m.getVarByName('x'))
    #test = [var for var in m.getVars() if "x" in var.VarName]
    #print(test)
    try:
        print('Obj val: %g' % m.objVal, "Solver", solver)
        res_x = solution_var(m,'x')
        res_y = solution_var(m,'y')
        res_z = solution_var(m,'z')
        res_ro = solution_var2(m, 'c')
        return True, [res_x,res_y,res_z,res_ro]
    except:
        #print('Infeasible')
        #res = printer(m.getVars(), [], len(drivers), len(customers))
        return False, []


def LinearizedCollaboProblem2(driver_set, customers_set, robot_set, middle_point_set,ro, v_value,  l_value, zero_infos, solver=-1, print_gurobi = False, timelimit = 1000, simple = False):
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
    x = m.addVars(len(drivers), len(customers), len(robots), vtype=GRB.BINARY, name="x")
    #y = m.addVars(len(customers), len(m_points), vtype=GRB.BINARY, name="y")
    #z = m.addVars(len(robots), len(customers), vtype=GRB.BINARY, name="z")
    cso = m.addVars(len(customers), vtype=GRB.INTEGER, name="c" )
    e = m.addVars(len(drivers), vtype=GRB.BINARY, name="e")
    #선형화를 위한 변수
    #h = m.addVars(len(drivers), len(customers),len(m_points), vtype=GRB.BINARY, name="h")
    b = m.addVars(len(drivers), len(customers), len(robots), vtype=GRB.CONTINUOUS, name="b") #크시
    g = m.addVars(len(drivers), len(customers), vtype=GRB.BINARY, name="g") #y
    #l = m.addVars(len(drivers), len(customers), vtype=GRB.CONTINUOUS, name="l") #z
    #print("Priority Customer", rev_sp)
    # Set objective #Eq(1)
    print('s_value',l_value.shape)
    #print(len(h))
    #input
    m.setObjective(gp.quicksum(l_value[i,j,r]*x[i,j,r] for i in drivers for j in customers for r in robots) + large_M*gp.quicksum(e[i] for i in drivers), GRB.MINIMIZE)

    m.addConstrs(e[i] == 1 - gp.quicksum(x[i, j, r] for j in customers for r in robots) for i in drivers)
    #Eq(2)
    m.addConstrs(gp.quicksum(x[i, j, r] for i in drivers for r in robots) <= 1 for j in customers)
    #Eq(3)
    m.addConstrs(gp.quicksum(x[i, j, r] for j in customers for r in robots) <= 1 for i in drivers)
    # Eq(4)
    #m.addConstrs(gp.quicksum(x[i, j, r]  for i in drivers for j in customers ) <= 1 for r in robots)
    m.addConstrs(gp.quicksum(x[i, j, r] for i in drivers for j in customers) <= 1 for r in robots[1:]) #robot 0는 더미 이므로
    if simple == False:
        #Eq(4) select 순서 제약식
        m.addConstrs(gp.quicksum(b[i, j, r] for j in customers for r in robots ) == ro[i] for i in drivers)
        m.addConstrs(b[i,j,r] - cso[j] <= driver_num*(1 - x[i,j,r]) for i in drivers for j in customers for r in robots)
        m.addConstrs(cso[j] - b[i, j, r]<= driver_num*(1 - x[i, j, r]) for i in drivers for j in customers for r in robots)
        m.addConstrs(b[i, j, r] <= (driver_num)*x[i,j, r] for i in drivers for j in customers for r in robots)
        #Eq(5)
        m.addConstrs(cso[j] <= driver_num + 1 for j in customers)
        #Eq(6)
        m.addConstr(gp.quicksum(cso[j] for j in customers) == sum_i + (driver_num) * (customer_num - driver_num))
        #Eq(7)
        #m.addConstrs(gp.quicksum(x[i, l, o]*v_value[i,l,o] - large_M*x[i, l, o] for l in customers for o in robots) + large_M >= v_value[i,j,r]*g[i,j]  for i in drivers for j in customers for r in robots)
        #Eq(12) -> input과 결합
        if zero_infos != []:
            for info in zero_infos[0]:
                m.addConstr(x[info[0],info[1], info[2]] == 0)
        #13
        m.addConstrs(cso[j] >= ro[i]*g[i,j] for i in drivers for j in customers)
    else:
        pass
    # Eq(7)
    m.addConstrs(gp.quicksum(x[i, l, o] * v_value[i, l, o] - large_M * x[i, l, o] for l in customers for o in robots) + large_M >=v_value[i, j, r] * x[i, j, r] for i in drivers for j in customers for r in robots)

    #14
    #m.addConstrs(cso[j] <= (ro[i])*(1 - g[i,j]) + (driver_num + 1)*g[i,j] for i in drivers for j in customers)
    #출력 설정
    if print_gurobi == False:
        m.setParam(GRB.Param.OutputFlag, 0)
    #m.setParam(GRB.Param.LogFile, 'test.txt')
    m.setParam("LogFile", 'log_test')
    m.Params.method = solver  # -1은 auto dedection이며, 1~5에 대한 차이.
    m.setParam("TimeLimit", timelimit)
    m.optimize()
    try:
        print('Obj val: %g' % m.objVal, "Solver", solver)
        res_x = solution_var(m,'x', length= 3)
        res_ro = solution_var2(m, 'c')
        return True, res_x, res_ro
    except:
        #print('Infeasible')
        #res = printer(m.getVars(), [], len(drivers), len(customers))
        return False, [], []

def PavoneRelocation(c,v,num_z,solver=-1, print_gurobi = True, timelimit = 1000):
    zones = list(range(num_z))
    m = gp.Model("mip1")
    u = m.addVars(c.shape[0], c.shape[1], vtype=GRB.INTEGER, name="u")
    v_d = sum(v)/num_z
    m.setObjective(gp.quicksum(c[i, j] * u[i, j] for i in zones for j in zones), GRB.MINIMIZE)
    m.addConstrs(v[i] + gp.quicksum(u[j, i] - u[i, j] for j in zones) >= v_d for i in zones)
    if print_gurobi == False:
        m.setParam(GRB.Param.OutputFlag, 0)
    m.setParam("LogFile2", 'log_test')
    m.Params.method = solver  # -1은 auto dedection이며, 1~5에 대한 차이.
    m.setParam("TimeLimit", timelimit)
    m.optimize()
    #print('Obj val: %g' % m.objVal, "Solver", solver)
    try:
        print('Obj val: %g' % m.objVal, "Solver", solver)
        res_x = solution_var(m, 'u', length=2)
        return True, res_x
    except:
        # print('Infeasible')
        # res = printer(m.getVars(), [], len(drivers), len(customers))
        return False, []


def PavoneRelocation2(c,w,solver=-1, print_gurobi = True, timelimit = 1000):
    robots = list(range(c.shape[0]))
    stores = list(range(c.shape[1]))
    m = gp.Model("mip1")
    u = m.addVars(c.shape[0], c.shape[1], vtype=GRB.BINARY, name="u")
    m.setObjective(gp.quicksum(c[i, j] * u[i, j] for i in robots for j in stores), GRB.MINIMIZE)
    m.addConstrs(gp.quicksum(u[j, i] for j in robots) >= w[i] for i in stores)
    m.addConstrs(gp.quicksum(u[i, j] for j in stores) <= 1 for i in robots)
    if print_gurobi == False:
        m.setParam(GRB.Param.OutputFlag, 0)
    m.setParam("LogFile2", 'log_test')
    m.Params.method = solver  # -1은 auto dedection이며, 1~5에 대한 차이.
    m.setParam("TimeLimit", timelimit)
    m.optimize()
    #print('Obj val: %g' % m.objVal, "Solver", solver)
    try:
        print('Obj val: %g' % m.objVal, "Solver", solver)
        res_x = solution_var(m, 'u', length=2)
        return True, res_x
    except:
        # print('Infeasible')
        # res = printer(m.getVars(), [], len(drivers), len(customers))
        return False, []

def solution_var(model,name, length = 2):
    res = []
    for variable in [var for var in model.getVars() if name in var.VarName]: #이름이 제대로 풀력 되는지 확인 필요
        #print(variable,variable.VarName,variable.X)
        #input('index 확인')
        if variable.X == 1:
            test = variable.VarName
            if length == 2:
                """
                i, _, j = (
                    test.replace(name, "")
                    .replace("[", "")
                    .replace("]", "")
                    .partition(",")
                )
                """
                i, j = (
                    test.replace(name, "")
                    .replace("[", "")
                    .replace("]", "")
                    .split(",")
                )
                #res.append(count)
                #print(i,_, j, variable.VarName)
                res.append([int(i), int(j)])
            else:
                #print(test)
                #input('체크')
                i, j, k = (
                    test.replace(name, "")
                    .replace("[", "")
                    .replace("]", "")
                    .split(",")
                )
                #res.append(count)
                #print(i,_, j, variable.VarName)
                res.append([int(i), int(j), int(k)])
    return res

def solution_var2(model,name):
    res = []
    for variable in model.getVars():
        if variable.VarName[0] == name:
            res.append(int(variable.x))
    return res