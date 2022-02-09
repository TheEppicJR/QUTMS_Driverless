from pyomo.environ import *
from pyomo.dae import *
import pyomo.contrib.fbbt.interval as interval

import matplotlib.pyplot as plt
from matplotlib import animation

import datetime
import numpy as np
import math, time

def fit_track_poly(elements, degree):
	x = []
	y = []
	for i in elements:
		x.append(i["x"])
		y.append(i["y"])
	fit = np.polynomial.polynomial.Polynomial.fit(x, y, degree)
	fit = fit.convert(domain=[-1, 1])
	return fit

def list_of_dict_to_list(list_of_dict, key):
	result = []
	for dict_item in list_of_dict:
		result.append(dict_item[key])
	return result



class MPCSolver:
	def __init__(self) -> None:
		self.solver = SolverFactory('ipopt')
		self.solver.options['tol'] = 1e-4 # was origionally to the -4
		self.solver.options['warm_start_init_point'] = 'yes'
		self.solver.options['warm_start_bound_push'] = 1e-2
		self.solver.options['warm_start_mult_bound_push'] = 1e-2
		self.solver.options['expect_infeasible_problem'] = 'no' # don't care if its infeasible
		self.solver.options['mu_init'] = 1e-2
		
		self.world_space_x_list = []
		self.world_space_y_list = []

		self.not_running = True

		self.epsilon = 1e-2

		self.mass = 0.041
		self.inertia_z = 27.8e-6
		self.l_f = 0.04 # wheel base length
		self.l_r = 0.07

		# tire model constants
		self.B_f = 3.38
		self.C_f = 1.2
		self.D_f = 0.192

		self.B_r = 4.0
		self.C_r = 1.9
		self.D_r = 0.175

		# electric drivetrain and friction constants
		self.C_r0 = 0.0518
		self.C_r2 = 0.00035
		self.C_m1 = 0.287 * 2
		self.C_m2 = 0.054

		self.init_track_constraints = []
		self.track_constraints = []

		self.mpc_time = 3
		self.mpc_samples = 24

		# do even more init i put in another func to mak it more readable
		self.initconst()


	#fuck if i know what any of this does
	def initconst(self):
		
		self.m = ConcreteModel()
		self.m.t = ContinuousSet(bounds=(0, self.mpc_time))

		self.m.D = Var(self.m.t) # throttle
		self.m.delta = Var(self.m.t) # steering angle
		self.m.D_dot = DerivativeVar(self.m.D)
		self.m.delta_dot = DerivativeVar(self.m.delta)

		self.m.constr_D_dot_upper = Constraint(self.m.t, rule=lambda m, t: (self.m.D_dot[t]) <= 3)
		self.m.constr_D_dot_lower = Constraint(self.m.t, rule=lambda m, t: (self.m.D_dot[t]) >= -3)
		self.m.constr_delta_dot_upper = Constraint(self.m.t, rule=lambda m, t: self.m.delta_dot[t] <= math.radians(180))
		self.m.constr_delta_dot_lower = Constraint(self.m.t, rule=lambda m, t: self.m.delta_dot[t] >= -math.radians(180))

		self.m.obstacle_penalty = Var(self.m.t)
		self.m.obstacle_caution_penalty = Var(self.m.t, initialize = 0)

		self.test_Var = 0
		self.world_space_x = 0
		self.world_space_y = 0
		self.world_space_phi = 0

		self.m.x = Var(self.m.t, initialize=0.0)
		self.m.y = Var(self.m.t, initialize=0.0)
		self.m.phi = Var(self.m.t) # heading
		self.m.v_x = Var(self.m.t)
		self.m.v_y = Var(self.m.t)
		self.m.omega = DerivativeVar(self.m.phi) # angular vel

		self.m.x_dot = DerivativeVar(self.m.x)
		self.m.y_dot = DerivativeVar(self.m.y)
		self.m.v_x_dot = DerivativeVar(self.m.v_x)
		self.m.v_y_dot = DerivativeVar(self.m.v_y)
		self.m.omega_dot = DerivativeVar(self.m.omega)

		self.m.F_y_f = Var(self.m.t)
		self.m.F_y_r = Var(self.m.t)
		self.m.F_x_r = Var(self.m.t)
		self.m.alpha_f = Var(self.m.t)
		self.m.alpha_r = Var(self.m.t)

		self.m.left_track_poly_c0 = Param(mutable=True, default = 0)
		self.m.left_track_poly_c1 = Param(mutable=True, default = 0)
		self.m.left_track_poly_c2 = Param(mutable=True, default = 0)
		self.m.left_track_poly_c3 = Param(mutable=True, default = 0)
		self.m.left_track_poly_c4 = Param(mutable=True, default = 0)
		self.m.right_track_poly_c0 = Param(mutable=True, default = 0)
		self.m.right_track_poly_c1 = Param(mutable=True, default = 0)
		self.m.right_track_poly_c2 = Param(mutable=True, default = 0)
		self.m.right_track_poly_c3 = Param(mutable=True, default = 0)
		self.m.right_track_poly_c4 = Param(mutable=True, default = 0)

		self.m.left_track_constr = Constraint(self.m.t, rule=lambda m, t: self.m.y[t] - self.m.obstacle_penalty[t] <= self.m.left_track_poly_c0 + self.m.left_track_poly_c1*self.m.x[t] + self.m.left_track_poly_c2*self.m.x[t]*self.m.x[t] + self.m.left_track_poly_c3*self.m.x[t]*self.m.x[t]*self.m.x[t] + self.m.left_track_poly_c4*self.m.x[t]*self.m.x[t]*self.m.x[t]*self.m.x[t] - self.m.obstacle_caution_penalty[t])  
		self.m.right_track_constr = Constraint(self.m.t, rule=lambda m, t: self.m.y[t] + self.m.obstacle_penalty[t] >= self.m.right_track_poly_c0 + self.m.right_track_poly_c1*self.m.x[t] + self.m.right_track_poly_c2*self.m.x[t]*self.m.x[t] + self.m.right_track_poly_c3*self.m.x[t]*self.m.x[t]*self.m.x[t] + self.m.right_track_poly_c4*self.m.x[t]*self.m.x[t]*self.m.x[t]*self.m.x[t] + self.m.obstacle_caution_penalty[t])

		self.m.x_target = Param(mutable=True, default = 0)
		self.m.y_target = Param(mutable=True, default = 0)

		self.m.D_init = Param(mutable=True, default = 0)
		self.m.delta_init = Param(mutable=True, default = 0)
		self.m.x_init = Param(mutable=True, default = 0)
		self.m.y_init = Param(mutable=True, default = 0)
		self.m.phi_init = Param(mutable=True, default = 0)
		self.m.v_x_init = Param(mutable=True, default = 0)
		self.m.v_y_init = Param(mutable=True, default = 0)
		self.m.omega_init = Param(mutable=True, default = 0)

		self.m.pc = ConstraintList()
		self.m.pc.add(self.m.D[0]==self.m.D_init)
		self.m.pc.add(self.m.delta[0]==self.m.delta_init)
		self.m.pc.add(self.m.x[0]==self.m.x_init)
		self.m.pc.add(self.m.y[0]==self.m.y_init)
		self.m.pc.add(self.m.phi[0]==self.m.phi_init)
		self.m.pc.add(self.m.v_x[0]==self.m.v_x_init)
		self.m.pc.add(self.m.v_y[0]==self.m.v_y_init)
		self.m.pc.add(self.m.omega[0]==self.m.omega_init)


		self.m.constr_0 = Constraint(self.m.t, rule=lambda m, t: self.m.obstacle_caution_penalty[t] >= 0)
		self.m.constr_1 = Constraint(self.m.t, rule=lambda m, t: self.m.v_x[t] >= -0.1)
		self.m.constr_2 = Constraint(self.m.t, rule=lambda m, t: self.m.D[t] <= 1.0)
		self.m.constr_3 = Constraint(self.m.t, rule=lambda m, t: self.m.D[t] >= -2)
		self.m.constr_4 = Constraint(self.m.t, rule=lambda m, t: self.m.delta[t] <= math.radians(90))
		self.m.constr_5 = Constraint(self.m.t, rule=lambda m, t: self.m.delta[t] >= -math.radians(90))

		# kinematic constraints
		self.m.x_dot_ode = Constraint(self.m.t, rule=lambda m, t: 
			self.m.x_dot[t] == self.m.v_x[t] * cos(self.m.phi[t]) - self.m.v_y[t] * sin(self.m.phi[t])
		)
		self.m.y_dot_ode = Constraint(self.m.t, rule=lambda m, t: 
			self.m.y_dot[t] == self.m.v_x[t] * sin(self.m.phi[t]) + self.m.v_y[t] * cos(self.m.phi[t])
		)
		# dynamics constraints
		self.m.v_x_dot_ode = Constraint(self.m.t, rule=lambda m, t: 
			self.m.v_x_dot[t] == 1/self.mass * (self.m.F_x_r[t] - self.m.F_y_f[t]*sin(self.m.delta[t]) + self.mass*self.m.v_y[t]*self.m.omega[t])
		)
		self.m.v_y_dot_ode = Constraint(self.m.t, rule=lambda m, t: 
			self.m.v_y_dot[t] == 1/self.mass * (self.m.F_y_r[t] + self.m.F_y_f[t]*cos(self.m.delta[t]) - self.mass*self.m.v_x[t]*self.m.omega[t])
		)
		self.m.omega_dot_ode = Constraint(self.m.t, rule=lambda m, t: 
			self.m.omega_dot[t] == 1/self.inertia_z * (self.m.F_y_f[t]*self.l_f*cos(self.m.delta[t]) - self.m.F_y_r[t]*self.l_r)
		)
		# tire / drivetrain dynamics constraints
		self.m.F_y_f_ode = Constraint(self.m.t, rule=lambda m, t: 
			self.m.F_y_f[t] == self.D_f*sin(self.C_f*atan(self.B_f*self.m.alpha_f[t]))
		)
		self.m.F_y_r_ode = Constraint(self.m.t, rule=lambda m, t: 
			self.m.F_y_r[t] == self.D_r*sin(self.C_r*atan(self.B_r*self.m.alpha_r[t]))
		)
		self.m.F_x_r_ode = Constraint(self.m.t, rule=lambda m, t: 
			self.m.F_x_r[t] == (self.C_m1 - self.C_m2*self.m.v_x[t])*self.m.D[t] - self.C_r0 - self.C_r2*pow(self.m.v_x[t], 2)
		)
		self.m.alpha_f_ode = Constraint(self.m.t, rule=lambda m, t: 
			self.m.alpha_f[t] == -atan((self.m.omega[t]*self.l_f + self.m.v_y[t]) / (self.m.v_x[t] + self.epsilon)) + self.m.delta[t]
		)
		self.m.alpha_r_ode = Constraint(self.m.t, rule=lambda m, t: 
			self.m.alpha_r[t] == atan((self.m.omega[t]*self.l_r - self.m.v_y[t]) / (self.m.v_x[t] + self.epsilon))
		)

		#TransformationFactory('dae.finite_difference').apply_to(m, wrt=self.m.t, nfe=mpc_samples)
		TransformationFactory('dae.collocation').apply_to(self.m, wrt=self.m.t, nfe=self.mpc_samples, ncp=1, scheme='LAGRANGE-RADAU')


		self.m.integral = Integral(self.m.t, wrt=self.m.t, rule=self.cost_function_integral)
		self.m.obj = Objective(expr=self.m.integral)

	def generate_track_polys(self, car_pos, number_of_points, backwards_points, mult):
		track_elements = self.retrieve_track_segment(car_pos, number_of_points, backwards_points, mult)
		left_poly = fit_track_poly(list_of_dict_to_list(track_elements, 'left'), 4)
		right_poly = fit_track_poly(list_of_dict_to_list(track_elements, 'right'), 4)
		return left_poly, right_poly, track_elements

	def cost_function_integral(self, m, t):
		return 0.25*(self.m.x[t] - self.m.x_target)**2 + 0.25*(self.m.y[t] - self.m.y_target)**2 + 900*(self.m.obstacle_penalty[t])**2 + 200*(self.m.obstacle_caution_penalty[t]-1)**2 + -10*(self.m.v_x[t])**2 #+ 1*(self.m.delta[t])**2


	def transform_track(self, car_pos):
		phi = car_pos["phi"]
		for i in range(len(self.init_track_constraints)):
			for element in self.init_track_constraints[i]:
				diffx = self.init_track_constraints[i][element]["x"] - car_pos["x"]
				diffy = self.init_track_constraints[i][element]["y"] - car_pos["y"]
				self.track_constraints[i][element] = {"x":diffx * cos(phi) + diffy * sin(phi), "y":diffy * cos(phi) - diffx * sin(phi)}


	def append_track(self, left_point, center_point, right_point):
		self.track_constraints.append({})
		self.init_track_constraints.append({"left" : left_point, "center" : center_point, "right" : right_point})

	def retrieve_track_segment(self, car_pos, number_of_points, backwards_points, mult):
		result = []
		closest_dist = 99999999
		closest_idx = 0
		for i in range(len(self.track_constraints)):
			track_element = self.track_constraints[i]
			dist = sqrt((car_pos["x"] - track_element["center"]["x"])**2 + 1*(car_pos["y"] - track_element["center"]["y"])**2)
			if dist < closest_dist:
				closest_dist = dist
				closest_idx = i
		for i in range(number_of_points):
			idx = (closest_idx + i*mult - backwards_points*mult) % len(self.track_constraints)
			result.append(self.track_constraints[idx])

		return result
	
	def prepfigs(self):
		self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3,1)

		# xy, left_poly, right_poly, left_track, right_track, center, actual_left_Track, actual_right_track
		self.lines = [(self.ax1.plot([], [], linestyle='--', marker='x', color='g')[0]), (self.ax1.plot([], [], lw=2)[0]), (self.ax1.plot([], [], lw=2)[0]), (self.ax1.plot([], [], '--bo')[0]), (self.ax1.plot([], [], '--bo')[0]), (self.ax1.plot([], [], linestyle='None', marker='x', color='pink')[0]), (self.ax1.plot([], [], linestyle='--', color = 'silver')[0]), (self.ax1.plot([], [], linestyle='--', color = 'silver')[0]), 
		(self.ax2.plot([], [], linestyle='-', color = 'black')[0]), (self.ax2.plot([], [], linestyle=':', color = 'red')[0]), (self.ax2.plot([], [], linestyle=':', color = 'blue')[0]), (self.ax2.plot([], [], linestyle='-', color = 'limegreen')[0]), (self.ax1.plot([], [], linestyle='--', color = 'red')[0])]
		#ax1.plot([0,10], [0,0], linestyle='--', color = 'black')
	
		self.lines_len_init =len(self.lines)-1

		self.plot_list = {}
		self.plot_params = ['delta', 'D', 'v_x', 'v_y']

		axis_lims_inner = 8
		axis_lims_outer = 100
		axis_lims_outeru = 20
		axis_lims_outerx = 60
		self.ax1.set_xlim(-axis_lims_inner, axis_lims_inner)
		self.ax1.set_ylim(-axis_lims_inner, axis_lims_inner)
		self.ax1.set_aspect('equal', 'box')
		self.ax2.set_xlim(-axis_lims_outerx, axis_lims_outerx)
		self.ax2.set_ylim(-axis_lims_outeru, axis_lims_outer)
		self.ax2.set_aspect('equal', 'box')
		self.ax3.set_xlim(0, self.mpc_time)
		self.ax3.set_ylim(-2, 2)
		self.ax3.set_aspect(0.1)


	def set_world_space(self, x, y, phi, v_x, v_y, omega):
		self.world_space_x = x
		self.world_space_y = y
		self.world_space_phi = phi
		#self.m.D_init = 
		#self.m.delta_init = 
		self.m.v_x_init = v_x
		self.m.v_y_init = v_y
		self.m.omega_init = omega

	def solve(self, solve_idx):
		self.transform_track({"x": self.world_space_x, "y": self.world_space_y, "phi": self.world_space_phi})

		left_track_poly, right_track_poly, track_elements = self.generate_track_polys({"x":self.m.x[0](), "y":self.m.y[0]()}, 6, 1, 8)
		target_track_element = track_elements[-1]["center"]
		self.m.x_target = target_track_element["x"]
		self.m.y_target = target_track_element["y"]

		self.m.left_track_poly_c0 = left_track_poly.coef[0]
		self.m.left_track_poly_c1 = left_track_poly.coef[1]
		self.m.left_track_poly_c2 = left_track_poly.coef[2]
		self.m.left_track_poly_c3 = left_track_poly.coef[3]
		self.m.left_track_poly_c4 = left_track_poly.coef[4]

		self.m.right_track_poly_c0 = right_track_poly.coef[0]
		self.m.right_track_poly_c1 = right_track_poly.coef[1]
		self.m.right_track_poly_c2 = right_track_poly.coef[2]
		self.m.right_track_poly_c3 = right_track_poly.coef[3]
		self.m.right_track_poly_c4 = right_track_poly.coef[4]

		track_x_space = np.linspace(self.m.x[0]()-15,self.m.x[0]()+15,300)
		self.lines[1].set_data(track_x_space, left_track_poly(track_x_space))
		self.lines[2].set_data(track_x_space, right_track_poly(track_x_space))
		self.lines[3].set_data(list_of_dict_to_list(list_of_dict_to_list(track_elements, 'left'), 'x'), list_of_dict_to_list(list_of_dict_to_list(track_elements, 'left'), 'y'))
		self.lines[4].set_data(list_of_dict_to_list(list_of_dict_to_list(track_elements, 'right'), 'x'), list_of_dict_to_list(list_of_dict_to_list(track_elements, 'right'), 'y'))
		self.lines[5].set_data(list_of_dict_to_list(list_of_dict_to_list(track_elements, 'center'), 'x'), list_of_dict_to_list(list_of_dict_to_list(track_elements, 'center'), 'y'))
		self.lines[6].set_data(list_of_dict_to_list(list_of_dict_to_list(self.track_constraints, 'left'), 'x'), list_of_dict_to_list(list_of_dict_to_list(self.track_constraints, 'left'), 'y'))
		self.lines[7].set_data(list_of_dict_to_list(list_of_dict_to_list(self.track_constraints, 'right'), 'x'), list_of_dict_to_list(list_of_dict_to_list(self.track_constraints, 'right'), 'y'))
		self.lines[8].set_data(self.world_space_x_list, self.world_space_y_list)
		self.lines[9].set_data(list_of_dict_to_list(list_of_dict_to_list(self.init_track_constraints, 'left'), 'x'), list_of_dict_to_list(list_of_dict_to_list(self.init_track_constraints, 'left'), 'y'))
		self.lines[10].set_data(list_of_dict_to_list(list_of_dict_to_list(self.init_track_constraints, 'right'), 'x'), list_of_dict_to_list(list_of_dict_to_list(self.init_track_constraints, 'right'), 'y'))
		self.lines[11].set_data([[self.world_space_x + -1*cos(self.world_space_phi), self.world_space_x + cos(self.world_space_phi)], [self.world_space_y + -1*sin(self.world_space_phi), self.world_space_y + sin(self.world_space_phi)]])
		self.lines[12].set_data([self.m.v_x_init(),0], [self.m.v_y_init(),0])

		if len(self.world_space_x_list) > 100:
			for i in range(10):
				self.world_space_x_list.pop(0)
				self.world_space_y_list.pop(0)

		start: float = time.time()
		try:
			self.solver.solve(self.m, tee=False)
		except:
			return [1, 2, 3, 4, 5, 6], [1, 2, 3, 4, 5, 6]
		#print(1/(time.time() - self.last_solve_time))

		print(f"Time taken MCP: {time.time()-start}")
		start: float = time.time()
		init_next_idx = self.m.t[2]

		world_space_phi = self.m.phi[init_next_idx]() + self.world_space_phi
		world_space_x = self.world_space_x + (self.m.x[init_next_idx]() * cos(world_space_phi) + self.m.y[init_next_idx]() * sin(world_space_phi))
		world_space_y = self.world_space_y + (self.m.y[init_next_idx]() * cos(world_space_phi) + self.m.x[init_next_idx]() * sin(world_space_phi))
		self.world_space_x_list.append(world_space_x)
		self.world_space_y_list.append(world_space_y)

		self.m.D_init = self.m.D[init_next_idx]()
		self.m.delta_init = self.m.delta[init_next_idx]()
		self.m.v_x_init = self.m.v_x[init_next_idx]()
		self.m.v_y_init = self.m.v_y[init_next_idx]()
		self.m.omega_init = self.m.omega[init_next_idx]()

		t_array = np.array([t for t in self.m.t]).tolist()
		x_array = np.array([self.m.x[t]() for t in self.m.t]).tolist()
		y_array = np.array([self.m.y[t]() for t in self.m.t]).tolist()

		self.lines[0].set_data(x_array, y_array)

		for element in self.plot_params:
			uniq_mul = 1
			if element == 'v_x':
				uniq_mul = 0.2
			elif element == 'delta':
				uniq_mul = math.degrees(0.5)
			self.plot_list[element] = np.array([getattr(self.m, element)[t]()*uniq_mul for t in self.m.t]).tolist()
		line_idx = self.lines_len_init
		for key in self.plot_list:
			if len(self.lines)-1 <= line_idx:
				new_line, = self.ax3.plot([], [], label=key, linestyle='--')
				self.lines.append(new_line)
			line_idx = line_idx + 1
			self.lines[line_idx].set_data(t_array,self.plot_list[key])

		plt.legend(loc='right')
		
		print(f"Time taken post MCP: {time.time()-start}")
		return x_array, y_array

	def plot(self):
		return self.fig