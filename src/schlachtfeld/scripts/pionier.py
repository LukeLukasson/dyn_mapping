#!/usr/bin/env python

import rospy
import sys
import copy
import math

from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose2D

class MapReader:
	
	def __init__(self, topic_name):
		
		# Subscribers
		self.subscriber = rospy.Subscriber(topic_name, OccupancyGrid, self.callback)
		
		# Publishlers
		self.pub_oc_init = rospy.Publisher("/map_init", OccupancyGrid)
		self.pub_oc_static = rospy.Publisher("/map_static", OccupancyGrid)
		self.pub_oc_dynamic = rospy.Publisher("/map_dynamic", OccupancyGrid)
		self.pub_oc_view = rospy.Publisher("/map_view_vizualizer", OccupancyGrid)
		
		# Globals
		self._oc_init = []
		self._oc_static = []
		self._oc_dynamic = []
		self._oc_view = []
		
		# position robot
		self._robo_pos = Pose2D
		self._robo_pos.x = 13
		self._robo_pos.y = 7
		
		# Parameter Simulation
		self._update_rate_velocity = 2
		self._range_view_vert = 5
		self._range_view_horz = 2*self._range_view_vert + 1
		
		# Parameter Algorithm (static)
		self._h_stat = 0.6 		# how fast shall we believe?
		self._l_stat = 0.1		# how fast shall it vanish?
		self._H_stat = self._h_stat/(1-self._h_stat)
		self._L_stat = self._l_stat/(1-self._l_stat)
		# Parameter Algorithm (dynamic)
		self._h_dyn = 0.99		# how fast shall we believe?
		self._l_dyn = 0.2		# how fast shall it vanish?
		self._H_dyn = self._h_dyn/(1-self._h_dyn)
		self._L_dyn = self._l_dyn/(1-self._l_dyn)
		self._lost_threshold = 90
		
	def callback(self, data):
		rospy.loginfo("Initialize Map reading")
		
		oc = data								# read in occupancy grid
		self._mmd = data.info 					# MapMetaData
		
		print "Resolution: ", self._mmd.resolution
		
		self._oc_static = copy.deepcopy(data)	# init static map
		self._oc_static.info.origin.position.x = self._mmd.width + 1
		
		self.init_oc_dynamic(data)
		
		print "Width x Height: %d x %d" % (self._mmd.width, self._mmd.height) 
		
		oc_lst = list(oc.data)
		oc_lst[self.coord_tf(self._robo_pos.x, self._robo_pos.y)] = 75
		
		# color sensor range for robot
		for i in range(self._range_view_horz):
			for j in range(self._range_view_vert):
				if oc_lst[self.coord_tf(self._robo_pos.x-self._range_view_vert+i, self._robo_pos.y+1+j)] != 100:
					oc_lst[self.coord_tf(self._robo_pos.x-self._range_view_vert+i, self._robo_pos.y+1+j)] = 0
					
		oc_tpl_new = tuple(oc_lst)
	
		# initizialize world map
		self._oc_init = oc
		self._oc_init.data = oc_tpl_new
		self._oc_init.info.origin.position.x = 0
		self._oc_init.info.origin.position.y = 0
		
		# initialize view vizualizer
		self._oc_view = copy.deepcopy(oc)
		self._oc_view.data = tuple([0] * (self._range_view_vert*self._range_view_horz))
		self._oc_view.info.origin.position.x = self._robo_pos.x - self._range_view_vert -1
		self._oc_view.info.origin.position.y = self._robo_pos.y
		self._oc_view.info.width = self._range_view_horz
		self._oc_view.info.height = self._range_view_vert

	def talker(self):
		rospy.init_node('talker', anonymous=True)
		r = rospy.Rate(5) # 10hz
		
		# simulate dynamic obstacle
		offset_wall = 5
		pos_dobj = 0		# initial posisiton = pos_dobj + offset_wall
		
		# move person init
		mv = 1
		oc_lst_orig_val = 0
		
		# introduce new static element into the map
		time_new_obs = 25
		tm_hlp = 1
		
		while not rospy.is_shutdown():

			# save environment
			oc_lst = list(self._oc_init.data)

			# trigger time indicator
			if oc_lst[0] == 51:
				oc_lst[0] = 49
			else:
				oc_lst[0] = 51
				
			# introduce new object after certain time
			if tm_hlp >= time_new_obs and tm_hlp < 3*time_new_obs:
				rospy.logwarn("object present")
				oc_lst[self.coord_tf(10, 9)] = 99
				oc_lst[self.coord_tf(10,10)] = 99
				tm_hlp += 1
			elif tm_hlp >= 2*time_new_obs:
				tm_hlp = 0
			else:
				oc_lst[self.coord_tf(10, 9)] = 1
				oc_lst[self.coord_tf(10,10)] = 1
				tm_hlp += 1
						
			# clear dynamic object before publishing new one
			if mv < self._update_rate_velocity:
				oc_lst[self.coord_tf(12,pos_dobj+offset_wall)] = 99
				mv += 1
			else:
				mv = 1																# reset move counter
				oc_lst[self.coord_tf(12,pos_dobj+offset_wall)] = 1					# clear old space
				pos_dobj = (pos_dobj + 1) % (self._mmd.height - offset_wall+1)		# make a step
				oc_lst[self.coord_tf(12,pos_dobj+offset_wall)] = 99					# update map

			# publish updated world map
			oc_tpl_update = tuple(oc_lst)
			self._oc_init.data = oc_tpl_update
			self.pub_oc_init.publish(self._oc_init)
			self.pub_oc_view.publish(self._oc_view)
			
			# update measurements static and dynamic maps
			self._oc_static = self.update_oc_static(self._oc_static)
			self._oc_dynamic = self.update_oc_dynamic(self._oc_static, self._oc_dynamic)
			
			# publish both of them
			self.pub_oc_static.publish(self._oc_static)
			self.pub_oc_dynamic.publish(self._oc_dynamic)

			
			# reset
			r.sleep()


	def coord_tf(self, x, y):
		return (y-1)*self._mmd.width + x - 1
		
	def init_oc_dynamic(self, oc):
		self._oc_dynamic = copy.deepcopy(oc)							# init dynamic map
		self._oc_dynamic.info.origin.position.y = self._mmd.height + 1

		oc_tpl_new = tuple([50] * (self._mmd.height*self._mmd.width))	# init with 0.5 probability (no prior knowledge)

		self._oc_dynamic.data = oc_tpl_new
		
	def update_oc_static(self, oc):
		# 6 cases:
		# S_t-1		o_t		Diff	Equal	Avg		L/H
		# 0			0		0		True	0		L
		# 50		0		50		False	25		L
		# 100		0 		100		False	50		L
		# 0			100		-100	False	50		L2 -> Problem for changing elements in environments
		# 50		100		-50		False	75		H
		# 100		100		0		True	100		H
		
		# try introducing L2
		l2 = 0.52
		L2 = l2/(1-l2)
		
		# get relevant old map for comparison
		oc_lst = list(oc.data)
		loc_old = []							# local occupancy grid to compare to
		for y in range(self._range_view_vert):
			for x in range(self._range_view_horz):
				loc_old.append(oc_lst[self.coord_tf(self._robo_pos.x-self._range_view_vert+x, self._robo_pos.y+1+y)])
		
		# get relevant msr map for comparison
		oc_msr = list(self._oc_init.data)
		loc_msr = []
		for y in range(self._range_view_vert):
			for x in range(self._range_view_horz):
				loc_msr.append(oc_msr[self.coord_tf(self._robo_pos.x-self._range_view_vert+x, self._robo_pos.y+1+y)])
				
		# get average of differencies
		loc_avg = [(i+j)/2 for i,j in zip(loc_old, loc_msr)]
		loc_diff = [i-j for i,j in zip(loc_old, loc_msr)]
		
		# apply model to results
		loc_model_avg = [self._H_stat if x > 55 else self._L_stat for x in loc_avg]
		loc_model_diff = [L2 if x < -87 else self._L_stat for x in loc_diff]		# ATTENTION: is a hack (should be 90)
		
		loc_model = [max(i,j) for i,j in zip(loc_model_avg, loc_model_diff)]		# think about it! see notes -> makes sense
		
		# never fully believe your map -> confidence factor
		conf_factor = 0.95
		loc_prev = [conf_factor*x/(100-conf_factor*x) for x in loc_old]
		
		# finally calculate p( S^t | o^1, ... , o^t, S^(t-1) ) , and represent it as int
		loc_fin = [max(1,math.ceil(100*i*j/(1 + i*j))) for i,j in zip(loc_model, loc_prev)]
		
		# math.ceil instead of round -> little threshold will be discovered!!!

		rospy.loginfo("static oc updated")
		
		# write it back
		i = 0
		for y in range(self._range_view_vert):
			for x in range(self._range_view_horz):
				oc_lst[self.coord_tf(self._robo_pos.x-self._range_view_vert+x, self._robo_pos.y+1+y)] = loc_fin[i]
				i += 1
						
		loc_tpl_new = tuple(oc_lst)
		oc.data = loc_tpl_new
		
		return oc	
		
	def update_oc_dynamic(self, oc, oc_dyn):
		# 6 cases:
		# S_t-1		o_t		Diff	Equal	Avg		L/H
		# 0			0		0		True	0		L
		# 50		0		50		False	25		L
		# 100		0 		100		False	50		L
		# 0			100		-100	False	50		H
		# 50		100		-50		False	75		L
		# 100		100		0		True	100		L
		
		# get relevant old static and dynamic map for comparison
		oc_lst_dyn = list(oc_dyn.data)
		loc_old_dyn = []							# local occupancy grid to compare to
		for y in range(self._range_view_vert):
			for x in range(self._range_view_horz):
				loc_old_dyn.append(oc_lst_dyn[self.coord_tf(self._robo_pos.x-self._range_view_vert+x, self._robo_pos.y+1+y)])

		oc_lst_stat = list(oc.data)
		loc_old_stat = []							# local occupancy grid to compare to
		for y in range(self._range_view_vert):
			for x in range(self._range_view_horz):
				loc_old_stat.append(oc_lst_stat[self.coord_tf(self._robo_pos.x-self._range_view_vert+x, self._robo_pos.y+1+y)])
						
		# get relevant msr map for comparison
		oc_msr = list(self._oc_init.data)
		loc_msr = []
		for y in range(self._range_view_vert):
			for x in range(self._range_view_horz):
				loc_msr.append(oc_msr[self.coord_tf(self._robo_pos.x-self._range_view_vert+x, self._robo_pos.y+1+y)])
				
		# get average of differencies
		loc_diff = [i-j for i,j in zip(loc_old_stat, loc_msr)]
		#print loc_diff
		
		# apply model to results
		loc_model = [self._H_dyn if x < -90 else self._L_dyn for x in loc_diff]
		#print loc_model
		
		# never fully believe your map -> confidence factor
		conf_factor = 0.95
		loc_prev = [conf_factor*x/(100-conf_factor*x) for x in loc_old_dyn]
		#print loc_prev
		
		# finally calculate p( D^t | o^1, ... , o^t, S^(t-1) ) , and represent it as int
		loc_fin = [max(1,round(100*i*j/(1 + i*j))) for i,j in zip(loc_model, loc_prev)]   # <-------------- HACK		
		#print loc_fin
		
		# prevent obstacles from being lost between the maps! keep object in dynamic map until it fully belongs to the static map
		# if in static map (> 5) and falling value in dynamic maps then keep it high! 
		loc_fin = [95 if j-k > 0 and (i > 5 and i < 85) else k for i,j,k in zip(loc_old_stat, loc_old_dyn, loc_fin)]
		#print loc_fin
		print loc_old_stat

		rospy.loginfo("dynamic oc updated")
		# write it back
		i = 0
		for y in range(self._range_view_vert):
			for x in range(self._range_view_horz):
				oc_lst_dyn[self.coord_tf(self._robo_pos.x-self._range_view_vert+x, self._robo_pos.y+1+y)] = loc_fin[i]
				i += 1
										
		loc_tpl_new = tuple(oc_lst_dyn)
		oc_dyn.data = loc_tpl_new
		
		return oc_dyn

if __name__ == '__main__':

	try:
		topic_name = sys.argv[1]
		print "got argument (map required): ", topic_name
		talkerInstance = MapReader(topic_name)
		talkerInstance.talker()
		
	except rospy.ROSInterruptException: pass
