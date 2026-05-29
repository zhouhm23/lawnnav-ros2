#!/usr/bin/python3
# 路径覆盖算法进度
# 原理：要覆盖的区域由一个多边形给出，其点从RViz使用“Publish Point”设置。当连续点等于第一个点时，多边形区域通过一种类似于Boustrophedon细胞分解[1]的算法划分为多个单元格，因此每个单元格都可以通过简单的往返运动来覆盖。然后将生成的目标点交给导航堆栈。
# 进度：实现rviz规划多边形，实现细胞分解, 实现规划路径，实现发布导航点
# 问题：路径覆盖的移动过程不够连贯，修正次数过多，覆盖率不够高
# TODO
# rviz里规划要包含障碍物才能运动
#export ROS_DOMAIN_ID=0
import sys
import os
import numpy as np
import pdb
import json
import tempfile
from shapely.geometry import Polygon, Point # pip3 install shapely
from shapely.ops import unary_union
from math import *
import time
import subprocess # sudo apt-get install ruby-full
import yaml

from scipy.spatial.transform import Rotation

from path_coverage.list_helper import * #from itertools import pairwise
from path_coverage.trapezoidal_coverage import calc_path as trapezoid_calc_path
from path_coverage.border_drive import border_calc_path

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point as rosPoint

import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import ReliabilityPolicy, QoSProfile
from ament_index_python.packages import get_package_share_directory

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose, ComputePathToPose

from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from action_msgs.msg import GoalStatus

from path_coverage.checkpoint import (
    save_checkpoint, load_checkpoint, clear_checkpoint,
    register_signal_handlers, CHECKPOINT_FILE,
)

# GoalTracker: use share directory (works in both src and install)
_share_dir = get_package_share_directory('path_coverage')
_scripts_test_dir = os.path.join(_share_dir, 'scripts', 'test')
if _scripts_test_dir not in sys.path:
    sys.path.insert(0, _scripts_test_dir)
from goal_tracker import GoalTracker


# Keep for backward compatibility; execution-stage cost filtering now uses
# `costmap_max_non_lethal` directly to match decomposition-stage logic.
INSCRIBED_INFLATED_OBSTACLE = 253





class MapDrive(Node): 
	def __init__(self):
		super().__init__('map_drive') 

		rate = 50
		self.declare_parameter('hz', rate)
		hz_param = self.get_parameter('hz').get_parameter_value().integer_value
		self.looprate = self.create_rate(hz_param)

		self.sub_node = rclpy.create_node('path_verifier_client')
		self.sub_node.get_logger().info('Created path_verifier_client node')

		# 使用 sub_node 以便在回调中 spin
		self.navigate_to_pose_client = ActionClient(self.sub_node, NavigateToPose, 'navigate_to_pose')
		self.navigate_through_poses_client = ActionClient(self.sub_node, NavigateThroughPoses, 'navigate_through_poses')
		self.compute_path_to_pose_client = ActionClient(self.sub_node, ComputePathToPose, 'compute_path_to_pose')
		self.goal_msg = ComputePathToPose.Goal()
		self.x = None
		self.y = None
		
		self.rospack  = get_package_share_directory('path_coverage') 

		# 位置输出文件（主路径 + 回退路径）
		self.filename = "/home/ubuntu/ros2_ws/src/path_coverage_ros2/params/pose_output.yaml"
		self.fallback_filename = "/tmp/pose_output.yaml"

		# 初始化
		self.pose_output = {}
		self.last_points = {}
		self.last_path = None
		self.lClickPoints = []
		self.local_costmap = None
		self.global_costmap = None
		self.goal_handle = None
		self.result_future = None
		
		self.declare_parameter("global_frame", "map")
		self.declare_parameter("robot_width", 0.171) 
		self.declare_parameter("costmap_max_non_lethal", 70)
		self.declare_parameter("boustrophedon_decomposition", True)
		self.declare_parameter("border_drive", False)
		self.declare_parameter("base_frame", "base_footprint")
		self.declare_parameter("num_points", 1) 
		self.declare_parameter("min_wp_dist", 0.2) 
		# 规划区域按用户多边形掩膜裁剪后，可额外向外扩张的距离（米）
		# 用于补偿栅格化边缘损失；设为 0 则严格按用户画的区域。
		self.declare_parameter("polygon_expand", 0.05)
		# 覆盖路径相对边界的内缩距离（米）：0 表示尽量贴边。
		self.declare_parameter("coverage_clearance", 0.0)
		# 外扩圈（polygon_expand）内允许的最大代价值。
		# 默认 0：仅允许白色自由区，避免扩张进入灰色膨胀层。
		self.declare_parameter("expand_max_non_lethal", 0)
		# 执行阶段（局部代价地图）允许的最大代价值。
		# 默认 0：导航目标尽量不落在灰色膨胀层。
		self.declare_parameter("drive_max_non_lethal", 0)
		# Visualization control
		self.declare_parameter("show_all_cells", False)
		self.declare_parameter("show_paths", True)
		# Static map masking (optional)
		self.declare_parameter("use_static_map_mask", False)
		self.declare_parameter("static_map_occupied_thresh", 65)

		self.global_frame = self.get_parameter("global_frame").get_parameter_value().string_value 
		self.robot_width = self.get_parameter("robot_width").get_parameter_value().double_value
		self.costmap_max_non_lethal = self.get_parameter("costmap_max_non_lethal").get_parameter_value().integer_value
		self.boustrophedon_decomposition = self.get_parameter("boustrophedon_decomposition").get_parameter_value().bool_value #  False #
		self.border_drive = self.get_parameter("border_drive").get_parameter_value().bool_value
		self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
		self.num_points = self.get_parameter("num_points").get_parameter_value().integer_value
		self.min_wp_dist = self.get_parameter("min_wp_dist").get_parameter_value().double_value
		self.polygon_expand = self.get_parameter("polygon_expand").get_parameter_value().double_value
		self.coverage_clearance = self.get_parameter("coverage_clearance").get_parameter_value().double_value
		self.expand_max_non_lethal = self.get_parameter("expand_max_non_lethal").get_parameter_value().integer_value
		self.drive_max_non_lethal = self.get_parameter("drive_max_non_lethal").get_parameter_value().integer_value
		self.show_all_cells = self.get_parameter("show_all_cells").get_parameter_value().bool_value
		self.show_paths = self.get_parameter("show_paths").get_parameter_value().bool_value
		self.use_static_map_mask = self.get_parameter("use_static_map_mask").get_parameter_value().bool_value
		self.static_map_occupied_thresh = self.get_parameter("static_map_occupied_thresh").get_parameter_value().integer_value
		self.static_map = None

		self.create_subscription(PointStamped, "/clicked_point", self.rvizPointReceived, 1)
		# self.global_map_sub = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.map_callback, QoSProfile(depth=300, reliability=ReliabilityPolicy.BEST_EFFORT))
		self.create_subscription(OccupancyGrid, f"/global_costmap/costmap", self.globalCostmapReceived, 10) 
		self.create_subscription(OccupancyGrid, f"/local_costmap/costmap", self.localCostmapReceived, 10)  
		self.create_subscription(OccupancyGrid, "/map", self.staticMapReceived, 1)

		self.pub_marker = self.create_publisher(Marker, 'path_coverage_marker', 16) 
		
		self.tfBuffer = Buffer()
		self.tf_listener = TransformListener(self.tfBuffer, self)

		self.get_logger().info('parameters::::global_frame::robot_width::costmap_max_non_lethal::boustrophedon_decomposition::border_drive::base_frame::num_points::min_wp_dist.')
		self.get_logger().info('::::::::::::::'+str(self.global_frame)+'::'+str(self.robot_width)+'::'+str(self.costmap_max_non_lethal)+'::'+str(self.boustrophedon_decomposition)+'::'+str(self.border_drive)+'::'+str(self.base_frame)+'::'+str(self.num_points)+'::'+str(self.min_wp_dist)+'::polygon_expand='+str(self.polygon_expand)+'::coverage_clearance='+str(self.coverage_clearance)+'::expand_max_non_lethal='+str(self.expand_max_non_lethal)+'::drive_max_non_lethal='+str(self.drive_max_non_lethal)+'::show_all_cells='+str(self.show_all_cells)+'::show_paths='+str(self.show_paths)+'::use_static_map_mask='+str(self.use_static_map_mask)+'::static_map_occupied_thresh='+str(self.static_map_occupied_thresh)+'.')
		# Checkpoint: register signal handlers for crash recovery
		register_signal_handlers()
		self._checkpoint_data = load_checkpoint()
		self._current_cell_idx = 0
		self._total_cells = 0
		if self._checkpoint_data:
			self.get_logger().info(
				f"[CHECKPOINT] Found resume checkpoint: "
				f"cell {self._checkpoint_data.get('cell_idx',0)}/{self._checkpoint_data.get('total_cells',0)}, "
				f"segment {self._checkpoint_data.get('segment_idx',0)}"
			)

		self.get_logger().info("Path coverage node initialized successfully...")

		# Heartbeat timer for process liveness monitoring (debug mid-run hangs)
		self._heartbeat_timer = self.create_timer(15.0, self._heartbeat_callback)
		self._coverage_start_time = None
		self._cover_state = "idle"

		# 目标不可达率追踪
		self._goal_tracker = GoalTracker(algo_type="improved")





	def _heartbeat_callback(self):
		"""Periodic liveness heartbeat for debugging mid-run hangs."""
		if self._coverage_start_time is not None:
			import time as _time
			elapsed = _time.time() - self._coverage_start_time
			self._cover_state = f"covering (elapsed={elapsed:.0f}s)"
		self.get_logger().info(f"[HEARTBEAT] node alive, state={self._cover_state}")





	def localCostmapReceived(self, costmap):
		# self.get_logger().info('local costmap received')
		self.local_costmap = costmap
		self.local_costmap_width = costmap.info.width*costmap.info.resolution
		self.local_costmap_height = costmap.info.height*costmap.info.resolution





	def globalCostmapReceived(self, costmap):
		# self.get_logger().info('global costmap received')
		self.global_costmap = costmap

	def staticMapReceived(self, grid):
		self.static_map = grid


		

	
	def visualization_cleanup(self):
		for id, points in self.last_points.items():
			if points is not None:
				self.visualize_trapezoid(points, id=id, show=False)
			self.last_points = {}
		if self.last_path is not None:
			self.visualize_path(self.last_path, False)
			self.last_path = None






	def visualize_cell(self, points, show=True, close=True):
		self.visualize_trapezoid(points, show, close)

	def visualize_cells_all(self, polys, id_offset=1000):
		"""Publish all cell boundaries at once (no paths)."""
		self.last_points = {}
		for i, poly in enumerate(polys):
			try:
				points = list(poly.exterior.coords)
			except Exception:
				continue
			cell_id = id_offset + i
			self.last_points[cell_id] = points
			self.visualize_trapezoid(points, show=True, close=True, id=cell_id)





	def visualize_area(self, points, show=True, close=True):
		self.visualize_trapezoid(points, show, close, id=1, red=1.0, blue=0.0)





	def visualize_trapezoid(self, points, show=True, close=True, id=0, red=0.0, green=0.0, blue=1.0):
		#self.get_logger().info("viz_trapezoid- 01")# -------------------------------------
		if len(points) < 2: 
			return
		#self.get_logger().info("viz_trapezoid- 02") # -------------------------------------
		self.last_points[id] = points if show else None
		#self.get_logger().info("viz_trapezoid- 03: ") # -------------------------------------
		msg = Marker()
		msg.header.frame_id = self.global_frame
		msg.header.stamp = self.get_clock().now().to_msg() # rospy.Time.now()
		msg.ns = "trapezoid"
		# msg.lifetime = Duration(seconds=0).to_msg() # rospy.Duration(0)
		msg.id = id
		msg.type = Marker.LINE_STRIP
		msg.action = Marker.ADD if show else Marker.DELETE
		msg.pose.orientation.w = float(1)
		msg.pose.orientation.x = float(0)
		msg.pose.orientation.y = float(0)
		msg.pose.orientation.z = float(0)
		msg.scale.x = 0.02
		# blue
		msg.color.r = red
		msg.color.g = green
		msg.color.b = blue
		msg.color.a = 1.0
		#self.get_logger().info("viz_trapezoid- 04: ") # -------------------------------------

		if close:
			points = points + [points[0]]
		#	self.get_logger().info("viz_trapezoid- 05: ") # -------------------------------------
		for point in points:
			point_msg = rosPoint()
			point_msg.x = point[0]
			point_msg.y = point[1]
			msg.points.append(point_msg)
		#	self.get_logger().info("viz_trapezoid- 06: ") # -------------------------------------

		self.pub_marker.publish(msg)
		#self.get_logger().info("viz_trapezoid- 07: ") # -------------------------------------
		time.sleep(0.3)
		self.get_logger().info("viz_trapezoid completed...") # -------------------------------------






	def visualize_path(self, path, show=True):
		i = 0
		self.get_logger().info("visualize_path- 001: ") # -------------------------------------
		self.last_path = path if show else None
		#self.get_logger().info("visualize_path- 002: ") # -------------------------------------
		for pos_last,pos_cur in pairwise(path):
		#	self.get_logger().info("visualize_path- 003: ") # -------------------------------------
			msg = Marker()
			msg.header.frame_id = self.global_frame
			msg.header.stamp =  self.get_clock().now().to_msg() # rospy.Time.now()
			msg.ns = "path"
			# msg.lifetime = Duration(seconds=2).to_msg() # erases the line after 2 secs.
			msg.id = i
			msg.type = Marker.ARROW
			msg.action = Marker.ADD if show else Marker.DELETE
			msg.pose.orientation.w = float(1)
			msg.pose.orientation.x = float(0)
			msg.pose.orientation.y = float(0)
			msg.pose.orientation.z = float(0)
			msg.scale.x = 0.01 # shaft diameter
			msg.scale.y = 0.03 # head diameter
			# green
			msg.color.g = 1.0
			msg.color.a = 1.0
		#	self.get_logger().info("visualize_path- 004: ") # -------------------------------------

			point_msg_start = rosPoint()
			point_msg_start.x = pos_last[0]
			point_msg_start.y = pos_last[1]
			msg.points.append(point_msg_start)
			point_msg_end = rosPoint()
			point_msg_end.x = pos_cur[0]
			point_msg_end.y = pos_cur[1]
			msg.points.append(point_msg_end)
		#	self.get_logger().info("visualize_path- 005: ") # -------------------------------------

			i+=1
		#	self.get_logger().info("visualize_path- 006: ") # -------------------------------------
			self.pub_marker.publish(msg)
			time.sleep(0.3)
		self.get_logger().info("visualize_path completed...") # -------------------------------------




	



	def rvizPointReceived(self, point):
		# print('i heard: ', point)
		self.lClickPoints.append(point)
		points = [(p.point.x, p.point.y) for p in self.lClickPoints]
		self.global_frame = point.header.frame_id
		# print('len(self.lClickPoints): ', len(self.lClickPoints))
		if len(self.lClickPoints) > 2:
			# All points must have same frame_id
			if len(set([p.header.frame_id for p in self.lClickPoints])) != 1:
				raise ValueError()
			points_x = [p.point.x for p in self.lClickPoints]
			points_y = [p.point.y for p in self.lClickPoints]
			avg_x_dist = list_avg_dist(points_x)
			avg_y_dist = list_avg_dist(points_y)
			dist_x_first_last = abs(points_x[0] - points_x[-1])
			dist_y_first_last = abs(points_y[0] - points_y[-1])
			if dist_x_first_last < avg_x_dist/10.0 and dist_y_first_last < avg_y_dist/10.0:
				# last point is close to maximum, construct polygon
				self.get_logger().info("Creating polygon %s" % (str(points)))
				#self.get_logger().info("i got here 0")
				self.visualize_area(points, close=True)
				#self.get_logger().info("i got here 00")
				if self.boustrophedon_decomposition:
					self.get_logger().info("do_boustrophedon initiated...")
					# Wait for global costmap to be available before planning
					while self.global_costmap is None and rclpy.ok():
						self.get_logger().info("Waiting for global costmap to become available...")
						rclpy.spin_once(self, timeout_sec=1.0)
					if self.global_costmap:
						self.do_boustrophedon(Polygon(points), self.global_costmap)
					else:
						self.get_logger().error("Global costmap not available, cannot perform Boustrophedon decomposition.")
				else:
					self.get_logger().info("drive_polygon initiated...")
					self.drive_polygon(Polygon(points))
				self.visualize_area(points, close=True, show=False)
				self.lClickPoints = []
				# this signifies the end afterwhich everything is cleaned.
				self.get_logger().info("writing the data to the YAML file...")
				# empty the pose_output dict
				# Write the data to yaml with fallback for readonly workspace cases
				self._write_pose_output_yaml()
				self.pose_output = {}	
				# 路径覆盖完成，结束线程
				self.get_logger().info("this signifies the end afterwhich everything is cleaned")
				self.destroy_node() # Stop the node, which will cause rclpy.spin() to exit
				return
			self.get_logger().info("i got here 6:")
		self.visualize_area(points, close=False)
		self.get_logger().info("finished successfully rvizPointReceived func.")



	def make_Polygons_shapely_polygons(self, Polygons):
		polygons = []
		polygon_area = []
		for polygon in Polygons:
			coords = [(x, y) for x, y in polygon]
			shapely_polygon = Polygon(coords)
			area = shapely_polygon.area
			polygons.append(shapely_polygon)
			polygon_area.append(area)
		return polygons, polygon_area


	def are_polygons_connected(self, poly1, poly2, threshold=2): 
		p1 = Polygon(poly1)
		p2 = Polygon(poly2)
		for c1 in p1.exterior.coords:
			for c2 in p2.exterior.coords:
				if math.sqrt((c1[0]-c2[0])**2 + (c1[1]-c2[1])**2) <= threshold:
					# print("c1,c2 :", c1,c2)
					return True
		return False



	def are_polygons_connected_with_increased_thresh(self, poly1, poly2):
		return self.are_polygons_connected(poly1, poly2, threshold=4)


	def find_connected_polygons(self, Polygons, polygon_area_threshold=0.0):
		# Keep all decomposed cells. The previous graph-ordering logic could drop
		# disconnected cells, which caused "only the first area gets covered".
		if not Polygons:
			return []

		filtered_polygons = []
		for polygon in Polygons:
			try:
				shp = Polygon([(x, y) for x, y in polygon])
			except Exception:
				continue
			if shp.is_empty:
				continue
			if shp.area > polygon_area_threshold:
				filtered_polygons.append(polygon)

		self.get_logger().info(f"Boustrophedon cells kept: {len(filtered_polygons)}/{len(Polygons)}")
		return filtered_polygons

	def _as_polygons(self, geom, min_area=0.0):
		"""Return all usable Polygon parts from arbitrary shapely geometry."""
		if geom is None:
			return []
		try:
			# Try to repair invalid geometry first.
			geom_fixed = geom.buffer(0)
		except Exception:
			geom_fixed = geom

		polys = []
		if getattr(geom_fixed, "geom_type", "") == "Polygon":
			if not geom_fixed.is_empty and geom_fixed.area > min_area:
				polys.append(geom_fixed)
			return polys

		if hasattr(geom_fixed, "geoms"):
			for g in geom_fixed.geoms:
				if getattr(g, "geom_type", "") == "Polygon" and (not g.is_empty) and g.area > min_area:
					polys.append(g)

		return polys

	def _as_single_polygon(self, geom, min_area=0.0):
		"""Return one usable Polygon (largest area) from arbitrary geometry."""
		polys = self._as_polygons(geom, min_area=min_area)
		if not polys:
			return None
		return max(polys, key=lambda p: p.area)

	def _normalize_polygon(self, geom, min_area=0.0, context=""):
		"""Normalize arbitrary geometry into a single Polygon or None."""
		if geom is None:
			return None
		poly = self._as_single_polygon(geom, min_area=min_area)
		if poly is None:
			self.get_logger().warn(f"{context} geometry is not a polygon, skip.")
			return None
		if poly.is_empty:
			self.get_logger().warn(f"{context} geometry is empty, skip.")
			return None
		return poly

	def _write_pose_output_yaml(self):
		"""Write pose output yaml with fallback when preferred path is not writable."""
		payload = dict(self.pose_output)
		payload["updatetime"] = time.time_ns()

		targets = [self.filename]
		if self.fallback_filename not in targets:
			targets.append(self.fallback_filename)

		last_err = None
		for target in targets:
			try:
				target_dir = os.path.dirname(target)
				if target_dir:
					os.makedirs(target_dir, exist_ok=True)
				with open(target, "w") as f:
					yaml.dump(payload, f)
				if target != self.filename:
					self.get_logger().warn(
						f"Primary pose output path not writable, wrote fallback: {target}"
					)
				return target
			except OSError as e:
				last_err = e
				self.get_logger().warn(f"Failed writing pose output to {target}: {e}")

		if last_err is not None:
			raise last_err















	def do_boustrophedon(self, poly, costmap):
		# Check if costmap is valid
		if costmap is None:
			self.get_logger().error("Boustrophedon decomposition failed: costmap is None.")
			return

		# 修复可能的自交多边形，避免掩膜判断异常。
		try:
			poly_core = poly.buffer(0)
			if hasattr(poly_core, "geoms") and poly_core.geom_type == "MultiPolygon":
				poly_core = max(poly_core.geoms, key=lambda g: g.area)
		except Exception:
			poly_core = poly

		poly_mask = poly_core

		# 允许在无障碍条件下按参数轻微外扩，避免边缘漏割。
		if self.polygon_expand > 0.0:
			try:
				poly_mask = poly_mask.buffer(self.polygon_expand, join_style=2)
			except Exception:
				pass

		# Cut polygon area from costmap
		self.get_logger().info("do_boustrophedon() 1: ") # -------------------------------------
		(minx, miny, maxx, maxy) = poly_mask.bounds
		self.get_logger().info("do_boustrophedon() 2: ") # -------------------------------------
		#self.get_logger().info("Converting costmap at x=%.2f..%.2f, y=%.2f..%.2f for Boustrophedon Decomposition" % (minx, maxx, miny, maxy))
		#self.get_logger().info("3: ")
		#self.get_logger().info("3: ", minx, miny, maxx, maxy) # -------------------------------------
		#self.get_logger().info("3.5: ", costmap.info.origin.position.x, costmap.info.origin.position.y, costmap.info.resolution) # -------------------------------------
		#self.get_logger().info(" ")
		# Convert to costmap coordinate using floor/ceil to avoid empty ranges
		from math import floor, ceil
		minx_idx = int(max(0, floor((minx - costmap.info.origin.position.x) / costmap.info.resolution)))
		maxx_idx = int(min(costmap.info.width - 1, ceil((maxx - costmap.info.origin.position.x) / costmap.info.resolution)))
		miny_idx = int(max(0, floor((miny - costmap.info.origin.position.y) /costmap.info.resolution)))
		maxy_idx = int(min(costmap.info.height - 1, ceil((maxy - costmap.info.origin.position.y) / costmap.info.resolution)))
		self.get_logger().info(f"do_boustrophedon costmap idx bounds: x {minx_idx}..{maxx_idx}, y {miny_idx}..{maxy_idx}")
		self.get_logger().info("do_boustrophedon() 4: ") # -------------------------------------
		# Check empty region
		if maxx_idx < minx_idx or maxy_idx < miny_idx:
			self.get_logger().warn("Converted costmap bounds are empty -> no cells to decompose")
			return
		self.get_logger().info("do_boustrophedon() 5: ") # -------------------------------------
		# Transform costmap values to values expected by boustrophedon_decomposition script
		rows = []
		# Two-stage mask construction:
		# 1) build core/expand candidates by cost threshold
		# 2) keep only cells connected to the core area (audit for expansion leakage)
		cell_meta = []
		for ix in range(minx_idx, maxx_idx + 1):
			column_meta = []
			for iy in range(miny_idx, maxy_idx + 1):
				x = (ix + 0.5) * costmap.info.resolution + costmap.info.origin.position.x
				y = (iy + 0.5) * costmap.info.resolution + costmap.info.origin.position.y
				pt = Point([x, y])

				if not poly_mask.covers(pt):
					column_meta.append((False, False, False))
					continue

				# Optional static map mask to keep obstacles consistent with /map
				if self.use_static_map_mask and self.static_map is not None:
					mx = int((x - self.static_map.info.origin.position.x) / self.static_map.info.resolution)
					my = int((y - self.static_map.info.origin.position.y) / self.static_map.info.resolution)
					if 0 <= mx < self.static_map.info.width and 0 <= my < self.static_map.info.height:
						mval = self.static_map.data[int(my * self.static_map.info.width + mx)]
						if mval < 0 or mval >= self.static_map_occupied_thresh:
							column_meta.append((True, False, False))
							continue

				data = costmap.data[int(iy * costmap.info.width + ix)]
				if data == -1:
					# Unknown 区域不作为可通行，避免外扩到未知区域
					column_meta.append((True, False, False))
					continue

				in_core = poly_core.covers(pt)
				core_ok = in_core and (data <= self.costmap_max_non_lethal)
				expand_ok = (not in_core) and (data <= self.expand_max_non_lethal)
				column_meta.append((True, core_ok, expand_ok))
			cell_meta.append(column_meta)

		from collections import deque
		w = len(cell_meta)
		h = len(cell_meta[0]) if w > 0 else 0
		visited = [[False for _ in range(h)] for _ in range(w)]
		q = deque()

		seed_count = 0
		for x in range(w):
			for y in range(h):
				_, core_ok, _ = cell_meta[x][y]
				if core_ok:
					visited[x][y] = True
					q.append((x, y))
					seed_count += 1

		if seed_count == 0:
			self.get_logger().warn("No traversable core cells found inside polygon; skip decomposition")
			return

		# 8-neighborhood preserves connectivity across diagonals in rasterized boundaries.
		dirs = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
		while q:
			x, y = q.popleft()
			for dx, dy in dirs:
				nx, ny = x + dx, y + dy
				if nx < 0 or ny < 0 or nx >= w or ny >= h:
					continue
				if visited[nx][ny]:
					continue
				in_mask, core_ok, expand_ok = cell_meta[nx][ny]
				if (not in_mask) or (not (core_ok or expand_ok)):
					continue
				visited[nx][ny] = True
				q.append((nx, ny))

		kept = 0
		for x in range(w):
			column = []
			for y in range(h):
				if visited[x][y]:
					column.append(-1)
					kept += 1
				else:
					column.append(0)
			rows.append(column)

		# Build a geometry mask of blocked cells (inside polygon mask but not traversable)
		# and clip decomposed cell polygons against it to avoid any black/inflation intrusion.
		blocked_cell_polys = []
		for x in range(w):
			for y in range(h):
				in_mask, core_ok, expand_ok = cell_meta[x][y]
				if not in_mask:
					continue
				if visited[x][y] or core_ok or expand_ok:
					continue
				ix = minx_idx + x
				iy = miny_idx + y
				x0 = ix * costmap.info.resolution + costmap.info.origin.position.x
				y0 = iy * costmap.info.resolution + costmap.info.origin.position.y
				x1 = (ix + 1) * costmap.info.resolution + costmap.info.origin.position.x
				y1 = (iy + 1) * costmap.info.resolution + costmap.info.origin.position.y
				blocked_cell_polys.append(Polygon([(x0, y0), (x1, y0), (x1, y1), (x0, y1)]))

		blocked_union = unary_union(blocked_cell_polys) if blocked_cell_polys else None

		self.get_logger().info(f"Expansion audit kept traversable cells: {kept}, seeds={seed_count}")


		#print("7: ") # -------------------------------------
		# self.get_logger().info("=..................=") 

		polygons = []
		# dump rows for debugging and allow manual inspection / ruby run
		with tempfile.NamedTemporaryFile(delete=False,mode='w', suffix='.json') as ftmp:
			ftmp.write(json.dumps(rows))
			ftmp.flush()
			self.get_logger().info(f"Saved decomposition input to {ftmp.name}")

			boustrophedon_script = os.path.join(self.rospack, "scripts/boustrophedon_decomposition.rb")

			try:
				result = subprocess.run(["ruby", boustrophedon_script, ftmp.name], capture_output=True, text=True)
				polygons = json.loads(result.stdout)
			except subprocess.CalledProcessError as e:
				self.get_logger().error(f"Boustrophedon script error: {e}")
				self.get_logger().error(f"stderr: {e.stderr if hasattr(e, 'stderr') else ''}")
			except Exception as e:
				self.get_logger().error(f"Failed to run/parsing boustrophedon script: {e}")


		#self.get_logger().info("10: ") # -------------------------------------
		print("----- Polygons: " +str(polygons)+".")

		ordered_polygons = self.find_connected_polygons(polygons)

		# 预先转换细胞到地图坐标并计算质心。
		cells = []
		cell_polys = []
		for cell in ordered_polygons:
			points = [
				(
					(point[0] + minx_idx) * costmap.info.resolution + costmap.info.origin.position.x,
					(point[1] + miny_idx) * costmap.info.resolution + costmap.info.origin.position.y,
				)
				for point in cell
			]
			try:
				poly_cell = Polygon(points)
			except Exception:
				continue
			if blocked_union is not None and not poly_cell.is_empty:
				try:
					poly_cell = poly_cell.difference(blocked_union)
				except Exception:
					pass
			for poly_part in self._as_polygons(poly_cell, min_area=0.0):
				c = poly_part.centroid
				cells.append({"poly": poly_part, "centroid": (c.x, c.y)})
				cell_polys.append(poly_part)

		if self.show_all_cells:
			self.visualize_cells_all(cell_polys)

		def _get_ref_pose_xy():
			if self.x is not None and self.y is not None:
				return (self.x, self.y)
			try:
				cur_tf = self.tfBuffer.lookup_transform(self.global_frame, self.base_frame, rclpy.time.Time())
				return (cur_tf.transform.translation.x, cur_tf.transform.translation.y)
			except (TransformException, LookupException, ConnectivityException, ExtrapolationException):
				return None

		# 最近邻迭代排序：每覆盖完一个细胞后重新选择下一个最近细胞。
		remaining = list(cells)
		total_cells = len(remaining)
		self._total_cells = total_cells

		# Resume logic: skip already-completed cells from checkpoint
		resume_cell_idx = 0
		resume_segment_idx = 0
		if self._checkpoint_data and self._checkpoint_data.get("total_cells") == total_cells:
			resume_cell_idx = self._checkpoint_data.get("cell_idx", 0)
			resume_segment_idx = self._checkpoint_data.get("segment_idx", 0)
			if resume_cell_idx > 0:
				self.get_logger().info(
					f"[RESUME] Skipping {resume_cell_idx} completed cells, "
					f"resuming from cell {resume_cell_idx + 1}"
				)
		# We can't skip in nearest-neighbor order; skip by removing from remaining
		# but nearest-neighbor reorders — simpler: just don't skip, let re-cover happen.
		# For segment-level resume, store the segment offset.
		self._resume_segment_idx = resume_segment_idx if resume_cell_idx == 0 else 0

		exec_idx = 1
		while remaining:
			ref = _get_ref_pose_xy()
			if ref is None:
				# 无法获取当前位姿时，退化为输入顺序
				next_i = 0
			else:
				rx, ry = ref
				next_i = min(
					range(len(remaining)),
					key=lambda i: (remaining[i]["centroid"][0] - rx) ** 2 + (remaining[i]["centroid"][1] - ry) ** 2,
				)

			next_cell = remaining.pop(next_i)
			self._current_cell_idx = exec_idx
			self.get_logger().info(
				f"Execute cell {exec_idx}/{total_cells}, centroid=({next_cell['centroid'][0]:.2f}, {next_cell['centroid'][1]:.2f})"
			)
			if self._coverage_start_time is None:
				import time as _time
				self._coverage_start_time = _time.time()
				self._cover_state = "covering"
			cell_ok = False
			for attempt in range(2):
				try:
					self.drive_polygon(next_cell["poly"])
					cell_ok = True
					break
				except Exception as e:
					if attempt == 0:
						self.get_logger().warn(
							f"Cell {exec_idx} attempt 1 failed: {e}. Waiting 3s before retry..."
						)
						time.sleep(3)
					else:
						self.get_logger().error(
							f"Cell {exec_idx} failed after 2 attempts, skipping. error={e}"
						)
			if not cell_ok:
				self.get_logger().warn(
					f"Cell {exec_idx}/{total_cells} skipped, coverage may be incomplete."
				)
				# Recovery: navigate to cell centroid to re-stabilize localization
				try:
					cx_next = next_cell.get("centroid", (0.0, 0.0))
					self.get_logger().info(
						f"Recovery: navigating to centroid ({cx_next[0]:.2f},{cx_next[1]:.2f}) "
						"to re-stabilize..."
					)
					import time as _time
					rec_ok = self.navigate_to_pose(cx_next[0], cx_next[1], 0.0)
					if not rec_ok:
						self.get_logger().warn("Recovery navigation cancelled or timed out.")
					_time.sleep(2.0)
				except Exception as rec_e:
					self.get_logger().warn(f"Recovery navigation failed: {rec_e}")
			exec_idx += 1
		# Clear checkpoint on normal completion
		clear_checkpoint()
		self.get_logger().info("[CHECKPOINT] Coverage completed, checkpoint cleared.")
		#self.get_logger().info("12: ") # -------------------------------------
		self._coverage_start_time = None
		self._cover_state = "idle"
		self.get_logger().info("Boustrophedon Decomposition completed...")
		self._goal_tracker.finish()







	def euler_to_quaternion(self, yaw, pitch, roll):
		qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
		qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
		qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
		qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
		return [qx, qy, qz, qw]











	def _getPathImpl(self, start, goal, planner_id='', use_start=False):

		# self.sub_node.get_logger().debug("-------------------- 'ComputePathToPose' --------------------")
		while not self.compute_path_to_pose_client.wait_for_server(timeout_sec=1.0):
			self.sub_node.get_logger().info("'ComputePathToPose' action server not available, waiting...")

		self.goal_msg.start = start
		self.goal_msg.goal = goal
		self.goal_msg.planner_id = planner_id
		self.goal_msg.use_start = use_start

		#self.sub_node.get_logger().info('Getting path...1')
		send_goal_future = self.compute_path_to_pose_client.send_goal_async(self.goal_msg)
		#self.sub_node.get_logger().info('Getting path...2')
		rclpy.spin_until_future_complete(self.sub_node, send_goal_future)
		#self.sub_node.get_logger().info('Getting path...3')
		self.goal_handle = send_goal_future.result()
		#self.sub_node.get_logger().info('Getting path...4')

		if not self.goal_handle.accepted:
			self.get_logger().error('Get path was rejected!')
			return None
		#self.sub_node.get_logger().info('Getting path...5')

		self.result_future = self.goal_handle.get_result_async()
		#self.sub_node.get_logger().info('Getting path...6')
		rclpy.spin_until_future_complete(self.sub_node, self.result_future)
		#self.sub_node.get_logger().info('Getting path...7')
		self.status = self.result_future.result().status
		#self.sub_node.get_logger().info('Getting path...8')
		if self.status != GoalStatus.STATUS_SUCCEEDED:
			self.sub_node.get_logger().warn(f'Getting path failed with status code: {self.status}')
			return None
		self.sub_node.get_logger().info("Get path completed...") 
		return self.result_future.result().result









	def getPath(self, start, goal, planner_id='', use_start=False):
		"""Send a `ComputePathToPose` action request."""
		rtn = self._getPathImpl(start, goal, planner_id='', use_start=False)
		if not rtn:
			return None
		else:
			return rtn.path








	def get_closest_possible_goal(self, pos_last, pos_next, angle, tolerance):

		'''  
		plan.pose = geometry_msgs.msg.PoseStamped(
			header=std_msgs.msg.Header(
				stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), 
				frame_id=''), 
			pose=geometry_msgs.msg.Pose(
				position=geometry_msgs.msg.Point(x=6.100000272691251, y=-4.599999736249448, z=0.0), 
				orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)))	
		'''

		angle_quat = self.euler_to_quaternion(angle,0,0)  # tf.transformations.quaternion_from_euler(0, 0, angle)

		start = PoseStamped()
		start.header.frame_id = self.global_frame # 'map'
		start.header.stamp = self.get_clock().now().to_msg()
		start.pose.position.x = pos_last[0]
		start.pose.position.y = pos_last[1]
		start.pose.orientation.x = angle_quat[0]
		start.pose.orientation.y = angle_quat[1]
		start.pose.orientation.z = angle_quat[2]
		start.pose.orientation.w = angle_quat[3]

		goal = PoseStamped()
		goal.header.frame_id = self.global_frame # 'map'
		goal.header.stamp = self.get_clock().now().to_msg()
		goal.pose.position.x = pos_next[0]
		goal.pose.position.y = pos_next[1]
		goal.pose.orientation.x = angle_quat[0]
		goal.pose.orientation.y = angle_quat[1]
		goal.pose.orientation.z = angle_quat[2]
		goal.pose.orientation.w = angle_quat[3]

		# sanity check for a valid path
		plan = self.getPath(start, goal) # plan = self.move_base_plan(start, goal, tolerance).plan

		if plan == None:
			return None
		
		if len(plan.poses) == 0:
			return None
		
		#pdb.set_trace()
		closest = None

		for pose in plan.poses:
			pose.header.stamp = self.get_clock().now().to_msg() # rospy.Time(0) # time for lookup does not need to be exact since we are stopped
			pose.header.frame_id = self.global_frame 

			to_frame_rel = pose
			from_frame_rel = self.local_costmap.header.frame_id

			try:
		#		self.get_logger().info(f'plan...1 {from_frame_rel}')
				local_pose_transform = self.tfBuffer.lookup_transform(self.global_frame, from_frame_rel, rclpy.time.Time()) 	
				local_pose = self.transform_pose(to_frame_rel, local_pose_transform)
		#		print("plan...2", local_pose)

			except (TransformException, LookupException, ConnectivityException, ExtrapolationException) as ex:
				# self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
				self.get_logger().info('plan...3')
				pass 

		#	self.get_logger().info('plan...4')
			cellx = round((local_pose.pose.position.x - self.local_costmap.info.origin.position.x) / self.local_costmap.info.resolution)
			celly = round((local_pose.pose.position.y - self.local_costmap.info.origin.position.y) / self.local_costmap.info.resolution)
		#	self.get_logger().info('plan...5')
			cellidx = int(celly*self.local_costmap.info.width+cellx)
		#	self.get_logger().info('plan...6')
			if cellidx < 0 or cellidx >= len(self.local_costmap.data):
				self.get_logger().warn("get_closest_possible_goal landed outside costmap, returning original goal.")
				return pos_next
			cost = self.local_costmap.data[cellidx]
		#	self.get_logger().info('plan...7')
			# 执行阶段使用更严格阈值（默认只走白色自由区），
			# 防止目标点或轨迹压入灰色膨胀层。
			if cost < 0 or cost > self.drive_max_non_lethal:
		#		self.get_logger().info('plan...8')
				break
		#	self.get_logger().info('plan...9')
			closest = pose
		#	self.get_logger().info(f'plan...10 closest: {closest}')
		self.get_logger().info("get_closest_possible_goal completed...") 
		if closest is None:
			self.get_logger().warn(
				"get_closest_possible_goal: all planned path points blocked by costmap, "
				"returning original goal as fallback."
			)
			return pos_next
		return (closest.pose.position.x, closest.pose.position.y)



	




	



	def transform_pose(self, pose, transform):
		# self.get_logger().info('transform_pose...0')
		# Extract translation and rotation from transform
		translation = [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]
		# self.get_logger().info('transform_pose...1')
		rotation = Rotation.from_quat([transform.transform.rotation.x, transform.transform.rotation.y,
                                    transform.transform.rotation.z, transform.transform.rotation.w])
		# self.get_logger().info('transform_pose...2')
		# Extract position and orientation from input pose
		position = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
		orientation = Rotation.from_quat([pose.pose.orientation.x, pose.pose.orientation.y,
												pose.pose.orientation.z, pose.pose.orientation.w])
		#self.get_logger().info('transform_pose...3')
		# Apply translation and rotation	
		transformed_position = rotation.apply(position) + translation # rotation * position + translation
		transformed_orientation = rotation * orientation
		#self.get_logger().info('transform_pose...4')
		# Create and return transformed pose
		transformed_pose = PoseStamped()
		#transformed_pose.header = transform.header.child_id
		transformed_pose.pose.position.x = transformed_position[0]
		transformed_pose.pose.position.y = transformed_position[1]
		transformed_pose.pose.position.z = transformed_position[2]
		transformed_pose.pose.orientation.w = transformed_orientation.as_quat()[0]
		transformed_pose.pose.orientation.x = transformed_orientation.as_quat()[1]
		transformed_pose.pose.orientation.y = transformed_orientation.as_quat()[2]
		transformed_pose.pose.orientation.z = transformed_orientation.as_quat()[3]

		return transformed_pose








	# ── 路径密化 ──────────────────────────────────────────────────────

	def _densify_path(self, path):
		"""在长线段上均匀插入中间点，确保相邻点间距 ≤ min_wp_dist。

		只在段长 > min_wp_dist * 2 时插入，过渡段（≈0.34m）自动跳过。
		"""
		if len(path) < 2:
			return path
		densified = [path[0]]
		for i in range(len(path) - 1):
			p1 = np.array(path[i])
			p2 = np.array(path[i + 1])
			seg = p2 - p1
			seg_len = np.linalg.norm(seg)
			if seg_len > self.min_wp_dist * 2:
				n_extra = int(seg_len / self.min_wp_dist) - 1
				for j in range(1, n_extra + 1):
					alpha = j / (n_extra + 1)
					intermediate = p1 + alpha * seg
					densified.append(tuple(intermediate))
			densified.append(tuple(p2))
		return densified

	def _group_by_direction(self, path):
		"""按方向将路径点分组：同向连续段归为一组，转折点单独成组。

		返回: [[(x, y, angle), ...], ...]
		"""
		if len(path) < 2:
			return [[(path[0][0], path[0][1], 0.0)]] if path else []

		groups = []
		current_group = []
		prev_angle = None

		for i in range(len(path) - 1):
			p1 = np.array(path[i])
			p2 = np.array(path[i + 1])
			seg = p2 - p1
			angle = math.atan2(seg[1], seg[0])

			if prev_angle is None:
				current_group.append((float(path[i][0]), float(path[i][1]), angle))
			else:
				# 角度变化 ≥ 30° → 转折点，关闭当前组，当前点成为转折点
				angle_diff = abs(math.atan2(math.sin(angle - prev_angle),
				                           math.cos(angle - prev_angle)))
				if angle_diff >= math.radians(30):
					# 最后一个点也加到当前组，然后关闭
					current_group.append((float(path[i][0]), float(path[i][1]), prev_angle))
					groups.append(current_group)
					current_group = []
					# 转折点单独起组
					current_group.append((float(path[i][0]), float(path[i][1]), angle))
				else:
					current_group.append((float(path[i][0]), float(path[i][1]), angle))

			prev_angle = angle

		# 最后一个点
		if path:
			last = path[-1]
			current_group.append((float(last[0]), float(last[1]), prev_angle if prev_angle is not None else 0.0))
		if current_group:
			groups.append(current_group)

		return groups

	def drive_path(self, path):
		try:
			self._drive_path_impl(path)
		except Exception as e:
			import traceback
			self.get_logger().error(
				f"drive_path crashed: {e}\\n{traceback.format_exc()}"
			)

	def _drive_path_impl(self, path):
		self.get_logger().info("o_o 1: ") # -------------------------------------
		if len(path) < 2:
			self.get_logger().warn("drive_path got empty/short path, skip.")
			return
		if self.show_paths:
			self.visualize_path(path)

		self.get_logger().info("o_o 2: ") # -------------------------------------

		try:
			initial_pos = self.tfBuffer.lookup_transform(self.global_frame, self.base_frame, rclpy.time.Time())
		except (TransformException, LookupException, ConnectivityException, ExtrapolationException) as ex:
			pass

		self.get_logger().info("o_o 3: ") # -------------------------------------
		if self.x == None and self.y == None:
			path.insert(0, (initial_pos.transform.translation.x, initial_pos.transform.translation.y))
		else:
			path.insert(0, (self.x, self.y))

		self.get_logger().info("o_o 4: "+ str(path)) # -------------------------------------

		# 密化：长线段插入中间点，强制全局规划器沿直线走
		path = self._densify_path(path)

		# 按方向分组：直行条带用 NavigateThroughPoses（无停车），转折点用 NavigateToPose
		groups = self._group_by_direction(path)
		self.get_logger().info(f"Path densified to {len(path)} points, grouped into {len(groups)} segments")

		group_idx = 0
		for group in groups:
			if not rclpy.ok:
				return
			group_idx += 1

			# Checkpoint resume: skip already-completed segments in this cell
			if self._resume_segment_idx > 0 and group_idx <= self._resume_segment_idx:
				self.get_logger().info(
					f"[RESUME] Skip segment {group_idx}/{len(groups)} (already completed)"
				)
				continue

			try:
				if len(group) <= 1:
					# 单点（转折/掉头）：NavigateToPose，精确控姿
					x, y, angle = group[0]
					self.get_logger().info(
						f"Segment {group_idx}/{len(groups)}: turn @ ({x:.2f},{y:.2f})")
					nav_ok = self.navigate_to_pose(x, y, angle)
					if not nav_ok:
						self.get_logger().warn(
							f"NavigateToPose failed to ({x:.2f},{y:.2f}), "
							"waiting 2s for costmap refresh, then retrying...")
						time.sleep(2)
						if rclpy.ok:
							nav_ok = self.navigate_to_pose(x, y, angle)
							if not nav_ok:
								self.get_logger().warn("Retry also failed, continuing.")
				else:
					# 多同向点（直行条带）：NavigateThroughPoses，平滑连续
					self.get_logger().info(
						f"Segment {group_idx}/{len(groups)}: straight {len(group)} pts "
						f"({group[0][0]:.2f},{group[0][1]:.2f}) -> "
						f"({group[-1][0]:.2f},{group[-1][1]:.2f})")
					nav_ok = self.navigate_through_poses(group)
					if not nav_ok:
						self.get_logger().warn(
							f"NavigateThroughPoses failed (segment {group_idx}), "
							"falling back to point-by-point NavigateToPose...")
						for x, y, angle in group:
							if not rclpy.ok:
								break
							pt_ok = self.navigate_to_pose(x, y, angle)
							if not pt_ok:
								self.get_logger().warn(
									f"Fallback waypoint ({x:.2f},{y:.2f}) failed, skipping.")
			except Exception as e:
				self.get_logger().error(
					f"Segment {group_idx} failed: {e}, skipping.")

			# Save checkpoint after each segment (segment-level granularity)
			save_checkpoint(
				cell_idx=self._current_cell_idx - 1,  # 0-based for checkpoint
				segment_idx=group_idx,
				total_cells=self._total_cells,
			)

		self.get_logger().info("o_o 11: ")
		if self.show_paths:
			self.visualize_path(path, False)
		self.get_logger().info("drive path completed...") # -------------------------------------






	def navigate_to_pose(self, x, y, angle):
		"""Sends a goal to the NavigateToPose action server and waits for the result."""
		self.get_logger().info(f'Waiting for NavigateToPose action server...')
		if not self.navigate_to_pose_client.wait_for_server(timeout_sec=5.0):
			self.get_logger().error('NavigateToPose action server not available after waiting!')
			self._goal_tracker.record_to_pose(x, y, self.x, self.y, False, "server_unavailable")
			return False

		# Create the goal message
		goal_msg = NavigateToPose.Goal()
		goal_msg.pose.header.frame_id = self.global_frame
		goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
		goal_msg.pose.pose.position.x = x
		goal_msg.pose.pose.position.y = y
		goal_msg.pose.pose.position.z = 0.0
		
		angle_quat = self.euler_to_quaternion(angle, 0, 0)
		goal_msg.pose.pose.orientation.x = angle_quat[0]
		goal_msg.pose.pose.orientation.y = angle_quat[1]
		goal_msg.pose.pose.orientation.z = angle_quat[2]
		goal_msg.pose.pose.orientation.w = angle_quat[3]

		self.get_logger().info(f'Sending goal to NavigateToPose server: x={x:.2f}, y={y:.2f}, angle={angle:.2f}')
		send_goal_future = self.navigate_to_pose_client.send_goal_async(goal_msg)
		
		try:
			rclpy.spin_until_future_complete(self.sub_node, send_goal_future, timeout_sec=10.0)
			goal_handle = send_goal_future.result()
		except Exception as e:
			self.get_logger().error(f"Exception while sending goal: {e}")
			self._goal_tracker.record_to_pose(x, y, self.x, self.y, False, f"send_exception:{e}")
			return False

		if goal_handle is None:
			self.get_logger().error('Timed out waiting for goal to be accepted by the server.')
			self._goal_tracker.record_to_pose(x, y, self.x, self.y, False, "send_timeout")
			return False

		if not goal_handle.accepted:
			self.get_logger().error('Goal rejected by server')
			self._goal_tracker.record_to_pose(x, y, self.x, self.y, False, "goal_rejected")
			return False

		self.get_logger().info('Goal accepted by server, waiting for result...')
		result_future = goal_handle.get_result_async()

		try:
			rclpy.spin_until_future_complete(self.sub_node, result_future, timeout_sec=60.0) # Wait up to 60 seconds for the result
			result = result_future.result()
		except Exception as e:
			self.get_logger().error(f"Exception while waiting for result: {e}")
			goal_handle.cancel_goal_async()
			self._goal_tracker.record_to_pose(x, y, self.x, self.y, False, f"result_exception:{e}")
			return False

		if result is None:
			self.get_logger().warn("Navigation timed out!")
			goal_handle.cancel_goal_async() # Cancel the goal if we timed out
			self._goal_tracker.record_to_pose(x, y, self.x, self.y, False, "result_timeout")
			return False

		status = result.status
		if status == GoalStatus.STATUS_SUCCEEDED:
			self.get_logger().info('Goal succeeded!')
			self._goal_tracker.record_to_pose(x, y, self.x, self.y, True, "")
			return True
		else:
			self.get_logger().warn(f'Goal failed with status: {status}')
			self._goal_tracker.record_to_pose(x, y, self.x, self.y, False, f"result_status:{status}")
			return False





	# ── NavigateThroughPoses ──────────────────────────────────────────

	def navigate_through_poses(self, poses):
		"""一次性发送同向路径点列表，机器人平滑穿行不中间停车。

		poses: [(x, y, angle), ...]
		返回 True/False。
		"""
		if not poses:
			return False

		n_pts = len(poses)
		x0, y0 = poses[0][0], poses[0][1]
		xn, yn = poses[-1][0], poses[-1][1]

		self.get_logger().info('Waiting for NavigateThroughPoses action server...')
		if not self.navigate_through_poses_client.wait_for_server(timeout_sec=5.0):
			self.get_logger().error('NavigateThroughPoses action server not available!')
			self._goal_tracker.record_through_poses(n_pts, x0, y0, xn, yn, self.x, self.y, False, "server_unavailable")
			return False

		goal_msg = NavigateThroughPoses.Goal()
		for (x, y, angle) in poses:
			angle_quat = self.euler_to_quaternion(angle, 0, 0)
			ps = PoseStamped()
			ps.header.frame_id = self.global_frame
			ps.header.stamp = self.get_clock().now().to_msg()
			ps.pose.position.x = x
			ps.pose.position.y = y
			ps.pose.position.z = 0.0
			ps.pose.orientation.x = angle_quat[0]
			ps.pose.orientation.y = angle_quat[1]
			ps.pose.orientation.z = angle_quat[2]
			ps.pose.orientation.w = angle_quat[3]
			goal_msg.poses.append(ps)

		self.get_logger().info(
			f'Sending NavigateThroughPoses: {n_pts} poses')

		send_goal_future = self.navigate_through_poses_client.send_goal_async(goal_msg)
		try:
			rclpy.spin_until_future_complete(self.sub_node, send_goal_future, timeout_sec=10.0)
			goal_handle = send_goal_future.result()
		except Exception as e:
			self.get_logger().error(f"NavigateThroughPoses send_goal exception: {e}")
			self._goal_tracker.record_through_poses(n_pts, x0, y0, xn, yn, self.x, self.y, False, f"send_exception:{e}")
			return False

		if goal_handle is None:
			self.get_logger().error('NavigateThroughPoses goal not accepted (timeout).')
			self._goal_tracker.record_through_poses(n_pts, x0, y0, xn, yn, self.x, self.y, False, "send_timeout")
			return False
		if not goal_handle.accepted:
			self.get_logger().error('NavigateThroughPoses goal rejected by server.')
			self._goal_tracker.record_through_poses(n_pts, x0, y0, xn, yn, self.x, self.y, False, "goal_rejected")
			return False

		# 超时 = 60s 基础 + 每点 2s
		timeout = max(60, n_pts * 2)
		self.get_logger().info(f'NavigateThroughPoses accepted, waiting (timeout={timeout}s)...')
		result_future = goal_handle.get_result_async()
		try:
			rclpy.spin_until_future_complete(self.sub_node, result_future, timeout_sec=timeout)
			result = result_future.result()
		except Exception as e:
			self.get_logger().error(f"NavigateThroughPoses result exception: {e}")
			goal_handle.cancel_goal_async()
			self._goal_tracker.record_through_poses(n_pts, x0, y0, xn, yn, self.x, self.y, False, f"result_exception:{e}")
			return False

		if result is None:
			self.get_logger().warn("NavigateThroughPoses timed out!")
			goal_handle.cancel_goal_async()
			self._goal_tracker.record_through_poses(n_pts, x0, y0, xn, yn, self.x, self.y, False, "result_timeout")
			return False

		status = result.status
		if status == GoalStatus.STATUS_SUCCEEDED:
			self.get_logger().info('NavigateThroughPoses succeeded!')
			self._goal_tracker.record_through_poses(n_pts, x0, y0, xn, yn, self.x, self.y, True, "")
			return True
		else:
			self.get_logger().warn(f'NavigateThroughPoses failed with status: {status}')
			self._goal_tracker.record_through_poses(n_pts, x0, y0, xn, yn, self.x, self.y, False, f"result_status:{status}")
			return False

	def add_more_waypoints(self, x1, y1, x2, y2, angle_quat):
		# Calculate the midpoint
		new_x = (x1 + x2) / 2.0
		new_y = (y1 + y2) / 2.0
		self.get_logger().info("-------- including one mid-point: (%f, %f)" % (new_x, new_y))
		# Append the mid waypoint to the data dictionary with the index as the key
		index = len(self.pose_output) + 1
		self.pose_output[index] = {
								"position":
								{
									"x": float(new_x),
									"y": float(new_y),
									"z": 0.0
								},
								"orientation":
								{
									"w": angle_quat[3],
									"x": angle_quat[0],
									"y": angle_quat[1],
									"z": angle_quat[2]
								},
							}


	def write_pose(self, x, y, angle):
		# # self.get_logger().info("Moving to (%f, %f, %.0f)" % (x, y, angle*180/pi))

		print("write_pose")

		angle_quat = self.euler_to_quaternion(angle,0,0)

		if len(self.pose_output) >= 1:
			last_index = len(self.pose_output)
			print("write_pose")
			x1 = self.pose_output[last_index]["position"]["x"]
			y1 = self.pose_output[last_index]["position"]["y"]
			print("write_pose")
			# Calculate the distance between the two coordinates
			distance = math.sqrt((x - x1)**2 + (y - y1)**2)
			# Check if the distance is greater than the minimum length
			if distance > self.min_wp_dist * 2: # Add a point only if the segment is long enough
				self.add_more_waypoints(x1, y1, x, y, angle_quat)

		# Append the values to the data dictionary with the index as the key
		index = len(self.pose_output) + 1
		self.pose_output[index] = {
                                "position":
                                {
                                    "x": x,
                                    "y": y,
                                    "z": 0.0
                                },
                                "orientation":
                                {
                                    "w": angle_quat[3],
                                    "x": angle_quat[0],
                                    "y": angle_quat[1],
                                    "z": angle_quat[2]
                                },
                            } 
		self.x = x
		self.y = y









	def drive_polygon(self, polygon):
		polygon = self._normalize_polygon(polygon, context="drive_polygon input")
		if polygon is None:
			return
		try:
			# Repair potential self-intersections and collapse multiparts to a single polygon.
			repaired = polygon.buffer(0)
		except Exception:
			repaired = polygon
		polygon = self._normalize_polygon(repaired, context="drive_polygon repair")
		if polygon is None:
			return
		self.get_logger().info("drive_polygon 1: ") # -------------------------------------
		if not self.show_all_cells:
			self.visualize_cell(polygon.exterior.coords[:])
		#self.get_logger().info("x_x 2: ") # -------------------------------------

		# Align longest side of the polygon to the horizontal axis
		angle = get_angle_of_longest_side_to_horizontal(polygon)
		#self.get_logger().info("x_x 3: ") # -------------------------------------
		if angle == None:
			self.get_logger().warn("Can not return polygon")
			return
		
		#self.get_logger().info("x_x 4: ") # -------------------------------------
		angle+=pi/2 # up/down instead of left/right
		poly_rotated = rotate_polygon(polygon, angle)
		#self.get_logger().info("x_x 5: ") # -------------------------------------

		self.get_logger().debug("Rotated polygon by %.0f: %s" % (angle*180/pi, str(poly_rotated.exterior.coords[:])))

		executed = False
		if self.border_drive:
		#	self.get_logger().info("x_x 6: ") # -------------------------------------
			path_rotated = border_calc_path(poly_rotated, self.robot_width, clearance=self.coverage_clearance)
		#	self.get_logger().info("x_x 7: ") # -------------------------------------
			if path_rotated:
				path = rotate_points(path_rotated, -angle)
		#		self.get_logger().info("x_x 8: ") # -------------------------------------
				self.drive_path(path)
				executed = True
			else:
				self.get_logger().warn("border_drive produced empty path, skipping border pass")
		#	self.get_logger().info("x_x 9: ") # -------------------------------------


		# run
		self.get_logger().info("x_x 10: ") # -------------------------------------
		path_rotated = trapezoid_calc_path(poly_rotated, self.robot_width, clearance=self.coverage_clearance)
		if path_rotated:
			# self.get_logger().info("x_x 11: ") # -------------------------------------
			path = rotate_points(path_rotated, -angle)
			# self.get_logger().info("x_x 12: ") # -------------------------------------
			self.drive_path(path)
			executed = True
		else:
			b = polygon.bounds
			self.get_logger().warn(
				"trapezoid path empty: area=%.3f, bounds=[%.3f,%.3f,%.3f,%.3f], width=%.3f, clearance=%.3f"
				% (polygon.area, b[0], b[1], b[2], b[3], self.robot_width, self.coverage_clearance)
			)
			if not self.border_drive:
				path_rotated = border_calc_path(poly_rotated, self.robot_width, clearance=self.coverage_clearance)
				if path_rotated:
					path = rotate_points(path_rotated, -angle)
					self.drive_path(path)
					executed = True
				else:
					self.get_logger().warn("border fallback also empty, no coverage for this cell")
		# self.get_logger().info("x_x 13: ") # -------------------------------------

		# cleanup
		if executed and (not self.show_all_cells):
			self.visualize_cell(polygon.exterior.coords[:], False)
		#self.get_logger().info("x_x 14: ") # -------------------------------------
		self.get_logger().debug("Polygon done")
		





	def private_shutdown(self):
		self.visualization_cleanup()
		"""Cancel pending task request and kill sub_node."""
		self.get_logger().info('Canceling current sub_node task i.e. if any.')
		if self.result_future and self.goal_handle:
			# Check if the sub_node is still valid before using it
			if self.sub_node and rclpy.ok() and self.sub_node.handle:
				try:
					future = self.goal_handle.cancel_goal_async()
					rclpy.spin_until_future_complete(self.sub_node, future, timeout_sec=1.0)
				except Exception as e:
					self.get_logger().warn(f"Exception during goal cancellation: {e}")
		
		if self.sub_node and rclpy.ok() and self.sub_node.handle:
			self.sub_node.destroy_node()
		
		self.get_logger().info("Private shutdown sequence finished.")
		return







def main(args=None):
	rclpy.init(args=args)
	p = MapDrive()
	try:
		rclpy.spin(p)
	except KeyboardInterrupt:
		p._goal_tracker.abort()
		p.get_logger().info('Shutting down gracefully...')
	except Exception as e:
		p.get_logger().error(f'Unexpected error: {e}')
	finally:
		p.private_shutdown()
		p.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
    main()