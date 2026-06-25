#!/usr/bin/env python3

import math
from datetime import datetime
from collections import defaultdict
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))

def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))

def dist2d(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

class LaberintSolver(Node):
    def __init__(self):
        super().__init__('laberint_solver')
        
        self.LINEAR_SPEED = 0.02
        self.SLOW_SPEED = 0.01
        self.MIN_SPEED = 0.0
        self.ANGULAR_SPEED = 0.25
        self.FRONT_BRAKE = 0.12
        self.FRONT_SLOW = 0.25
        self.SIDE_PANIC = 0.12
        self.DIAG_PANIC = 0.14
        self.CHASSIS_FILTER = 0.05

        self.WALL_STOP = 0.20
        self.TURN_ANGLE = 55
        self.TURN_ANGLE_RAD = math.radians(self.TURN_ANGLE)

        self.last_movement_time = self.get_clock().now()
        self.escape_start_x = 0.0
        self.escape_start_y = 0.0
        self.escape_dir_linear = -1.0
        self.escape_dir_angular = 0.0

        self.scan_data = None
        self.finished = False
        self.meta_reached = False
        self.pos_x = self.pos_y = self.yaw = 0.0
        self.grid_res = 0.30
        self.visited_cells = defaultdict(int)

        self.topological_graph = []
        self.current_node = None
        self.returning_to_parent = False
        self.path_letters = []
        self.INTERSECT_OPEN_THRESHOLD = 0.35  

        self.mode = "EXPLORING"
        self.target_yaw = 0.0
        self.backtrack_after_turn = False

        self.detection_cooldown = False
        self.cooldown_x = self.cooldown_y = 0.0
        self.COOLDOWN_DISTANCE = 0.10
        self.in_intersection_zone = False
        self.verify_start_x = self.verify_start_y = 0.0

        self.last_angular = 0.0
        self.max_ang_step = 0.12
        self.last_best_angle = 0.0

        self.stuck_ref_x = 0.0
        self.stuck_ref_y = 0.0
        self.stuck_timer_start = self.get_clock().now()
        self.STUCK_RADIUS = 0.05

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos)
        self.create_subscription(Odometry, '/odom', self.odom_cb, qos)
        self.create_subscription(Bool, '/meta', self.meta_cb, 10)
        self.create_timer(0.1, self.control_loop)

    def scan_cb(self, msg):
        if not msg.ranges: return
        self.scan_data = [float('inf') if (r is None or math.isnan(r) or math.isinf(r) or r<=0 or r<self.CHASSIS_FILTER) else min(r,2.0) for r in msg.ranges]

    def odom_cb(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.visited_cells[self.cell(self.pos_x, self.pos_y)] += 1

    def meta_cb(self, msg): self.meta_reached = bool(msg.data)

    def svals(self, ca, w=10):
        if not self.scan_data: return []
        n=len(self.scan_data); ca=int(ca)%n; h=w//2
        return [min(self.scan_data[(ca+o)%n],2.0) for o in range(-h,h+1) if self.scan_data[(ca+o)%n]>0.01 and not math.isinf(self.scan_data[(ca+o)%n])]
    
    def savg(self, ca, w=10):
        v=self.svals(ca,w); return sum(v)/len(v) if v else 2.0
        
    def smin(self, ca, w=10):
        v=self.svals(ca,w); return min(v) if v else 2.0
        
    def front_metrics(self):
        return min(self.smin(0,20),self.smin(10,14),self.smin(350,14)), self.savg(0,28)
        
    def side_metrics(self):
        return self.smin(90,24),self.smin(270,24),self.smin(45,18),self.smin(315,18)
        
    def cmd(self, l, a):
        m=Twist(); m.linear.x,m.angular.z=float(l),float(a); self.cmd_pub.publish(m)
        
    def stop(self): self.cmd(0,0)
    
    def clamp(self, v, lo, hi): return max(lo,min(hi,v))
    
    def cell(self, x, y): return (round(x/self.grid_res),round(y/self.grid_res))
    
    def smooth_ang(self, t):
        self.last_angular = self.clamp(self.last_angular + self.clamp(t - self.last_angular, -self.max_ang_step, self.max_ang_step), -0.45, 0.45)
        return self.last_angular
        
    def activate_cooldown(self):
        self.detection_cooldown = True
        self.cooldown_ticks = 30
        self.cooldown_x = self.pos_x
        self.cooldown_y = self.pos_y

    def execute_safe_turn(self, target_yaw, angular_speed):
        diff = angle_diff(target_yaw, self.yaw)
        if abs(diff) < 0.05:
            self.cmd(0, 0); return True
        speed = min(abs(diff) * 1.5, angular_speed)
        if speed < 0.08: speed = 0.08
        self.cmd(0, speed if diff > 0 else -speed)
        return False

    def simplify_path(self):
        changed=True
        while changed and len(self.path_letters)>=3:
            changed=False; seq="".join(self.path_letters[-3:])
            reps={"LBL":"S","RBR":"S","LBS":"R","RBS":"L","SBL":"R","SBR":"L","LBR":"B","RBL":"B","SBS":"B"}
            if seq in reps:
                self.path_letters=self.path_letters[:-3]+list(reps[seq])
                changed=True

    def process_topological_node(self, paths, came_from):
        if self.returning_to_parent:
            self.returning_to_parent=False
            self.path_letters.append('B'); self.simplify_path()
            node=self.current_node
            if node is None:
                node={'id':len(self.topological_graph),'unexplored':paths.copy(),'parent':None,
                      'came_from':came_from,'going_to':None,'x':self.pos_x,'y':self.pos_y}
                self.topological_graph.append(node); self.current_node=node
        else:
            NODE_MERGE_DIST = 0.15
            existing_node = None
            for n in self.topological_graph:
                if dist2d(self.pos_x, self.pos_y, n['x'], n['y']) < NODE_MERGE_DIST:
                    existing_node = n
                    break

            if existing_node:
                node = existing_node
                self.current_node = node
            else:
                node={'id':len(self.topological_graph),'unexplored':paths.copy(),'parent':self.current_node,
                      'came_from':came_from,'going_to':None,'x':self.pos_x,'y':self.pos_y}
                self.topological_graph.append(node); self.current_node=node

        if not node['unexplored']:
            self.topological_graph = []
            self.current_node = None
            self.returning_to_parent = False
            self.path_letters = []
            
            best_dir, best_v = paths[0], float('inf')
            for d in paths:
                v = self.visited_cells.get(self.cell(self.pos_x+0.35*math.cos(d), self.pos_y+0.35*math.sin(d)), 0)
                if v < best_v: 
                    best_v, best_dir = v, d
                    
            self.target_yaw = best_dir
            self.mode = "EXECUTING_TURN"
            return

        valid = []
        for d in node['unexplored']:
            if abs(angle_diff(d, came_from)) < 0.5:
                continue
            for p in paths:
                if abs(angle_diff(d, p)) < 0.8:
                    valid.append(d)
                    break
        
        if not valid:
            valid = [p for p in paths if abs(angle_diff(p, came_from)) > 0.5]
        
        if not valid:
            self.topological_graph = []
            self.current_node = None
            self.returning_to_parent = False
            self.path_letters = []
            
            best_dir, best_v = paths[0], float('inf')
            for d in paths:
                v = self.visited_cells.get(self.cell(self.pos_x+0.35*math.cos(d), self.pos_y+0.35*math.sin(d)), 0)
                if v < best_v: 
                    best_v, best_dir = v, d
                    
            self.target_yaw = best_dir
            self.mode = "EXECUTING_TURN"
            return

        best_dir,best_v,best_i=valid[0],float('inf'),0
        for i,d in enumerate(valid):
            v=self.visited_cells.get(self.cell(self.pos_x+0.35*math.cos(d),self.pos_y+0.35*math.sin(d)),0)
            if v<best_v: best_v,best_dir,best_i=v,d,i
        
        for j, d in enumerate(node['unexplored']):
            if abs(angle_diff(d, best_dir)) < 0.8:
                node['unexplored'].pop(j)
                break
        
        node['going_to']=best_dir
        self.simplify_path()
        self.target_yaw=best_dir
        self.mode="EXECUTING_TURN"

    def control_loop(self):
        if not self.scan_data or self.finished: return
        if self.meta_reached:
            self.stop(); self.finished=True
            return

        fmin, favg = self.front_metrics()
        lmin, rmin, flmin, frmin = self.side_metrics()

        if self.mode == "ESCAPING":
            adv_escape = dist2d(self.pos_x, self.pos_y, self.escape_start_x, self.escape_start_y)
            if adv_escape >= 0.08:
                self.stop()
                self.mode = "EXPLORING"
                self.activate_cooldown()
                self.last_movement_time = self.get_clock().now()
                return
            
            back = self.savg(180, 20)
            if back < 0.14 or back >= 2:
                self.stop()
                self.mode = "EXPLORING"
                self.activate_cooldown()
                self.last_movement_time = self.get_clock().now()
                return
            
            self.cmd(self.SLOW_SPEED * self.escape_dir_linear, self.escape_dir_angular)
            return

        if self.mode in ["EXECUTING_TURN", "TURN_180"] or fmin < self.FRONT_BRAKE:
            tiempo_atrapado = (self.get_clock().now() - self.last_movement_time).nanoseconds / 1e9
            limite_tiempo = 50.0 if self.mode == "TURN_180" else 25.0
            
            if tiempo_atrapado > limite_tiempo: 
                self.stop()
                self.escape_start_x = self.pos_x
                self.escape_start_y = self.pos_y
                
                if self.backtrack_after_turn:
                    self.backtrack_after_turn = False
                    self.returning_to_parent = True
                
                self.escape_dir_linear = -1.0 
                if lmin < rmin:
                    self.escape_dir_angular = -0.08 
                else:
                    self.escape_dir_angular = 0.08
                    
                self.mode = "ESCAPING"
                return
        else:
            self.last_movement_time = self.get_clock().now()

        if self.mode != "ESCAPING":
            dist_recorrida = dist2d(self.pos_x, self.pos_y, self.stuck_ref_x, self.stuck_ref_y)
            tiempo_en_zona = (self.get_clock().now() - self.stuck_timer_start).nanoseconds / 1e9
            
            if dist_recorrida > self.STUCK_RADIUS:
                self.stuck_ref_x = self.pos_x
                self.stuck_ref_y = self.pos_y
                self.stuck_timer_start = self.get_clock().now()
            else:
                limite_tiempo = 40.0 if self.mode == "TURN_180" else 50.0 
                
                if tiempo_en_zona > limite_tiempo:
                    self.stop()
                    self.escape_start_x = self.pos_x
                    self.escape_start_y = self.pos_y
                    
                    if self.backtrack_after_turn:
                        self.backtrack_after_turn = False
                        self.returning_to_parent = True
                    
                    self.escape_dir_linear = -1.0 
                    if lmin < rmin:
                        self.escape_dir_angular = -0.08 
                    else:
                        self.escape_dir_angular = 0.08
                        
                    self.mode = "ESCAPING"
                    
                    self.stuck_ref_x = self.pos_x
                    self.stuck_ref_y = self.pos_y
                    self.stuck_timer_start = self.get_clock().now()
                    return

        if self.mode == "APPROACHING_INTERSECTION":
            if lmin < rmin and lmin < 0.40:
                err = lmin - 0.155
            elif rmin < 0.40:
                err = 0.155 - rmin
            else:
                err = 0.0
                
            ac = self.clamp(err * 3.5, -0.40, 0.40)
            grado_cero = self.smin(0, 10)
            adv = dist2d(self.pos_x, self.pos_y, self.verify_start_x, self.verify_start_y)

            if adv < 0.04 and grado_cero > 0.18:
                self.cmd(self.SLOW_SPEED, ac); return

            self.stop()
            self.mode = "ANALYZING_NODE"
            return

        if self.mode == "ANALYZING_NODE":
            frente_abierto = self.savg(0, 15) > 0.29
            izq_abierto = self.savg(90, 15) > 0.35 or self.savg(45, 30) > 0.45
            der_abierto = self.savg(270, 15) > 0.35 or self.savg(315, 30) > 0.45

            caminos_detectados_relativos = []
            if frente_abierto: caminos_detectados_relativos.append(0.0)
            if izq_abierto: caminos_detectados_relativos.append(self.TURN_ANGLE_RAD)
            if der_abierto: caminos_detectados_relativos.append(-self.TURN_ANGLE_RAD)

            fp = [normalize_angle(self.yaw + ang) for ang in caminos_detectados_relativos]
            cf = normalize_angle(self.yaw + math.pi)

            if len(fp) == 0:
                val_L = max(self.savg(90, 20), self.savg(45, 20))
                val_R = max(self.savg(270, 20), self.savg(315, 20))
                
                if val_L > 0.30 and val_L > (val_R + 0.10):
                    self.target_yaw = normalize_angle(self.yaw + self.TURN_ANGLE_RAD)
                    self.mode = "EXECUTING_TURN"
                elif val_R > 0.30 and val_R > (val_L + 0.10):
                    self.target_yaw = normalize_angle(self.yaw - self.TURN_ANGLE_RAD)
                    self.mode = "EXECUTING_TURN"
                elif val_L > 0.30 and val_R > 0.45:
                    self.target_yaw = normalize_angle(self.yaw + self.TURN_ANGLE_RAD) if val_L > val_R else normalize_angle(self.yaw - self.TURN_ANGLE_RAD)
                    self.mode = "EXECUTING_TURN"
                else:
                    self.target_yaw = normalize_angle(self.yaw + math.pi)
                    self.activate_cooldown()
                    self.mode = "TURN_180"
                return

            if self.returning_to_parent:
                es_el_padre_real = False
                if self.current_node:
                    dist_al_padre = dist2d(self.pos_x, self.pos_y, self.current_node['x'], self.current_node['y'])
                    if dist_al_padre < 0.25:
                        es_el_padre_real = True

                if self.current_node is None or es_el_padre_real:
                    self.returning_to_parent = False
                    self.simplify_path()
                    
                    if self.current_node and len(self.current_node['unexplored']) > 0:
                        best_match_diff = float('inf')
                        best_dir = cf
                        best_unexp_idx = -1
                        
                        for current_dir in fp:
                            for i, unexp_dir in enumerate(self.current_node['unexplored']):
                                diff = abs(angle_diff(current_dir, unexp_dir))
                                if diff < best_match_diff:
                                    best_match_diff = diff
                                    best_dir = current_dir
                                    best_unexp_idx = i
                                    
                        if best_match_diff < 0.8: 
                            self.current_node['unexplored'].pop(best_unexp_idx)
                        else:
                            best_dir = fp[-1] if len(fp) > 1 else fp[0]
                            
                        self.target_yaw = best_dir
                        self.mode = "EXECUTING_TURN"
                    else:
                        self.topological_graph = []
                        self.current_node = None
                        self.returning_to_parent = False
                        self.path_letters = []
                        
                        best_dir, best_v = fp[0], float('inf')
                        for d in fp:
                            v = self.visited_cells.get(self.cell(self.pos_x+0.35*math.cos(d), self.pos_y+0.35*math.sin(d)), 0)
                            if v < best_v: best_v, best_dir = v, d
                                
                        self.target_yaw = best_dir
                        self.mode = "EXECUTING_TURN"
                else:
                    self.returning_to_parent = False
                    cf_hacia_padre = self.yaw
                    self.process_topological_node(fp, cf_hacia_padre)
                return

            if len(fp) == 1:
                if fp[0] == 0.0:
                    self.mode = "EXPLORING"
                    self.activate_cooldown()
                    self.cooldown_ticks = 20
                else:
                    self.target_yaw = fp[0]
                    self.mode = "EXECUTING_TURN"
            else:
                self.process_topological_node(fp, cf)

        if self.mode == "EXECUTING_TURN":
            if self.execute_safe_turn(self.target_yaw, self.ANGULAR_SPEED):
                self.mode = "EXPLORING"
                self.activate_cooldown()
                self.in_intersection_zone = True
            return

        if self.mode == "TURN_180":
            if self.execute_safe_turn(self.target_yaw, self.ANGULAR_SPEED * 0.8):
                if self.backtrack_after_turn:
                    self.backtrack_after_turn = False
                    self.returning_to_parent = True
                self.mode = "EXPLORING"
                self.activate_cooldown()
                self.in_intersection_zone = True
            return

        if lmin < 0.30 and rmin < 0.30:
            self.in_intersection_zone = False

        izq_abierto_mov = self.savg(90, 20) > 0.35 or self.savg(45, 20) > 0.45
        der_abierto_mov = self.savg(270, 20) > 0.35 or self.savg(315, 20) > 0.45
        has_lat = izq_abierto_mov or der_abierto_mov

        if abs(self.last_angular) > 0.15: has_lat = False
        
        if self.detection_cooldown:
            dist_desde_nodo = dist2d(self.pos_x, self.pos_y, self.cooldown_x, self.cooldown_y)
            if dist_desde_nodo >= self.COOLDOWN_DISTANCE:
                self.detection_cooldown = False
                self.in_intersection_zone = False 
            else: 
                has_lat = False
                self.in_intersection_zone = True

        if has_lat and not self.detection_cooldown and not self.in_intersection_zone:
            self.mode = "APPROACHING_INTERSECTION" 
            self.verify_start_x, self.verify_start_y = self.pos_x, self.pos_y
            self.in_intersection_zone = True
            return

        elif not self.detection_cooldown and fmin <= self.WALL_STOP and has_lat:
            self.stop()
            self.mode = "ANALYZING_NODE"
            self.verify_start_x, self.verify_start_y = self.pos_x, self.pos_y
            self.in_intersection_zone = True
            return

        back_dist = self.savg(180, 24)
        if fmin < 0.20 and lmin < 0.20 and rmin < 0.20 and back_dist > 0.30:
            has_exit = False
            for sd in range(0, 360, 15):
                if 140 <= sd <= 220: continue
                if self.savg(sd, 14) > 0.30: has_exit = True; break
            if not has_exit:
                self.target_yaw = normalize_angle(self.yaw + math.pi)
                self.backtrack_after_turn = True
                self.mode = "TURN_180"; return

        if fmin < self.FRONT_BRAKE:
            td = 1.0 if lmin > rmin else -1.0
            self.cmd(0, td * self.ANGULAR_SPEED * 0.6); return

        TARGET = 0.155        
        TARGET_DIAG = 0.220   
        WALL_VISIBLE = 0.32   
        
        c_flmin = min(flmin, 0.26)
        c_frmin = min(frmin, 0.26)
        
        error = 0.0

        if lmin < WALL_VISIBLE and rmin < WALL_VISIBLE:
            if lmin < rmin:
                error = (lmin - TARGET) * 3.0 + (c_flmin - TARGET_DIAG) * 1.5
            else:
                error = (TARGET - rmin) * 3.0 + (TARGET_DIAG - c_frmin) * 1.5
        elif lmin < WALL_VISIBLE:
            error = (lmin - TARGET) * 3.0 + (c_flmin - TARGET_DIAG) * 1.5
        elif rmin < WALL_VISIBLE:
            error = (TARGET - rmin) * 3.0 + (TARGET_DIAG - c_frmin) * 1.5

        if lmin < 0.115 or flmin < 0.13:
            error = -0.5  
        elif rmin < 0.115 or frmin < 0.13:
            error = 0.5   

        angular = self.smooth_ang(self.clamp(error, -0.45, 0.45))
        
        linear = self.LINEAR_SPEED
        if fmin < self.FRONT_SLOW or abs(angular) > 0.20:
            linear = self.SLOW_SPEED

        self.cmd(linear, angular)

def main(args=None):
    rclpy.init(args=args)
    node = LaberintSolver()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok(): node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()