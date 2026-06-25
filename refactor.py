#!/usr/bin/env python3

import math
from collections import defaultdict
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

def fetch_yaw(q):
    vy = 2.0 * (q.w * q.z + q.x * q.y)
    vx = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(vy, vx)

def calc_offset(a1, a2):
    return math.atan2(math.sin(a1 - a2), math.cos(a1 - a2))

def wrap_rads(ang):
    return math.atan2(math.sin(ang), math.cos(ang))

def calc_hypot(x_a, y_a, x_b, y_b):
    return math.sqrt((x_a - x_b)**2 + (y_a - y_b)**2)

class CoreNavigator(Node):
    def __init__(self):
        super().__init__('core_navigator')
        
        self.vel_fwd = 0.02
        self.vel_slow = 0.01
        self.vel_min = 0.0
        self.spin_max = 0.25
        self.stop_th = 0.12
        self.slow_th = 0.25
        self.panic_s = 0.12
        self.panic_d = 0.14
        self.blind_th = 0.05

        self.wall_gap = 0.20
        self.deg_limit = 55
        self.rad_limit = math.radians(self.deg_limit)

        self.time_mark = self.get_clock().now()
        self.evd_x = 0.0
        self.evd_y = 0.0
        self.evd_lin = -1.0
        self.evd_ang = 0.0

        self.laser_arr = None
        self.is_done = False
        self.goal_flag = False
        self.curr_x = self.curr_y = self.curr_h = 0.0
        self.map_res = 0.30
        self.heat_map = defaultdict(int)

        self.node_net = []
        self.active_idx = None
        self.is_reversing = False
        self.char_hist = []
        self.gap_th = 0.35  

        self.status = "STATE_DRIVE"
        self.aim_h = 0.0
        self.flag_reverse = False

        self.cd_active = False
        self.cd_origin_x = self.cd_origin_y = 0.0
        self.cd_dist = 0.10
        self.inside_junc = False
        self.chk_x = self.chk_y = 0.0

        self.prev_w = 0.0
        self.w_step = 0.12
        self.prev_best_h = 0.0

        self.jam_x = 0.0
        self.jam_y = 0.0
        self.jam_time = self.get_clock().now()
        self.jam_rad = 0.05

        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        custom_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(LaserScan, '/scan', self.cb_laser, custom_qos)
        self.create_subscription(Odometry, '/odom', self.cb_kinematics, custom_qos)
        self.create_subscription(Bool, '/meta', self.cb_target, 10)
        self.create_timer(0.1, self.main_tick)

    def cb_laser(self, msg):
        if not msg.ranges: return
        self.laser_arr = [float('inf') if (r is None or math.isnan(r) or math.isinf(r) or r<=0 or r<self.blind_th) else min(r,2.0) for r in msg.ranges]

    def cb_kinematics(self, msg):
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y
        self.curr_h = fetch_yaw(msg.pose.pose.orientation)
        self.heat_map[self.get_grid(self.curr_x, self.curr_y)] += 1

    def cb_target(self, msg): self.goal_flag = bool(msg.data)

    def get_slice(self, deg, span=10):
        if not self.laser_arr: return []
        tot=len(self.laser_arr); deg=int(deg)%tot; half=span//2
        return [min(self.laser_arr[(deg+i)%tot],2.0) for i in range(-half,half+1) if self.laser_arr[(deg+i)%tot]>0.01 and not math.isinf(self.laser_arr[(deg+i)%tot])]
    
    def get_mean(self, deg, span=10):
        arr=self.get_slice(deg,span); return sum(arr)/len(arr) if arr else 2.0
        
    def get_min(self, deg, span=10):
        arr=self.get_slice(deg,span); return min(arr) if arr else 2.0
        
    def fetch_front(self):
        return min(self.get_min(0,20),self.get_min(10,14),self.get_min(350,14)), self.get_mean(0,28)
        
    def fetch_sides(self):
        return self.get_min(90,24),self.get_min(270,24),self.get_min(45,18),self.get_min(315,18)
        
    def push_vel(self, lin, ang):
        t=Twist(); t.linear.x,t.angular.z=float(lin),float(ang); self.pub_vel.publish(t)
        
    def halt(self): self.push_vel(0,0)
    
    def constrain(self, val, bottom, top): return max(bottom,min(top,val))
    
    def get_grid(self, px, py): return (round(px/self.map_res),round(py/self.map_res))
    
    def filter_spin(self, trg):
        self.prev_w=self.constrain(self.prev_w+self.constrain(trg-self.prev_w,-self.w_step,self.w_step),-self.spin_max,self.spin_max)
        return self.prev_w
        
    def trigger_cd(self):
        self.cd_active = True
        self.cd_origin_x = self.curr_x
        self.cd_origin_y = self.curr_y

    def find_gaps(self):
        opts = []
        for d in [0, 180]:
            arr = self.get_slice(d, 24)
            if arr and sum(arr)/len(arr) >= self.gap_th:
                opts.append(wrap_rads(self.curr_h + math.radians(d)))
        for sd in [90, 270]:
            best_val = 0.0
            for os in [-10, 0, 10]:
                arr = self.get_slice(sd+os, 14)
                if arr:
                    val = sum(arr)/len(arr)
                    if val > best_val: best_val = val
            if best_val >= self.gap_th:
                rot = self.rad_limit if sd == 90 else -self.rad_limit
                opts.append(wrap_rads(self.curr_h + rot))
        return opts

    def exec_rot(self, trg_h, max_w):
        err = calc_offset(trg_h, self.curr_h)
        if abs(err) < 0.05:
            self.halt(); return True
        w_val = min(abs(err) * 1.5, max_w)
        if w_val < 0.08: w_val = 0.08
        self.push_vel(0, w_val if err > 0 else -w_val)
        return False

    def rec_char(self, trg_a):
        diff=math.degrees(calc_offset(trg_a,self.curr_h))
        self.char_hist.append('L' if diff>45 else 'R' if diff<-45 else 'S')
        
    def reduce_hist(self):
        mod=True
        while mod and len(self.char_hist)>=3:
            mod=False; chunk="".join(self.char_hist[-3:])
            rules={"LBL":"S","RBR":"S","LBS":"R","RBS":"L","SBL":"R","SBR":"L","LBR":"B","RBL":"B","SBS":"B"}
            if chunk in rules:
                self.char_hist=self.char_hist[:-3]+list(rules[chunk]); mod=True

    def eval_intersection(self, routes, origin_route):
        if self.is_reversing:
            self.is_reversing=False
            self.char_hist.append('B'); self.reduce_hist()
            ptr=self.active_idx
            if ptr is None:
                ptr={'uid':len(self.node_net),'open':routes.copy(),'up':None,
                      'src':origin_route,'dst':None,'nx':self.curr_x,'ny':self.curr_y}
                self.node_net.append(ptr); self.active_idx=ptr
        else:
            merge_th = 0.15
            found_ptr = None
            for p in self.node_net:
                if calc_hypot(self.curr_x, self.curr_y, p['nx'], p['ny']) < merge_th:
                    found_ptr = p
                    break

            if found_ptr:
                ptr = found_ptr
                self.active_idx = ptr
            else:
                ptr={'uid':len(self.node_net),'open':routes.copy(),'up':self.active_idx,
                      'src':origin_route,'dst':None,'nx':self.curr_x,'ny':self.curr_y}
                self.node_net.append(ptr); self.active_idx=ptr

        if not ptr['open']:
            self.node_net = []
            self.active_idx = None
            self.is_reversing = False
            self.char_hist = []
            
            top_h, top_w = routes[0], float('inf')
            for r in routes:
                w = self.heat_map.get(self.get_grid(self.curr_x+0.35*math.cos(r), self.curr_y+0.35*math.sin(r)), 0)
                if w < top_w: 
                    top_w, top_h = w, r
                    
            self.aim_h = top_h
            self.status = "STATE_ROT"
            return

        ok_routes = []
        for r in ptr['open']:
            if abs(calc_offset(r, origin_route)) < 0.5:
                continue
            for op in routes:
                if abs(calc_offset(r, op)) < 0.8:
                    ok_routes.append(r)
                    break
        
        if not ok_routes:
            ok_routes = [op for op in routes if abs(calc_offset(op, origin_route)) > 0.5]
        
        if not ok_routes:
            self.node_net = []
            self.active_idx = None
            self.is_reversing = False
            self.char_hist = []
            
            top_h, top_w = routes[0], float('inf')
            for r in routes:
                w = self.heat_map.get(self.get_grid(self.curr_x+0.35*math.cos(r), self.curr_y+0.35*math.sin(r)), 0)
                if w < top_w: 
                    top_w, top_h = w, r
                    
            self.aim_h = top_h
            self.status = "STATE_ROT"
            return

        top_h,top_w,top_idx=ok_routes[0],float('inf'),0
        for idx,r in enumerate(ok_routes):
            w=self.heat_map.get(self.get_grid(self.curr_x+0.35*math.cos(r),self.curr_y+0.35*math.sin(r)),0)
            if w<top_w: top_w,top_h,top_idx=w,r,idx
        
        for k, r in enumerate(ptr['open']):
            if abs(calc_offset(r, top_h)) < 0.8:
                ptr['open'].pop(k)
                break
        
        ptr['dst']=top_h
        self.rec_char(top_h); self.reduce_hist()
        self.aim_h=top_h
        self.status="STATE_ROT"

    def eval_heat(self, rel_d):
        th=self.curr_h+math.radians(rel_d); p=0.0
        for dx in (0.3,0.6,0.9):
            w=self.heat_map.get(self.get_grid(self.curr_x+dx*math.cos(th),self.curr_y+dx*math.sin(th)),0)
            p+=1.6 if w==0 else 0.6 if w<3 else 0.0 if w<7 else -1.2
        return p

    def scan_optimal_vec(self):
        top_w,top_a=-1e9,0
        for a in range(-95,96,5):
            c_a=a%360; mean=self.get_mean(c_a,14); p_min=self.get_min(c_a,10); w_min=self.get_min(c_a,24)
            scr=mean*1.8+p_min*0.8+w_min*1.8-abs(a)*0.015+self.eval_heat(a)
            if w_min<0.20:scr-=6.0
            elif w_min<0.26:scr-=3.0
            if p_min<0.16:scr-=2.5
            scr-=abs(a-self.prev_best_h)*0.005
            if scr>top_w:top_w,top_a=scr,a
        self.prev_best_h=top_a; return top_a,top_w

    def main_tick(self):
        if not self.laser_arr or self.is_done: return
        if self.goal_flag:
            self.halt(); self.is_done=True; return

        min_f, mean_f = self.fetch_front()
        min_l, min_r, min_fl, min_fr = self.fetch_sides()

        if self.status == "STATE_FREE":
            prog = calc_hypot(self.curr_x, self.curr_y, self.evd_x, self.evd_y)
            if prog >= 0.08:
                self.halt()
                self.status = "STATE_DRIVE"
                self.trigger_cd()
                self.time_mark = self.get_clock().now()
                return
            
            clear_b = self.get_mean(180, 20)
            if clear_b < 0.14 or clear_b >= 2:
                self.halt()
                self.status = "STATE_DRIVE"
                self.trigger_cd()
                self.time_mark = self.get_clock().now()
                return
            
            self.push_vel(self.vel_slow * self.evd_lin, self.evd_ang)
            return

        if self.status in ["STATE_ROT", "STATE_U"] or min_f < self.stop_th:
            stuck_t = (self.get_clock().now() - self.time_mark).nanoseconds / 1e9
            th_time = 50.0 if self.status == "STATE_U" else 25.0
            
            if stuck_t > th_time: 
                self.halt()
                self.evd_x = self.curr_x
                self.evd_y = self.curr_y
                
                if self.flag_reverse:
                    self.flag_reverse = False
                    self.is_reversing = True
                
                self.evd_lin = -1.0 
                if min_l < min_r:
                    self.evd_ang = -0.08 
                else:
                    self.evd_ang = 0.08
                    
                self.status = "STATE_FREE"
                return
        else:
            self.time_mark = self.get_clock().now()

        if self.status != "STATE_FREE":
            d_moved = calc_hypot(self.curr_x, self.curr_y, self.jam_x, self.jam_y)
            t_area = (self.get_clock().now() - self.jam_time).nanoseconds / 1e9
            
            if d_moved > self.jam_rad:
                self.jam_x = self.curr_x
                self.jam_y = self.curr_y
                self.jam_time = self.get_clock().now()
            else:
                th_area = 40.0 if self.status == "STATE_U" else 50.0 
                
                if t_area > th_area:
                    self.halt()
                    self.evd_x = self.curr_x
                    self.evd_y = self.curr_y
                    
                    if self.flag_reverse:
                        self.flag_reverse = False
                        self.is_reversing = True
                    
                    self.evd_lin = -1.0 
                    if min_l < min_r:
                        self.evd_ang = -0.08 
                    else:
                        self.evd_ang = 0.08
                        
                    self.status = "STATE_FREE"
                    
                    self.jam_x = self.curr_x
                    self.jam_y = self.curr_y
                    self.jam_time = self.get_clock().now()
                    return

        if self.status == "STATE_APPROACH":
            if min_l < min_r and min_l < 0.40:
                off_v = min_l - 0.155
            elif min_r < 0.40:
                off_v = 0.155 - min_r
            else:
                off_v = 0.0
                
            w_mod = self.constrain(off_v * 3.5, -0.40, 0.40)
            f_zero = self.get_min(0, 10)
            prg = calc_hypot(self.curr_x, self.curr_y, self.chk_x, self.chk_y)

            if prg < 0.04 and f_zero > 0.18:
                self.push_vel(self.vel_slow, w_mod); return

            self.halt()
            self.status = "STATE_SCAN"
            return

        if self.status == "STATE_SCAN":
            f_clear = self.get_mean(0, 15) > 0.29
            l_clear = self.get_mean(90, 15) > 0.35 or self.get_mean(45, 30) > 0.45
            r_clear = self.get_mean(270, 15) > 0.35 or self.get_mean(315, 30) > 0.45

            rel_opts = []
            if f_clear: rel_opts.append(0.0)
            if l_clear: rel_opts.append(self.rad_limit)
            if r_clear: rel_opts.append(-self.rad_limit)

            abs_opts = [wrap_rads(self.curr_h + deg) for deg in rel_opts]
            back_h = wrap_rads(self.curr_h + math.pi)

            if len(abs_opts) == 0:
                v_l = max(self.get_mean(90, 20), self.get_mean(45, 20))
                v_r = max(self.get_mean(270, 20), self.get_mean(315, 20))
                
                if v_l > 0.30 and v_l > (v_r + 0.10):
                    self.aim_h = wrap_rads(self.curr_h + self.rad_limit)
                    self.status = "STATE_ROT"
                elif v_r > 0.30 and v_r > (v_l + 0.10):
                    self.aim_h = wrap_rads(self.curr_h - self.rad_limit)
                    self.status = "STATE_ROT"
                elif v_l > 0.30 and v_r > 0.45:
                    self.aim_h = wrap_rads(self.curr_h + self.rad_limit) if v_l > v_r else wrap_rads(self.curr_h - self.rad_limit)
                    self.status = "STATE_ROT"
                else:
                    self.aim_h = wrap_rads(self.curr_h + math.pi)
                    self.trigger_cd()
                    self.status = "STATE_U"
                return

            if self.is_reversing:
                is_parent = False
                if self.active_idx:
                    d_p = calc_hypot(self.curr_x, self.curr_y, self.active_idx['nx'], self.active_idx['ny'])
                    if d_p < 0.25:
                        is_parent = True

                if self.active_idx is None or is_parent:
                    self.is_reversing = False
                    self.char_hist.append('B')
                    self.reduce_hist()
                    
                    if self.active_idx and len(self.active_idx['open']) > 0:
                        min_err = float('inf')
                        opt_h = back_h
                        opt_idx = -1
                        
                        for abs_h in abs_opts:
                            for idx, u_h in enumerate(self.active_idx['open']):
                                err = abs(calc_offset(abs_h, u_h))
                                if err < min_err:
                                    min_err = err
                                    opt_h = abs_h
                                    opt_idx = idx
                                    
                        if min_err < 0.8: 
                            self.active_idx['open'].pop(opt_idx)
                        else:
                            opt_h = abs_opts[-1] if len(abs_opts) > 1 else abs_opts[0]
                            
                        self.aim_h = opt_h
                        self.status = "STATE_ROT"
                    else:
                        self.node_net = []
                        self.active_idx = None
                        self.is_reversing = False
                        self.char_hist = []
                        
                        opt_h, min_w = abs_opts[0], float('inf')
                        for a in abs_opts:
                            w = self.heat_map.get(self.get_grid(self.curr_x+0.35*math.cos(a), self.curr_y+0.35*math.sin(a)), 0)
                            if w < min_w: min_w, opt_h = w, a
                                
                        self.aim_h = opt_h
                        self.status = "STATE_ROT"
                else:
                    self.is_reversing = False
                    back_ptr = self.curr_h
                    self.eval_intersection(abs_opts, back_ptr)
                return

            if len(abs_opts) == 1:
                if abs_opts[0] == 0.0:
                    self.status = "STATE_DRIVE"
                    self.trigger_cd()
                else:
                    self.aim_h = abs_opts[0]
                    self.status = "STATE_ROT"
            else:
                self.eval_intersection(abs_opts, back_h)

        if self.status == "STATE_ROT":
            if self.exec_rot(self.aim_h, self.spin_max):
                self.status = "STATE_DRIVE"
                self.trigger_cd()
                self.inside_junc = True
            return

        if self.status == "STATE_U":
            if self.exec_rot(self.aim_h, self.spin_max * 0.8):
                if self.flag_reverse:
                    self.flag_reverse = False
                    self.is_reversing = True
                self.status = "STATE_DRIVE"
                self.trigger_cd()
                self.inside_junc = True
            return

        if min_l < 0.30 and min_r < 0.30:
            self.inside_junc = False

        l_mov = self.get_mean(90, 20) > 0.35 or self.get_mean(45, 20) > 0.45
        r_mov = self.get_mean(270, 20) > 0.35 or self.get_mean(315, 20) > 0.45
        lat_sig = l_mov or r_mov

        if abs(self.prev_w) > 0.15: lat_sig = False
        
        if self.cd_active:
            cd_offset = calc_hypot(self.curr_x, self.curr_y, self.cd_origin_x, self.cd_origin_y)
            if cd_offset >= self.cd_dist:
                self.cd_active = False
                self.inside_junc = False 
            else: 
                lat_sig = False
                self.inside_junc = True
                
                if min_fl < 0.20 or min_l < 0.16:
                    defc = max(0.20 - min_fl, 0.16 - min_l)
                elif min_fr < 0.20 or min_r < 0.16:
                    defc = max(0.20 - min_fr, 0.16 - min_r)

        if lat_sig and not self.cd_active and not self.inside_junc:
            self.halt()
            self.status = "STATE_SCAN" 
            self.chk_x, self.chk_y = self.curr_x, self.curr_y
            self.inside_junc = True
            return

        elif not self.cd_active and min_f <= self.wall_gap and lat_sig:
            self.halt()
            self.status = "STATE_SCAN"
            self.chk_x, self.chk_y = self.curr_x, self.curr_y
            self.inside_junc = True
            return

        s_back = self.get_mean(180, 24)
        if min_f < 0.20 and min_l < 0.20 and min_r < 0.20 and s_back > 0.30:
            esc = False
            for dx in range(0, 360, 15):
                if 140 <= dx <= 220: continue
                if self.get_mean(dx, 14) > 0.30: esc = True; break
            if not esc:
                self.aim_h = wrap_rads(self.curr_h + math.pi)
                self.flag_reverse = True
                self.status = "STATE_U"; return

        if min_f < self.stop_th:
            w_mod = 1.0 if min_l > min_r else -1.0
            self.push_vel(0, w_mod * self.spin_max * 0.6); return

        trg = 0.155        
        trg_d = 0.220   
        viz_t = 0.32   
        
        cmp_fl = min(min_fl, 0.26)
        cmp_fr = min(min_fr, 0.26)
        
        err_w = 0.0

        if min_l < viz_t and min_r < viz_t:
            if min_l < min_r:
                err_w = (min_l - trg) * 3.0 + (cmp_fl - trg_d) * 1.5
            else:
                err_w = (trg - min_r) * 3.0 + (trg_d - cmp_fr) * 1.5
        elif min_l < viz_t:
            err_w = (min_l - trg) * 3.0 + (cmp_fl - trg_d) * 1.5
        elif min_r < viz_t:
            err_w = (trg - min_r) * 3.0 + (trg_d - cmp_fr) * 1.5

        if min_l < 0.115 or min_fl < 0.13:
            err_w = -0.6  
        elif min_r < 0.115 or min_fr < 0.13:
            err_w = 0.6   

        w_out = self.constrain(err_w, -0.45, 0.45)
        
        v_out = self.vel_fwd
        if min_f < self.slow_th or abs(w_out) > 0.20:
            v_out = self.vel_slow

        self.push_vel(v_out, w_out)

def main(args=None):
    rclpy.init(args=args)
    sys = CoreNavigator()
    try: rclpy.spin(sys)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok(): sys.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()