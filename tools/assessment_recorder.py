#!/usr/bin/env python3
"""Record a typed assessment run from an already-running ROS 2 graph."""

import argparse, csv, json, math, signal, subprocess, sys, time
from datetime import datetime, timezone
from pathlib import Path
import yaml
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from drone_msgs.msg import ControllerDiagnostics, MotorRPM, TrajectorySetpoint
from geometry_msgs.msg import PoseArray, PoseStamped, WrenchStamped
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String, UInt32
from visualization_msgs.msg import MarkerArray

from assessment_metrics import (ExperimentStopController, PathHistory,
    mission_relative_time, point_box_distance)

EXPERIMENTS = ("hover", "single_goal", "multi_goal", "navigation", "disturbance", "failure_case")
FIELDS = ("recording_time_s mission_time_s goal_x goal_y goal_z reference_x reference_y reference_z "
          "actual_x actual_y actual_z goal_error_x goal_error_y goal_error_z goal_position_error "
          "tracking_error_x tracking_error_y tracking_error_z tracking_error reference_acceleration_x "
          "reference_acceleration_y reference_acceleration_z reference_velocity_x "
          "reference_velocity_y reference_velocity_z reference_yaw velocity_x velocity_y "
          "velocity_z speed roll pitch yaw angular_speed_x angular_speed_y angular_speed_z m1_rpm "
          "m2_rpm m3_rpm m4_rpm horizontal_saturated altitude_saturated attitude_saturated "
          "mixer_saturated raw_obstacle_distance safety_clearance mission_waypoint_index "
          "mission_complete mission_success navigation_goal_index navigation_segment_index navigation_visited_goals "
          "navigation_complete navigation_success interactive_ready interactive_active "
          "interactive_goal_count collision_state external_wrench_active external_force_x "
          "external_force_y external_force_z integral_compensation_x integral_compensation_y").split()


def stamp_s(stamp): return stamp.sec + stamp.nanosec * 1e-9


def euler(q):
    n = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
    if not math.isfinite(n) or n < 1e-12: return math.nan, math.nan, math.nan
    x,y,z,w=q.x/n,q.y/n,q.z/n,q.w/n
    return (math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
            math.asin(max(-1,min(1,2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))


def git_state():
    try:
        commit=subprocess.check_output(["git","rev-parse","HEAD"],text=True).strip()
        dirty=bool(subprocess.check_output(["git","status","--porcelain"],text=True).strip())
        return commit, dirty
    except (OSError, subprocess.CalledProcessError): return None, None


def environment(path):
    data=yaml.safe_load(Path(path).read_text()); p=next(iter(data.values()))["ros__parameters"]
    values=p.get("obstacles",[]); boxes=[]
    for i in range(0,len(values),6):
        x,y,z,sx,sy,sz=map(float,values[i:i+6]); boxes.append(((x-sx/2,y-sy/2,z-sz/2),(x+sx/2,y+sy/2,z+sz/2)))
    return boxes,float(p["safety_radius"])

def configured_target(path):
    if not path:return None
    data=yaml.safe_load(Path(path).read_text())
    data=data.get("target",data) if isinstance(data,dict) else data
    if isinstance(data,dict):return [float(data[k]) for k in ("x","y","z")]
    if isinstance(data,(list,tuple)) and len(data)>=3:return list(map(float,data[:3]))
    raise ValueError("target config must contain x/y/z or a three-value list")


class Recorder:
    def __init__(self,node,args):
        self.node,self.args=node,args; self.output=Path(args.output); self.output.mkdir(parents=True,exist_ok=True)
        self.wall_start=time.monotonic(); self.record_start=None; self.mission_start=None; self.mission_time_source=None
        self.last_ros=None; self.current_goal=configured_target(args.target_config); self.final_goal=self.current_goal; self.reference=None; self.reference_velocity=None; self.reference_acceleration=None; self.reference_yaw=None; self.goals=[]
        self.rpm=self.diag=self.imu=None; self.force=[0.,0.,0.]; self.force_active=None
        self.mission_index=self.navigation_index=self.navigation_segment=self.visited=None
        self.mission_complete=self.mission_success=None; self.nav_complete=self.nav_success=None
        self.ready=self.active=self.goal_count=self.collision=None; self.editor_status=self.mission_status=""
        self.maximum_altitude=-math.inf; self.maximum_speed=0.; self.nonzero_rpm=False
        self.boxes,self.safety_radius=environment(args.environment_config); self.history=PathHistory()
        self.events=[]; self.counts={}; self.stop=False; self.failure_reason=None; self.goal_order=[]; self.activated_goals=set()
        self.controller=ExperimentStopController(args.experiment,args.steady_window,
            args.arrival_position_threshold,args.arrival_speed_threshold,args.arrival_hold_time,
            args.recovery_position_threshold,args.recovery_speed_threshold,args.recovery_hold_time,
            args.failure_observation_window)
        self.handle=(self.output/"samples.csv").open("w",newline=""); self.writer=csv.DictWriter(self.handle,fieldnames=FIELDS); self.writer.writeheader()
        self.subscriptions=[]; self.subscribe()

    def qos(self):
        return QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def sub(self,t,topic,cb,qos=20):
        self.subscriptions.append(self.node.create_subscription(t,topic,cb,qos)); self.counts[topic]=0
    def tick(self,topic): self.counts[topic]=self.counts.get(topic,0)+1
    def event(self,name,value=None,at=None):
        t=self.last_ros if at is None else at
        mission_time=mission_relative_time(t,self.mission_start)
        self.events.append({"recording_time_s":None if self.record_start is None or t is None else t-self.record_start,
          "mission_time_s":mission_time,
          "event":name,"details":value or {}})
    def changed(self,name,value):
        key="_event_"+name
        if getattr(self,key,object())!=value: setattr(self,key,value); self.event(name,{"value":value})
    def start_mission(self,source,at=None):
        if self.mission_start is None:
            self.mission_start=(self.last_ros if at is None else at); self.mission_time_source=source
            self.event("mission_started",{"source":source},self.mission_start)
    def activate_goal(self,index,source,at=None):
        if self.mission_start is None or index is None or index in self.activated_goals:return
        t=self.last_ros if at is None else at
        if t is None or t+1e-9<self.mission_start:return
        self.activated_goals.add(index);self.goal_order.append(index)
        self.event("goal_activated",{"goal_index":index,"source":source},t)

    def subscribe(self):
        latched=self.qos()
        self.sub(Odometry,"/drone/odom",self.on_odom,50); self.sub(Imu,"/drone/imu",self.on_imu,50)
        self.sub(MotorRPM,"/drone/motor_rpm_cmd",self.on_rpm); self.sub(ControllerDiagnostics,"/drone/controller/diagnostics",self.on_diag)
        self.sub(PoseStamped,"/drone/goal",self.on_goal); self.sub(TrajectorySetpoint,"/drone/trajectory_setpoint",self.on_reference)
        self.sub(NavPath,"/drone/path",lambda m:self.on_path("actual",m))
        for name in ("planned","simplified","reference"): self.sub(NavPath,f"/drone/{name}_path",lambda m,n=name:self.on_path(n,m),latched)
        self.sub(PoseArray,"/drone/mission/goals",lambda m:self.on_goals(m,"basic"),latched)
        self.sub(UInt32,"/drone/mission/current_waypoint_index",self.on_mission_index,latched)
        self.sub(Bool,"/drone/mission/complete",self.on_mission_complete,latched)
        self.sub(PoseArray,"/drone/interactive_goals/selected_goals",lambda m:self.on_goals(m,"navigation"),latched)
        self.sub(PoseStamped,"/drone/multi_goal/current_goal_pose",self.on_navigation_goal,latched)
        self.sub(UInt32,"/drone/multi_goal/current_goal_index",self.on_nav_index)
        self.sub(UInt32,"/drone/multi_goal/current_segment",self.on_nav_segment)
        self.sub(UInt32,"/drone/multi_goal/visited_goals",self.on_visited)
        self.sub(Bool,"/drone/multi_goal/complete",self.on_nav_complete); self.sub(Bool,"/drone/multi_goal/success",self.on_nav_success)
        self.sub(String,"/drone/interactive_goals/status",lambda m:self.on_status("editor",m.data),latched)
        self.sub(Bool,"/drone/interactive_goals/ready",self.on_ready,latched); self.sub(UInt32,"/drone/interactive_goals/count",self.on_count,latched)
        self.sub(String,"/drone/interactive_mission/status",lambda m:self.on_status("mission",m.data),latched)
        self.sub(Bool,"/drone/interactive_mission/active",self.on_active,latched)
        self.sub(Bool,"/drone/environment/in_collision",lambda m:self.on_bool("collision",m.data)); self.sub(MarkerArray,"/drone/environment/markers",lambda m:self.tick("/drone/environment/markers"),latched)
        self.sub(Bool,"/drone/external_wrench/active",lambda m:self.on_bool("force",m.data),latched)
        self.sub(WrenchStamped,"/drone/external_wrench/applied",self.on_force,latched)

    def on_imu(self,m): self.tick("/drone/imu"); self.imu=m
    def on_rpm(self,m):
        self.tick("/drone/motor_rpm_cmd"); self.rpm=m
        self.nonzero_rpm|=max(abs(m.m1_front_left_ccw_rpm),abs(m.m2_rear_left_cw_rpm),abs(m.m3_rear_right_ccw_rpm),abs(m.m4_front_right_cw_rpm))>1e-6
    def on_diag(self,m): self.tick("/drone/controller/diagnostics"); self.diag=m
    def on_goal(self,m):
        self.tick("/drone/goal"); self.current_goal=[m.pose.position.x,m.pose.position.y,m.pose.position.z]; self.final_goal=self.current_goal
        goal_time=stamp_s(m.header.stamp) or self.last_ros
        self.start_mission("goal_received",goal_time); self.event("goal_received",{"position":self.current_goal},goal_time)
    def on_reference(self,m):
        self.tick("/drone/trajectory_setpoint"); self.reference=[m.position.x,m.position.y,m.position.z]
        self.reference_velocity=[m.velocity.x,m.velocity.y,m.velocity.z]
        self.reference_acceleration=[m.acceleration.x,m.acceleration.y,m.acceleration.z]
        self.reference_yaw=m.yaw
        if self.args.experiment=="disturbance": self.current_goal=self.current_goal or self.reference; self.final_goal=self.current_goal; self.start_mission("trajectory_started",self.last_ros)
        self.changed("trajectory_started",True)
    def on_goals(self,m,kind):
        topic="/drone/mission/goals" if kind=="basic" else "/drone/interactive_goals/selected_goals"; self.tick(topic)
        points=[[p.position.x,p.position.y,p.position.z] for p in m.poses]
        if points: self.goals=points; self.final_goal=points[-1]; self.goal_count=len(points)
        if kind=="basic" and points and self.args.experiment=="multi_goal":
            mission_time=stamp_s(m.header.stamp) or self.last_ros
            self.start_mission("mission_goals_received",mission_time);self.event("mission_goals_received",{"goal_count":len(points)},mission_time)
            self.activate_goal(self.mission_index,"mission_waypoint_index",mission_time)
    def on_mission_index(self,m):
        self.tick("/drone/mission/current_waypoint_index"); self.mission_index=int(m.data); self.changed("waypoint_index_changed",self.mission_index)
        if self.goals and self.mission_index<len(self.goals): self.current_goal=self.goals[self.mission_index]
        self.activate_goal(self.mission_index,"mission_waypoint_index")
    def on_mission_complete(self,m): self.tick("/drone/mission/complete"); self.mission_complete=bool(m.data); self.changed("mission_complete_changed",self.mission_complete)
    def on_navigation_goal(self,m): self.tick("/drone/multi_goal/current_goal_pose"); self.current_goal=[m.pose.position.x,m.pose.position.y,m.pose.position.z]
    def on_nav_index(self,m):
        self.tick("/drone/multi_goal/current_goal_index"); self.navigation_index=int(m.data); self.changed("navigation_goal_index_changed",self.navigation_index)
        self.activate_goal(self.navigation_index,"navigation_goal_index")
    def on_nav_segment(self,m):
        self.tick("/drone/multi_goal/current_segment"); self.navigation_segment=int(m.data)
    def on_visited(self,m): self.tick("/drone/multi_goal/visited_goals"); self.visited=int(m.data); self.changed("navigation_visited_goals_changed",self.visited)
    def on_nav_complete(self,m): self.tick("/drone/multi_goal/complete"); self.nav_complete=bool(m.data); self.changed("navigation_complete_changed",self.nav_complete)
    def on_nav_success(self,m): self.tick("/drone/multi_goal/success"); self.nav_success=bool(m.data); self.changed("navigation_success_changed",self.nav_success)
    def on_ready(self,m): self.tick("/drone/interactive_goals/ready"); self.ready=bool(m.data); self.changed("interactive_ready_changed",self.ready)
    def on_count(self,m): self.tick("/drone/interactive_goals/count"); self.goal_count=int(m.data); self.changed("interactive_goal_count_changed",self.goal_count)
    def on_active(self,m):
        self.tick("/drone/interactive_mission/active"); self.active=bool(m.data)
        if self.active:self.start_mission("interactive_mission_active",self.last_ros)
        self.changed("interactive_active_changed",self.active)
        if self.active:self.activate_goal(self.navigation_index,"navigation_goal_index",self.last_ros)
    def on_status(self,kind,value):
        topic=f"/drone/interactive_{'goals' if kind=='editor' else 'mission'}/status"; self.tick(topic)
        if kind=="editor": self.editor_status=value; self.changed("editor_status_changed",value)
        else: self.mission_status=value; self.changed("mission_status_changed",value)
        upper=value.upper()
        if any(token in upper for token in ("REJECTED", "FAILED", "INVALID", "INSIDE ", "OUTSIDE ")):
            self.failure_reason=value; self.start_mission("failure_status_observed",self.last_ros)
    def on_bool(self,kind,value):
        topic="/drone/environment/in_collision" if kind=="collision" else "/drone/external_wrench/active"; self.tick(topic)
        if kind=="collision": self.collision=bool(value); self.changed("collision_changed",self.collision)
        else: self.force_active=bool(value)
    def on_force(self,m): self.tick("/drone/external_wrench/applied"); self.force=[m.wrench.force.x,m.wrench.force.y,m.wrench.force.z]
    def on_path(self,name,m):
        topic="/drone/path" if name=="actual" else f"/drone/{name}_path"; self.tick(topic)
        points=[[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in m.poses]
        recording_time=None if self.record_start is None or self.last_ros is None else self.last_ros-self.record_start
        mission_time=mission_relative_time(self.last_ros,self.mission_start)
        self.history.add(name,points,recording_time,mission_time,self.navigation_index if self.args.experiment=="navigation" else self.mission_index)

    def on_odom(self,m):
        self.tick("/drone/odom"); now=stamp_s(m.header.stamp); self.last_ros=now
        if self.record_start is None: self.record_start=now; self.event("recording_started",at=now)
        p=m.pose.pose.position; v=m.twist.twist.linear; w=m.twist.twist.angular; roll,pitch,yaw=euler(m.pose.pose.orientation)
        if self.imu: roll,pitch,yaw=euler(self.imu.orientation); w=self.imu.angular_velocity
        actual=[p.x,p.y,p.z]; speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z); self.maximum_altitude=max(self.maximum_altitude,p.z); self.maximum_speed=max(self.maximum_speed,speed)
        def err(target):
            if target is None:return [None]*3,None
            e=[target[i]-actual[i] for i in range(3)]; return e,math.sqrt(sum(x*x for x in e))
        ge,gerr=err(self.current_goal); te,terr=err(self.reference)
        raw=min((point_box_distance(actual,b) for b in self.boxes),default=None)
        row={k:"" for k in FIELDS}; row.update({"recording_time_s":now-self.record_start,"mission_time_s":"" if self.mission_start is None else now-self.mission_start,
          "actual_x":p.x,"actual_y":p.y,"actual_z":p.z,"velocity_x":v.x,"velocity_y":v.y,"velocity_z":v.z,"speed":speed,
          "roll":roll,"pitch":pitch,"yaw":yaw,"angular_speed_x":w.x,"angular_speed_y":w.y,"angular_speed_z":w.z,
          "raw_obstacle_distance":"" if raw is None else raw,"safety_clearance":"" if raw is None else raw-self.safety_radius})
        for prefix,target,error,value in (("goal",self.current_goal,ge,gerr),("reference",self.reference,te,terr)):
            if target is not None:
                row.update({f"{prefix}_x":target[0],f"{prefix}_y":target[1],f"{prefix}_z":target[2]})
            if prefix=="goal" and value is not None: row.update({"goal_error_x":error[0],"goal_error_y":error[1],"goal_error_z":error[2],"goal_position_error":value})
            if prefix=="reference" and value is not None: row.update({"tracking_error_x":error[0],"tracking_error_y":error[1],"tracking_error_z":error[2],"tracking_error":value})
        if self.reference_acceleration is not None:
            row.update(dict(zip(
                ("reference_acceleration_x", "reference_acceleration_y", "reference_acceleration_z"),
                self.reference_acceleration)))
        if self.reference_velocity is not None:
            row.update(dict(zip(
                ("reference_velocity_x", "reference_velocity_y", "reference_velocity_z"),
                self.reference_velocity)))
        if self.reference_yaw is not None:
            row["reference_yaw"] = self.reference_yaw
        state={"mission_waypoint_index":self.mission_index,"mission_complete":self.mission_complete,"mission_success":self.mission_success,
          "navigation_goal_index":self.navigation_index,"navigation_segment_index":self.navigation_segment,"navigation_visited_goals":self.visited,"navigation_complete":self.nav_complete,"navigation_success":self.nav_success,
          "interactive_ready":self.ready,"interactive_active":self.active,"interactive_goal_count":self.goal_count,"collision_state":self.collision,"external_wrench_active":self.force_active}
        row.update({k:"" if val is None else int(val) if isinstance(val,bool) else val for k,val in state.items()})
        if self.rpm: row.update(dict(zip(("m1_rpm","m2_rpm","m3_rpm","m4_rpm"),(self.rpm.m1_front_left_ccw_rpm,self.rpm.m2_rear_left_cw_rpm,self.rpm.m3_rear_right_ccw_rpm,self.rpm.m4_front_right_cw_rpm))))
        if self.diag: row.update({"horizontal_saturated":int(self.diag.horizontal_saturated),"altitude_saturated":int(self.diag.altitude_saturated),"attitude_saturated":int(self.diag.attitude_saturated),"mixer_saturated":int(self.diag.mixer_saturated),"integral_compensation_x":self.diag.horizontal_i_acceleration_x,"integral_compensation_y":self.diag.horizontal_i_acceleration_y})
        row.update({"external_force_x":self.force[0],"external_force_y":self.force[1],"external_force_z":self.force[2]}); self.writer.writerow(row)
        mt=None if self.mission_start is None else now-self.mission_start
        for ev in self.controller.update(mt or 0.,self.mission_start is not None,gerr,math.hypot(ge[0],ge[1]) if ge[0] is not None else None,speed,
          bool(self.mission_complete),bool(self.nav_complete),self.nav_success,self.active,self.force_active,self.failure_reason): self.event(ev)
        if self.controller.stopped: self.stop=True

    def finish(self):
        if not self.controller.stopped: self.controller.timeout()
        self.event("recording_stopped",{"reason":self.controller.stop_reason}); self.handle.close(); commit,dirty=git_state()
        metadata={"schema_version":3,"experiment":self.args.experiment,"status":self.args.run_status,"repository_commit":commit,"git_dirty":dirty,
          "generated_at":datetime.now(timezone.utc).isoformat(),"target_config":self.args.target_config,"mission_time_source":self.mission_time_source,"stop_reason":self.controller.stop_reason,"final_state":self.controller.state,
          "target_position":self.final_goal,"goals":self.goals,"goal_order":self.goal_order,"safety_radius_m":self.safety_radius,"failure_reason":self.controller.failure_reason or self.failure_reason,
          "safety_observations":{"maximum_altitude_m":self.maximum_altitude if math.isfinite(self.maximum_altitude) else None,"maximum_speed_m_s":self.maximum_speed,"nonzero_rpm_observed":self.nonzero_rpm},
          "thresholds":{"steady_window_s":self.args.steady_window,"arrival_position_threshold_m":self.args.arrival_position_threshold,"arrival_speed_threshold_m_s":self.args.arrival_speed_threshold,"arrival_hold_time_s":self.args.arrival_hold_time,
           "recovery_position_threshold_m":self.args.recovery_position_threshold,"recovery_speed_threshold_m_s":self.args.recovery_speed_threshold,"recovery_hold_time_s":self.args.recovery_hold_time,"failure_observation_window_s":self.args.failure_observation_window,"ground_motion_threshold_m":self.args.ground_motion_threshold,"timeout_s":self.args.timeout},"topic_message_counts":self.counts}
        (self.output/"metadata.json").write_text(json.dumps(metadata,indent=2,allow_nan=False)+"\n")
        with (self.output/"events.csv").open("w",newline="") as f:
            w=csv.DictWriter(f,fieldnames=("recording_time_s","mission_time_s","event","details")); w.writeheader()
            for e in self.events:w.writerow({**e,"details":json.dumps(e["details"],separators=(",",":"))})
        (self.output/"paths.json").write_text(json.dumps(self.history.as_dict(),indent=2)+"\n")
        (self.output/"recorder.log").write_text(f"stop_reason={self.controller.stop_reason}\nfinal_state={self.controller.state}\ncounts={json.dumps(self.counts,sort_keys=True)}\n")


def arguments():
    p=argparse.ArgumentParser(description=__doc__); p.add_argument("--experiment",choices=EXPERIMENTS,required=True); p.add_argument("--output",required=True); p.add_argument("--run-status",choices=("smoke","candidate","final"),default="candidate")
    p.add_argument("--target-config"); p.add_argument("--environment-config",default="src/drone_bringup/config/environment.yaml")
    for name,default in (("steady-window",3.),("arrival-position-threshold",.1),("arrival-speed-threshold",.08),("arrival-hold-time",1.),("recovery-position-threshold",.1),("recovery-speed-threshold",.08),("recovery-hold-time",1.),("failure-observation-window",2.),("ground-motion-threshold",.1),("timeout",120.)): p.add_argument("--"+name,type=float,default=default)
    a=p.parse_args()
    if any(not math.isfinite(v) or v<=0 for k,v in vars(a).items() if isinstance(v,float)):p.error("all numeric thresholds must be finite and positive")
    return a


def main():
    a=arguments(); rclpy.init(); n=rclpy.create_node("assessment_recorder"); rec=Recorder(n,a)
    def stop(*_): rec.stop=True
    signal.signal(signal.SIGINT,stop); signal.signal(signal.SIGTERM,stop)
    try:
        while rclpy.ok() and not rec.stop:
            rclpy.spin_once(n,timeout_sec=.1)
            if time.monotonic()-rec.wall_start>=a.timeout: rec.controller.timeout(); rec.stop=True
        return 0 if rec.counts["/drone/odom"] else 2
    finally: rec.finish(); n.destroy_node(); rclpy.shutdown()


if __name__=="__main__":sys.exit(main())
