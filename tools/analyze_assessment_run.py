#!/usr/bin/env python3
"""Compute fixed metrics and figures from a schema-v3 assessment recording."""
import argparse,csv,json,math
from pathlib import Path
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import yaml
from assessment_metrics import (directional_disturbance_metrics, goal_timing,
    held_condition_start, longest_true_duration, navigation_phase_start,
    path_length, phased_tracking_metrics, projection_overshoot,
    require_nonnegative_mission_times, wrap_to_pi, yaw_error_metrics)

def num(v):
    try:return float(v)
    except (TypeError,ValueError):return math.nan
def vals(xs):return [x for x in xs if math.isfinite(x)]
def avg(xs):
    x=vals(xs); return sum(x)/len(x) if x else None
def rms(xs):
    x=vals(xs); return math.sqrt(sum(v*v for v in x)/len(x)) if x else None
def minimum(xs):
    x=vals(xs); return min(x) if x else None
def maximum(xs):
    x=vals(xs); return max(x) if x else None
def read_csv(path):
    with path.open(newline="") as f:return [{k:num(v) for k,v in r.items()} for r in csv.DictReader(f)]
def commanded_rpm(row,index):
    return row.get(f"commanded_motor_rpm_m{index}",row.get(f"m{index}_rpm",math.nan))
def saturation_timeline(run,rows):
    path=run/"diagnostics.csv"
    if path.exists():
        diagnostics=[row for row in read_csv(path) if math.isfinite(row["mission_time_s"])]
        return ([row["mission_time_s"] for row in diagnostics],
                [row["any_saturated"]==1 for row in diagnostics],
                "diagnostics_callbacks")
    return ([row["mission_time_s"] for row in rows],
            [any(row[x]==1 for x in ("horizontal_saturated","altitude_saturated","attitude_saturated","mixer_saturated")) for row in rows],
            "legacy_odom_samples")
def last_window(rows,seconds):
    end=rows[-1]["mission_time_s"]; return [r for r in rows if r["mission_time_s"]>=end-seconds]
def segments(paths,name):return paths.get(name+"_segments",[])
def segments_length(items):return sum(path_length(s["points"]) for s in items) if items else None
def event_rows(path):
    with path.open(newline="") as f:
        result=[]
        for r in csv.DictReader(f):
            r["recording_time_s"]=num(r["recording_time_s"]);r["mission_time_s"]=num(r["mission_time_s"])
            try:r["details"]=json.loads(r["details"])
            except json.JSONDecodeError:r["details"]={}
            result.append(r)
        return result
def event_times(events,name):return [e["mission_time_s"] for e in events if e["event"]==name and math.isfinite(e["mission_time_s"])]
def observed(rows,key,value=True):return any(math.isfinite(r.get(key,math.nan)) and bool(r[key])==value for r in rows)
def target_snapshot_matches(meta,rows):
    targets=meta.get("targets")
    if not targets and meta.get("target_position") is not None:
        targets=[{"position":meta["target_position"],"yaw_rad":meta.get("target_yaw_rad")}]
    if not targets or not rows:return targets or [],False
    try:
        final=[float(value) for value in targets[-1]["position"]]
        observed_final=[rows[-1][f"goal_{axis}"] for axis in "xyz"]
        valid=len(final)==3 and all(math.isfinite(value) for value in final+observed_final)
        return targets,valid and all(abs(a-b)<=1e-6 for a,b in zip(final,observed_final))
    except (KeyError,TypeError,ValueError):return targets,False
def check(metric_name,actual,threshold,passed,source):
    return {"metric_name":metric_name,"actual_value":actual,"threshold":threshold,
            "passed":None if passed is None else bool(passed),"source":source}
def protocol_checks(experiment,metrics,meta,target_ok):
    checks={}
    def add(name,actual,threshold,passed,source):checks[name]=check(name,actual,threshold,passed,source)
    stop=meta.get("stop_reason")
    add("stop_reason_not_timeout",stop,"must not start with timeout",None if stop is None else not stop.startswith("timeout"),"metadata.json:stop_reason")
    add("finite_attitude",metrics.get("non_finite_attitude_count"),0,metrics.get("non_finite_attitude_count")==0,"samples.csv:roll,pitch,yaw")
    add("finite_commanded_rpm",metrics.get("non_finite_rpm_count"),0,metrics.get("non_finite_rpm_count")==0,"samples.csv:commanded_motor_rpm / legacy m*_rpm")
    divergence=metrics.get("attitude_divergence_detected")
    add("no_attitude_divergence",divergence,False,None if divergence is None else divergence is False,"analyzer:attitude duration")
    end_sat=metrics.get("saturated_at_end")
    add("not_saturated_at_end",end_sat,False,None if end_sat is None else end_sat is False,"diagnostics.csv:last mission callback / legacy Odom")
    if experiment in ("hover","single_goal"):
        add("final_position_error",metrics.get("final_position_error_m"),"< 0.10 m",metrics.get("final_position_error_m") is not None and metrics["final_position_error_m"]<.10,"samples.csv:last mission sample")
        add("final_speed",metrics.get("final_speed_m_s"),"< 0.08 m/s",metrics.get("final_speed_m_s") is not None and metrics["final_speed_m_s"]<.08,"samples.csv:last mission sample")
        add("correct_target_recorded",metrics.get("recorded_targets"),True,target_ok,"metadata.json target snapshot + samples.csv goal")
        add("arrival_and_steady_stop",stop,"arrival_and_steady_window_complete",stop=="arrival_and_steady_window_complete","metadata.json:stop_reason")
    elif experiment=="multi_goal":
        count=metrics.get("goal_count") or 0; expected=list(range(count)); order=metrics.get("goal_order")
        add("mission_complete",metrics.get("mission_complete"),True,metrics.get("mission_complete") is True,"events.csv + samples.csv")
        add("goal_visit_order",order,expected,order==expected and count>0,"samples.csv:mission_waypoint_index")
        add("final_position_error",metrics.get("final_position_error_m"),"< 0.10 m",metrics.get("final_position_error_m") is not None and metrics["final_position_error_m"]<.10,"samples.csv:last mission sample")
        add("final_speed",metrics.get("final_speed_m_s"),"< 0.08 m/s",metrics.get("final_speed_m_s") is not None and metrics["final_speed_m_s"]<.08,"samples.csv:last mission sample")
        timing=[metrics.get(name) for name in ("goal_activation_times_s","per_goal_arrival_times_s","per_goal_duration_s")]
        timing_ok=count>0 and all(isinstance(values,list) and len(values)==count and all(value is not None and math.isfinite(value) for value in values) for values in timing)
        add("complete_per_goal_timing",{"goal_count":count,"activation":timing[0],"arrival":timing[1],"duration":timing[2]},"all values available",timing_ok,"events.csv:goal_activated and completion")
    elif experiment in ("navigation","static_avoidance","narrow_corridor"):
        add("navigation_complete",metrics.get("navigation_complete"),True,metrics.get("navigation_complete") is True,"samples.csv:/drone/multi_goal/complete")
        add("navigation_success",metrics.get("navigation_success"),True,metrics.get("navigation_success") is True,"samples.csv:/drone/multi_goal/success")
        add("no_collision",metrics.get("collision_observed"),False,metrics.get("collision_observed") is False,"samples.csv:/drone/environment/in_collision")
        add("navigation_tracking_max_error",metrics.get("navigation_tracking_max_error_m"),"< 0.05 m",metrics.get("navigation_tracking_max_error_m") is not None and metrics["navigation_tracking_max_error_m"]<.05,"samples.csv:trajectory_setpoint tracking, navigation phase")
        add("minimum_safety_clearance",metrics.get("minimum_safety_clearance_m"),">= 0.085 m",metrics.get("minimum_safety_clearance_m") is not None and metrics["minimum_safety_clearance_m"]>=.085,"samples.csv:AABB distance - safety radius")
        add("final_position_error",metrics.get("final_position_error_m"),"< 0.05 m",metrics.get("final_position_error_m") is not None and metrics["final_position_error_m"]<.05,"samples.csv:last mission sample")
        add("final_speed",metrics.get("final_speed_m_s"),"< 0.03 m/s",metrics.get("final_speed_m_s") is not None and metrics["final_speed_m_s"]<.03,"samples.csv:last mission sample")
        add("zero_rpm_saturation_samples",metrics.get("saturation_sample_count"),0,metrics.get("saturation_sample_count")==0,"diagnostics.csv callbacks / legacy Odom")
    overall=bool(checks) and all(item["passed"] is True for item in checks.values())
    reasons=[f"{name}: actual={item['actual_value']!r}, required={item['threshold']!r}" for name,item in checks.items() if item["passed"] is not True]
    return checks,overall,reasons
def derived_paths(run,experiment):
    names=["summary.json","trajectory_xy.png","trajectory_3d.png","position_xyz.png","position_error.png","attitude.png","motor_rpm.png","yaw_tracking.png"]
    if experiment in ("multi_goal","navigation"):names += ["goal_progress.png","per_goal_error.png"]
    if experiment=="navigation":names += ["planned_simplified_reference_actual.png","tracking_error.png","navigation_tracking_error.png","obstacle_clearance.png"]
    if experiment=="disturbance":names += ["horizontal_error.png","external_force.png","integral_compensation.png","recovery.png"]
    if experiment=="failure_case":names += ["failure_timeline.png","safety_state.png"]
    return [run/name for name in names]
def plot(path,draw,xlabel="mission time (s)",ylabel=""):
    fig,ax=plt.subplots(figsize=(8,4.8));draw(ax);ax.set_xlabel(xlabel);ax.set_ylabel(ylabel);ax.grid(alpha=.3)
    if ax.get_legend_handles_labels()[1]:ax.legend();fig.tight_layout();fig.savefig(path,dpi=150);plt.close(fig)
def figures(run,rows,paths,experiment,navigation_start=None,force_start=None,force_release=None):
    t=[r["mission_time_s"] for r in rows]
    plot(run/"trajectory_xy.png",lambda a:(a.plot([r["actual_x"] for r in rows],[r["actual_y"] for r in rows],label="actual"),a.scatter([rows[-1]["goal_x"]],[rows[-1]["goal_y"]],marker="x",label="goal")),"x (m)","y (m)")
    fig=plt.figure(figsize=(8,5));a=fig.add_subplot(111,projection="3d");a.plot([r["actual_x"] for r in rows],[r["actual_y"] for r in rows],[r["actual_z"] for r in rows]);a.set(xlabel="x (m)",ylabel="y (m)",zlabel="z (m)");fig.tight_layout();fig.savefig(run/"trajectory_3d.png",dpi=150);plt.close(fig)
    plot(run/"position_xyz.png",lambda a:[a.plot(t,[r["actual_"+x] for r in rows],label=x) for x in "xyz"],ylabel="position (m)")
    plot(run/"position_error.png",lambda a:a.plot(t,[r["goal_position_error"] for r in rows],label="goal error"),ylabel="goal error (m)")
    plot(run/"attitude.png",lambda a:[a.plot(t,[r[x] for r in rows],label=x) for x in ("roll","pitch","yaw")],ylabel="angle (rad)")
    plot(run/"motor_rpm.png",lambda a:[a.plot(t,[commanded_rpm(r,i) for r in rows],label=f"commanded M{i}") for i in range(1,5)],ylabel="commanded RPM")
    yaw_key="reference_yaw" if experiment in ("navigation","static_avoidance","narrow_corridor") else "goal_yaw"
    yaw_rows=[r for r in rows if math.isfinite(r.get(yaw_key,math.nan)) and math.isfinite(r.get("yaw",math.nan))]
    if yaw_rows:
        plot(run/"yaw_tracking.png",lambda a:(a.plot([r["mission_time_s"] for r in yaw_rows],[r["yaw"] for r in yaw_rows],label="actual yaw"),a.plot([r["mission_time_s"] for r in yaw_rows],[r[yaw_key] for r in yaw_rows],label="reference yaw"),a.plot([r["mission_time_s"] for r in yaw_rows],[wrap_to_pi(r[yaw_key]-r["yaw"]) for r in yaw_rows],label="wrapped error")),ylabel="angle (rad)")
    if experiment in ("multi_goal","navigation"):
        index="mission_waypoint_index" if experiment=="multi_goal" else "navigation_goal_index"
        plot(run/"goal_progress.png",lambda a:a.step(t,[r[index] for r in rows],where="post"),ylabel="goal index")
        plot(run/"per_goal_error.png",lambda a:a.plot(t,[r["goal_position_error"] for r in rows]),ylabel="goal error (m)")
    if experiment=="navigation":
        def routes(a):
            colors={"planned":"tab:orange","simplified":"tab:green","reference":"tab:red"}
            for name in colors:
                for i,s in enumerate(segments(paths,name)):a.plot([p[0] for p in s["points"]],[p[1] for p in s["points"]],color=colors[name],label=name if i==0 else None)
            a.plot([r["actual_x"] for r in rows],[r["actual_y"] for r in rows],label="actual",color="tab:blue")
        plot(run/"planned_simplified_reference_actual.png",routes,"x (m)","y (m)")
        def tracking_plot(a):
            before=[r["tracking_error"] if navigation_start is not None and r["mission_time_s"]<navigation_start else math.nan for r in rows]
            after=[r["tracking_error"] if navigation_start is not None and r["mission_time_s"]>=navigation_start else math.nan for r in rows]
            a.plot(t,before,label="takeoff");a.plot(t,after,label="navigation")
            if navigation_start is not None:a.axvline(navigation_start,color="black",ls="--",label="navigation phase start")
        plot(run/"tracking_error.png",tracking_plot,ylabel="tracking error (m)")
        nav_rows=[r for r in rows if navigation_start is not None and r["mission_time_s"]>=navigation_start]
        plot(run/"navigation_tracking_error.png",lambda a:a.plot([r["mission_time_s"] for r in nav_rows],[r["tracking_error"] for r in nav_rows],label="navigation"),ylabel="tracking error (m)")
        plot(run/"obstacle_clearance.png",lambda a:(a.plot(t,[r["safety_clearance"] for r in rows]),a.axhline(0,color="red",ls="--")),ylabel="safety clearance (m)")
    if experiment=="disturbance":
        h=[math.hypot(r["goal_error_x"],r["goal_error_y"]) for r in rows];f=[math.hypot(r["external_force_x"],r["external_force_y"]) for r in rows]
        plot(run/"horizontal_error.png",lambda a:a.plot(t,h),ylabel="horizontal error (m)");plot(run/"external_force.png",lambda a:a.plot(t,f),ylabel="force (N)")
        plot(run/"integral_compensation.png",lambda a:[a.plot(t,[r["integral_compensation_"+x] for r in rows],label=x) for x in "xy"],ylabel="acceleration (m/s²)")
        samples=[{"actual":[r["actual_x"],r["actual_y"]],"goal":[r["goal_x"],r["goal_y"]],"force":[r["external_force_x"],r["external_force_y"]],"force_active":r["external_wrench_active"]==1} for r in rows]
        _,_,_,signed=directional_disturbance_metrics(samples)
        def recovery_plot(a):
            a.plot(t,h,label="horizontal error")
            if signed:a.plot(t,signed,label="signed force-direction displacement")
            a.axhline(0,color="black",ls=":",label="goal crossing")
            if force_start is not None:a.axvline(force_start,color="tab:red",ls="--",label="force start")
            if force_release is not None:a.axvline(force_release,color="tab:green",ls="--",label="force release")
        plot(run/"recovery.png",recovery_plot,ylabel="displacement (m)")
    if experiment=="failure_case":
        plot(run/"failure_timeline.png",lambda a:(a.step(t,[r["interactive_ready"] for r in rows],label="ready"),a.step(t,[r["interactive_active"] for r in rows],label="active")),ylabel="state")
        plot(run/"safety_state.png",lambda a:(a.plot(t,[r["actual_z"] for r in rows],label="altitude"),a.plot(t,[r["speed"] for r in rows],label="speed")),ylabel="m / m/s")

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("run",type=Path);p.add_argument("--parameters",type=Path,default=Path("results/parameters"));p.add_argument("--overwrite-existing",action="store_true");a=p.parse_args()
    meta=json.loads((a.run/"metadata.json").read_text())
    existing=[path for path in derived_paths(a.run,meta["experiment"]) if path.exists()]
    if existing and (meta.get("status")=="final" or not a.overwrite_existing):
        raise SystemExit("refusing to overwrite existing derived result files: "+", ".join(str(path) for path in existing))
    if meta.get("status")=="final" and a.overwrite_existing:raise SystemExit("--overwrite-existing is forbidden for final runs")
    rows_all=read_csv(a.run/"samples.csv");events=event_rows(a.run/"events.csv");paths=json.loads((a.run/"paths.json").read_text())
    if any(b["recording_time_s"]<=x["recording_time_s"] for x,b in zip(rows_all,rows_all[1:])):raise SystemExit("recording timestamps are not strictly increasing")
    try:
        require_nonnegative_mission_times([r["mission_time_s"] for r in rows_all+events])
        require_nonnegative_mission_times([item.get("mission_time_s") for name in ("planned","simplified","reference") for item in segments(paths,name)])
    except ValueError as error:raise SystemExit(str(error))
    mission_rows=[r for r in rows_all if math.isfinite(r["mission_time_s"])]
    rows=[r for r in mission_rows if math.isfinite(r["goal_position_error"])]
    if meta["experiment"]=="failure_case" and len(rows)<2: rows=mission_rows
    if len(rows)<2:raise SystemExit("fewer than two mission samples")
    has_goal=math.isfinite(rows[-1]["goal_position_error"])
    environment=yaml.safe_load((a.parameters/"environment.yaml").read_text());ep=next(iter(environment.values()))["ros__parameters"]
    controller=yaml.safe_load((a.parameters/"controller.yaml").read_text());cp=next(iter(controller.values()))["ros__parameters"]
    steady=last_window(rows,min(3.,rows[-1]["mission_time_s"]-rows[0]["mission_time_s"]));final=last_window(rows,1.)
    sample_pairs=[([r[f"actual_{x}"] for x in "xyz"],[r[f"goal_{x}"] for x in "xyz"]) for r in rows] if has_goal else []
    over,overpct=projection_overshoot(sample_pairs,meta["experiment"]=="hover") if sample_pairs else (None,None)
    rpm=[commanded_rpm(r,i) for r in rows for i in range(1,5)]
    saturation_times,sat,saturation_source=saturation_timeline(a.run,rows)
    times=[r["mission_time_s"] for r in rows];att_bad=[abs(r["roll"])>max(.5,2*cp["max_tilt_angle"]) or abs(r["pitch"])>max(.5,2*cp["max_tilt_angle"]) for r in rows]
    actual=[[r[f"actual_{x}"] for x in "xyz"] for r in rows];goal=[rows[-1][f"goal_{x}"] for x in "xyz"] if has_goal else None
    plan={name:segments(paths,name) for name in ("planned","simplified","reference")}; lengths={name:segments_length(plan[name]) for name in plan}
    tracking_samples=[(r["mission_time_s"],r["tracking_error"]) for r in rows]
    activation_events=[{"mission_time_s":e["mission_time_s"],**e["details"]} for e in events if e["event"]=="goal_activated"]
    navigation_start,navigation_source=navigation_phase_start(paths,activation_events) if meta["experiment"]=="navigation" else (None,None)
    full_tracking,takeoff_tracking,navigation_tracking=phased_tracking_metrics(tracking_samples,navigation_start)
    targets,target_ok=target_snapshot_matches(meta,rows)
    yaw_key="reference_yaw" if meta["experiment"] in ("navigation","static_avoidance","narrow_corridor") else "goal_yaw"
    yaw_rows=[r for r in rows if (navigation_start is None or meta["experiment"]!="navigation" or r["mission_time_s"]>=navigation_start)]
    yaw_stats=yaw_error_metrics([r.get("yaw",math.nan) for r in yaw_rows],[r.get(yaw_key,math.nan) for r in yaw_rows])
    raw=minimum([r["raw_obstacle_distance"] for r in rows]);safety=float(ep["safety_radius"])
    arrival=held_condition_start(times,[r["goal_position_error"]<meta["thresholds"]["arrival_position_threshold_m"] and r["speed"]<meta["thresholds"]["arrival_speed_threshold_m_s"] for r in rows],meta["thresholds"]["arrival_hold_time_s"]) if has_goal else None
    straight=math.dist(actual[0],goal) if goal else None
    metrics={"final_position_error_m":rows[-1]["goal_position_error"] if has_goal else None,"final_speed_m_s":rows[-1]["speed"],"final_window_mean_error_m":avg([r["goal_position_error"] for r in final]),
      "recorded_targets":targets,"target_snapshot_matches_samples":target_ok,
      "yaw_metrics_status":yaw_stats["status"],"yaw_sample_count":yaw_stats["sample_count"],"final_yaw_error_rad":yaw_stats["final_error_rad"],"yaw_error_rms_rad":yaw_stats["rms_error_rad"],"maximum_absolute_yaw_error_rad":yaw_stats["maximum_absolute_error_rad"],
      "arrival_time_s":arrival,
      "maximum_overshoot_m":over,"maximum_overshoot_percent":overpct,"steady_state_mean_error_m":avg([r["goal_position_error"] for r in steady]),"steady_state_rms_error_m":rms([r["goal_position_error"] for r in steady]),"steady_state_max_error_m":maximum([r["goal_position_error"] for r in steady]),
      "maximum_goal_position_error_m":maximum([r["goal_position_error"] for r in rows]),
      "full_mission_tracking_max_error_m":full_tracking["max_error_m"],"full_mission_tracking_rms_error_m":full_tracking["rms_error_m"],
      "minimum_raw_obstacle_distance_m":raw,"safety_radius_m":safety,"minimum_safety_clearance_m":None if raw is None else raw-safety,
      "straight_line_distance_m":straight,"actual_path_length_m":path_length(actual),"flight_time_s":times[-1]-times[0],
      "planned_segment_count":len(plan["planned"]),"simplified_segment_count":len(plan["simplified"]),"reference_segment_count":len(plan["reference"]),
      "planned_path_length_m":lengths["planned"],"simplified_path_length_m":lengths["simplified"],"reference_path_length_m":lengths["reference"],
      "path_efficiency":None if not lengths["reference"] else path_length(actual)/lengths["reference"],"planned_vs_straight_ratio":None if not lengths["planned"] or not straight else lengths["planned"]/straight,"actual_vs_reference_ratio":None if not lengths["reference"] else path_length(actual)/lengths["reference"],
      "maximum_absolute_roll_rad":maximum([abs(r["roll"]) for r in rows]),"maximum_absolute_pitch_rad":maximum([abs(r["pitch"]) for r in rows]),"maximum_angular_speed_rad_s":maximum([math.sqrt(sum(r[f"angular_speed_{x}"]**2 for x in "xyz")) for r in rows]),
      "non_finite_attitude_count":sum(not all(math.isfinite(r[x]) for x in ("roll","pitch","yaw")) for r in rows),"attitude_divergence_detected":longest_true_duration(times,att_bad)>=1.,
      "minimum_commanded_motor_rpm":minimum(rpm),"maximum_commanded_motor_rpm":maximum(rpm),
      "minimum_motor_rpm":minimum(rpm),"maximum_motor_rpm":maximum(rpm),
      "motor_rpm_semantics":"commanded_motor_rpm","saturation_sample_source":saturation_source,
      "saturation_sample_count":sum(sat),"longest_saturation_duration_s":longest_true_duration(saturation_times,sat),"saturated_at_end":sat[-1] if sat else None,"non_finite_rpm_count":sum(not math.isfinite(x) for x in rpm)}
    experiment=meta["experiment"]
    if experiment in ("multi_goal","navigation"):
        index="mission_waypoint_index" if experiment=="multi_goal" else "navigation_goal_index"; unique=[]
        for r in rows:
            if math.isfinite(r[index]) and int(r[index]) not in unique:unique.append(int(r[index]))
        per_errors=[];segment_overshoots=[]
        for i in unique:
            group=[r for r in rows if math.isfinite(r[index]) and int(r[index])==i];per_errors.append(group[-1]["goal_position_error"])
            pairs=[([r[f"actual_{x}"] for x in "xyz"],[r[f"goal_{x}"] for x in "xyz"]) for r in group]
            value,percent=projection_overshoot(pairs);segment_overshoots.append({"goal_index":i,"maximum_overshoot_m":value,"maximum_overshoot_percent":percent})
        completed=event_times(events,"mission_complete_changed" if experiment=="multi_goal" else "navigation_complete_changed")
        goal_count=len(meta.get("goals") or unique)
        relevant=[event for event in activation_events if event.get("source")== ("mission_waypoint_index" if experiment=="multi_goal" else "navigation_goal_index")]
        activations,arrivals,durations=goal_timing(goal_count,relevant,completed[-1] if completed else None)
        metrics.update({"goal_count":goal_count,"visited_goal_count":int(maximum([r["navigation_visited_goals"] for r in rows]) or len(unique)) if experiment=="navigation" else len(unique),"goal_order":unique,"goal_activation_times_s":activations,"per_goal_arrival_times_s":arrivals,"per_goal_duration_s":durations,"per_goal_final_errors_m":per_errors,"segment_overshoots":segment_overshoots,
          "mission_complete":observed(rows,"mission_complete") if experiment=="multi_goal" else observed(rows,"navigation_complete"),"mission_success":None if experiment=="multi_goal" else observed(rows,"navigation_success"),"total_mission_time_s":times[-1]})
        if experiment=="multi_goal":
            per_goal_yaw=[]
            for index,arrival_time in enumerate(arrivals):
                target_yaw=(targets[index].get("yaw_rad") if index<len(targets) and isinstance(targets[index],dict) else None)
                if arrival_time is None or target_yaw is None:
                    per_goal_yaw.append({"goal_index":index,"arrival_time_s":arrival_time,"status":"unavailable","yaw_error_rad":None});continue
                sample=min(rows,key=lambda row:abs(row["mission_time_s"]-arrival_time))
                error=wrap_to_pi(target_yaw-sample["yaw"])
                per_goal_yaw.append({"goal_index":index,"arrival_time_s":arrival_time,"status":"available" if error is not None else "unavailable","yaw_error_rad":error})
            metrics["per_goal_arrival_yaw_error"]=per_goal_yaw
    if experiment=="navigation":metrics.update({"takeoff_tracking_sample_count":takeoff_tracking["sample_count"],"takeoff_tracking_max_error_m":takeoff_tracking["max_error_m"],"takeoff_tracking_rms_error_m":takeoff_tracking["rms_error_m"],"navigation_tracking_sample_count":navigation_tracking["sample_count"],"navigation_tracking_max_error_m":navigation_tracking["max_error_m"],"navigation_tracking_rms_error_m":navigation_tracking["rms_error_m"],"navigation_tracking_final_error_m":navigation_tracking["final_error_m"],"navigation_yaw_tracking_status":yaw_stats["status"],"navigation_yaw_tracking_sample_count":yaw_stats["sample_count"],"navigation_yaw_tracking_final_error_rad":yaw_stats["final_error_rad"],"navigation_yaw_tracking_rms_error_rad":yaw_stats["rms_error_rad"],"navigation_yaw_tracking_maximum_absolute_error_rad":yaw_stats["maximum_absolute_error_rad"],"collision_observed":observed(rows,"collision_state"),"navigation_complete":observed(rows,"navigation_complete"),"navigation_success":observed(rows,"navigation_success"),"preflight_ready_observed":observed(rows,"interactive_ready"),"execution_active_observed":observed(rows,"interactive_active")})
    if experiment=="disturbance":
        start=event_times(events,"external_force_started");release=event_times(events,"external_force_released");recovery=event_times(events,"recovery_confirmed");start=start[0] if start else None;release=release[0] if release else None
        force_rows=[r for r in rows if start is not None and release is not None and start<=r["mission_time_s"]<=release];tail=force_rows[-max(1,int(len(force_rows)*.2)):] if force_rows else []
        horizontal=[math.hypot(r["goal_error_x"],r["goal_error_y"]) for r in rows]
        after=[r for r in rows if release is not None and r["mission_time_s"]>=release]
        recovery_start=held_condition_start([r["mission_time_s"] for r in after],[math.hypot(r["goal_error_x"],r["goal_error_y"])<meta["thresholds"]["recovery_position_threshold_m"] and r["speed"]<meta["thresholds"]["recovery_speed_threshold_m_s"] for r in after],meta["thresholds"]["recovery_hold_time_s"]) if after else None
        disturbance_samples=[{"actual":[r["actual_x"],r["actual_y"]],"goal":[r["goal_x"],r["goal_y"]],"force":[r["external_force_x"],r["external_force_y"]],"force_active":r["external_wrench_active"]==1} for r in rows]
        mean_force,peak_direction,reverse,_=directional_disturbance_metrics(disturbance_samples)
        metrics.update({"force_start_time_s":start,"force_release_time_s":release,"force_duration_s":None if start is None or release is None else release-start,"mean_horizontal_force_n":mean_force,"peak_horizontal_deviation_m":maximum(horizontal),"peak_force_direction_displacement_m":peak_direction,"disturbance_steady_state_error_m":avg([math.hypot(r["goal_error_x"],r["goal_error_y"]) for r in tail]),"recovery_time_s":None if recovery_start is None or release is None else recovery_start-release,"reverse_overshoot_m":reverse})
    if experiment=="failure_case":
        request=event_times(events,"mission_started");after=[r for r in rows if not request or r["mission_time_s"]>=0];maxalt=maximum([r["actual_z"] for r in after]);maxspeed=maximum([r["speed"] for r in after]);ground=meta["thresholds"]["ground_motion_threshold_m"]
        metrics.update({"failure_detected":bool(meta.get("failure_reason")),"failure_reason":meta.get("failure_reason"),"ready_ever_true":observed(rows,"interactive_ready"),"active_ever_true":observed(rows,"interactive_active"),"maximum_altitude_after_request_m":maxalt,"maximum_speed_after_request_m_s":maxspeed,"unsafe_motion_detected":bool((maxalt or 0)>ground or (maxspeed or 0)>ground),"collision_observed":observed(rows,"collision_state")})
    legacy_checks={"assignment_hover_error_pass":None if metrics["final_position_error_m"] is None else metrics["final_position_error_m"]<.3,"project_final_error_pass":None if metrics["final_position_error_m"] is None else metrics["final_position_error_m"]<.1,"finite_attitude_pass":metrics["non_finite_attitude_count"]==0,"finite_rpm_pass":metrics["non_finite_rpm_count"]==0,"attitude_stability_pass":not metrics["attitude_divergence_detected"],"rpm_end_saturation_pass":None if metrics["saturated_at_end"] is None else not metrics["saturated_at_end"]}
    if experiment=="navigation":legacy_checks["navigation_pass"]=metrics["navigation_complete"] and metrics["navigation_success"] and not metrics["collision_observed"] and metrics["minimum_safety_clearance_m"]>0 and metrics["navigation_tracking_max_error_m"] is not None
    if experiment=="failure_case":legacy_checks["failure_safety_pass"]=metrics["failure_detected"] and not metrics["active_ever_true"] and not metrics["unsafe_motion_detected"] and not metrics["collision_observed"]
    checks,overall_pass,failure_reasons=protocol_checks(experiment,metrics,meta,target_ok)
    gate_file="interactive_goal_executor.yaml" if experiment=="navigation" else "mission.yaml"
    gate_key="goal_yaw_tolerance" if experiment=="navigation" else "yaw_tolerance"
    try:gate_params=next(iter(yaml.safe_load((a.parameters/gate_file).read_text()).values()))["ros__parameters"];yaw_gate=gate_params[gate_key]
    except (OSError,KeyError,TypeError):yaw_gate=None
    summary={"schema_version":3,"experiment":experiment,"scenario_id":meta.get("scenario_id"),"status":meta["status"],"repository_commit":meta["repository_commit"],"sample_count":len(rows_all),"mission_sample_count":len(rows),"navigation_phase_start_time_s":navigation_start,"navigation_phase_start_source":navigation_source,"metrics":metrics,"yaw_analysis":{"status":yaw_stats["status"],"reference_field":yaw_key,"error_definition":"wrap_to_pi(reference_yaw - actual_yaw)","completion_gate_yaw_tolerance_rad":yaw_gate,"completion_gate_source":f"{gate_file}:{gate_key}","acceptance_threshold_applied":False},"assignment_thresholds":{"hover_final_position_error_max_m":.3},"project_thresholds":{"final_position_error_max_m":.1,"minimum_safety_clearance_strictly_greater_than_m":0.,"non_finite_value_count":0,"attitude_divergence_detected":False,"saturated_at_end":False},"legacy_checks":legacy_checks,"checks":checks,"overall_pass":overall_pass,"failure_reasons":failure_reasons}
    (a.run/"summary.json").write_text(json.dumps(summary,indent=2,allow_nan=False)+"\n");figures(a.run,rows,paths,experiment,navigation_start,metrics.get("force_start_time_s"),metrics.get("force_release_time_s"));print(json.dumps(metrics,indent=2))
if __name__=="__main__":main()
