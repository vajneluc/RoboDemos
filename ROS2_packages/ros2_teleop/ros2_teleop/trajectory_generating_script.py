import pandas as pd
import sys
from interpolator import InterpolatorT, HermiteQuinticInterpolatorT

default_waypoints_csv_path = "/home/ros/dumps/waypoints.csv"

if len(sys.argv) > 1:
    waypoints_csv_path = sys.argv[1]
else:
    waypoints_csv_path = default_waypoints_csv_path
    
q_table_path = "/home/ros/dumps/generated/q.csv"
dq_table_path ="/home/ros/dumps/generated/dq.csv"
ddq_table_path = "/home/ros/dumps/generated/ddq.csv"


start_stop_index = list(input("Enter ROW index of start and stop waypoints or press ENTER to continue:\n").split())
if len(start_stop_index) == 0:
    start_index = 0
    stop_index = 1
    
else:
    start_index = int(start_stop_index[0])
    stop_index = int(start_stop_index[1])

print(f"Start Waypoint Row = {start_index}\nStop Waypoint Row = {stop_index}")

df = pd.read_csv(waypoints_csv_path)

start_waypoint_configuration = []
stop_waypoint_configuration = []

names_list = ["panda_joint1", 
              "panda_joint2",
              "panda_joint3",
              "panda_joint4", 
              "panda_joint5", 
              "panda_joint6", 
              "panda_joint7"]

for index, row in df.iterrows():
    if index == start_index:
        start_waypoint_configuration = [row[x] for x in names_list]
    if index == stop_index:
        stop_waypoint_configuration = [row[x] for x in names_list]

print(f"start: {start_waypoint_configuration},\nstop: {stop_waypoint_configuration}")
# we have 2 waypoints - now create interpolators for them...
interpolators = []
T = 10.0 # 10 seconds
for i in range(7):
    q_init = start_waypoint_configuration[i]
    q_target = stop_waypoint_configuration[i]
    a = q_init
    b = q_target - q_init
    interp = HermiteQuinticInterpolatorT(T, a, b)
    interpolators.append(interp)

# use the interpolators to create interpolated trajectory
ts_generated = [] # timestamps in ns, with some start time
q_generated = []
dq_generated = []
ddq_generated = []

generate_period = 0.03
t_start_s = 0.0
t_current_s = t_start_s

while t_current_s < t_start_s + T:
    t_current_ns = int(t_current_s * 1e9)
    t = t_current_s - t_start_s
    # interpolate values
    q_curr = [0] * 7
    dq_curr = [0] * 7
    ddq_curr = [0] * 7
    for i in range(7):
        q_curr[i] = interpolators[i].f(t)
        dq_curr[i] = interpolators[i].df(t)
        ddq_curr[i] = interpolators[i].ddf(t)
    ts_generated.append(t_current_ns)
    q_generated.append([t_current_ns]+ q_curr)
    dq_generated.append([t_current_ns]+ dq_curr)
    ddq_generated.append([t_current_ns]+ ddq_curr)
    t_current_s += generate_period

# save CSV
table_header = ["time_ns"] + names_list
q_df = pd.DataFrame(q_generated, columns=table_header)
dq_df = pd.DataFrame(dq_generated, columns=table_header)
ddq_df = pd.DataFrame(ddq_generated, columns=table_header)


q_df.to_csv(q_table_path, index=False)
dq_df.to_csv(dq_table_path, index=False)
ddq_df.to_csv(ddq_table_path, index=False)



