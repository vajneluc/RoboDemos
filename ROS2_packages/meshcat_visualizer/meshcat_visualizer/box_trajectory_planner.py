import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from panda_simple_kinematics import PandaSimpleKinematics, rotate_vec_by_angle, rotate_xy_by_angle
import pandas as pd
 

class BoxTrajectoryPlanner(object):
    def __init__(self, minx, maxx, miny, maxy, minz, maxz):
        self.minx = minx
        self.miny = miny
        self.minz = minz
        self.maxx = maxx
        self.maxy = maxy
        self.maxz = maxz

    def __repr__(self):
        return f"BoxTrajectoryPlanner(x=<{self.minx}, {self.maxx}> y=<{self.miny}, {self.maxy}> z=<{self.minz}, {self.maxz}>)"
    
    def sample(self):
        x = self.minx + np.random.random()* (self.maxx - self.minx)
        y = self.miny + np.random.random()* (self.maxy - self.miny)
        z = self.minz + np.random.random()* (self.maxz - self.minz)
        return np.array([x,y,z])

    def sample_near(self, pt, radius):
        r = radius * np.random.random()
        phi = 2*np.pi * np.random.random()
        theta = np.pi * np.random.random() - np.pi/2
        z = pt[2] + radius * np.sin(theta)
        rr = radius * np.cos(theta)
        x = pt[0] + rr * np.cos(phi)
        y = pt[1] + rr * np.sin(phi)

        x = max(min(x, self.maxx), self.minx)
        y = max(min(y, self.maxy), self.miny)
        z = max(min(z, self.maxz), self.minz)
        return np.array([x,y,z])



"""
https://frankaemika.github.io/docs/control_parameters.html

Limits in the Cartesian space are as follows:

Velocity: p' max: Translation 1.7 ms-1, Rotation 2.5 rads-1, Elbow 2.1750 rads-1
Acceleration: p'' max: Translation 13.0 ms-1, Rotation 25.0 rads-2, Elbow 10.0 rads-2
Jerk: p'''max: Translation 6500.0 ms-3, Rotation 12500.0 rads-3, Elbow 5000.0 rads-3

(Elbow = Joint4)

"""


if __name__ == "__main__":

    planner = BoxTrajectoryPlanner(0.3, 0.6, -0.5, 0.5, 0.1, 0.6)
    psk = PandaSimpleKinematics()

    print(planner)

    N = 100
    points = []    
    pt = np.array([0.4, 0, 0.5])    
    points.append(pt)
    for i in range(N):
        print(pt)
        pt = planner.sample_near(pt, 0.1)
        points.append(pt)



    # create smooth path
    delta = 0.01 # 1cm

    s = 0
    current_s = 0
    sampled_points = []
    for pt, nextpt in zip(points[:-1], points[1:]):
        print("Segment:", pt, nextpt)
        vec = nextpt - pt
        len_vec = np.linalg.norm(vec)
        norm_vec = vec / len_vec
        while current_s < len_vec:
            current_pt = pt + current_s * norm_vec
            sampled_points.append(current_pt)
            current_s += delta
            s += delta
        if current_s > len_vec:
            current_s -= len_vec

    # use inverse kinematics to plan joint-space trajectory for Panda robot...
    qs = [psk.make_qvec(*psk.inverse_kinematics(*pt.tolist())) for pt in sampled_points]

    qsdf = []
    delta_t = 0.05 # 50 ms
    time_ns = 0
    for i, q in enumerate(qs):
        print(q)
        row = dict(time_ns=time_ns)
        for j in range(7):
            row[f"panda_joint{j+1}"] = q[j]
        qsdf.append(row)
        time_ns += int(delta_t * 1e9)
    qsdf = pd.DataFrame(qsdf)
    qsdf.to_excel("/home/ros/dumps/q_generated_100.xlsx", index=False)
    qsdf.to_csv("/home/ros/dumps/q_generated_100.csv", index=False)


    print("Number of points in trajectory:", len(qs))


    fig = plt.figure()
    
    # syntax for 3-D projection
    ax = plt.axes(projection ='3d')
    
    # defining all 3 axis
    z = [pt[2] for pt in points]
    x = [pt[0] for pt in points]
    y = [pt[1] for pt in points]
    
    # plotting
    ax.plot3D(x, y, z, 'green')
    plt.show()

    fig = plt.figure()
    
    # syntax for 3-D projection
    ax = plt.axes(projection ='3d')
    
    # defining all 3 axis
    z = [pt[2] for pt in sampled_points]
    x = [pt[0] for pt in sampled_points]
    y = [pt[1] for pt in sampled_points]
    
    # plotting
    ax.plot3D(x, y, z, 'green')
    plt.show()
