import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from panda_simple_kinematics import PandaSimpleKinematics, rotate_vec_by_angle, rotate_xy_by_angle
import pandas as pd
from interpolator import HermiteQuinticInterpolatorT


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

    N = 100
    points = []    
    pt = np.array([0.4, 0, 0.5])    
    points.append(pt)
    for i in range(N):
        pt = planner.sample_near(pt, 0.1)
        points.append(pt)



    # create smooth path
    delta = 0.03333333 # secs
    transition_time = 3.0 # 2 secs
    waiting_time = 2.0

    xs = []
    ys = []
    zs = []
    dxs = []
    dys = []
    dzs = []
    ddxs = []
    ddys = []
    ddzs = []
    dddxs = []
    dddys = []
    dddzs = []

    for pt, nextpt in zip(points[:-1], points[1:]):
        print("Segment:", pt, nextpt)
        vec = nextpt - pt
        len_vec = np.linalg.norm(vec)
        norm_vec = vec / len_vec

        ax, ay, az = pt.tolist()
        bx, by, bz = vec.tolist()
        ix = HermiteQuinticInterpolatorT(transition_time, ax, bx)
        iy = HermiteQuinticInterpolatorT(transition_time, ay, by)
        iz = HermiteQuinticInterpolatorT(transition_time, az, bz)

        t = 0
        while t <= transition_time:
            x = ix.f(t)
            y = iy.f(t)
            z = iz.f(t)
            dx = ix.df(t)
            dy = iy.df(t)
            dz = iz.df(t)
            ddx = ix.ddf(t)
            ddy = iy.ddf(t)
            ddz = iz.ddf(t)
            dddx = ix.dddf(t)
            dddy = iy.dddf(t)
            dddz = iz.dddf(t)
            xs.append(x)
            ys.append(y)
            zs.append(z)
            dxs.append(dx)
            dys.append(dy)
            dzs.append(dz)
            ddxs.append(ddx)
            ddys.append(ddy)
            ddzs.append(ddz)
            dddxs.append(dddx)
            dddys.append(dddy)
            dddzs.append(dddz)
            t += delta

        t = 0
        while t <= waiting_time:
            xs.append(x)
            ys.append(y)
            zs.append(z)
            dxs.append(dx)
            dys.append(dy)
            dzs.append(dz)
            ddxs.append(ddx)
            ddys.append(ddy)
            ddzs.append(ddz)
            dddxs.append(dddx)
            dddys.append(dddy)
            dddzs.append(dddz)
            t += delta


    # use inverse kinematics to plan joint-space trajectory for Panda robot...
    qs = [psk.make_qvec(*psk.inverse_kinematics(*pt)) for pt in zip(xs, ys, zs)]
    qs = np.array(qs)

    # ??? strange results? not correctly used...
    dqs = np.diff(qs, n=1, append=0) / delta
    ddqs = np.diff(dqs, n=1, append=0) / delta
    dddqs = np.diff(ddqs, n=1, append=0) / delta

    # this seems to be correct...
    dqs = []
    for q0, q1 in zip(qs[:-1], qs[1:]):
        dq = (q1-q0)/delta
        dqs.append(dq)
    dqs.append(np.zeros(7))
    dqs = np.array(dqs)

    ddqs = []
    for q0, q1 in zip(dqs[:-1], dqs[1:]):
        ddq = (q1-q0)/delta
        ddqs.append(ddq)
    ddqs.append(np.zeros(7))
    ddqs = np.array(ddqs)

    dddqs = []
    for q0, q1 in zip(ddqs[:-1], ddqs[1:]):
        dddq = (q1-q0)/delta
        dddqs.append(dddq)
    dddqs.append(np.zeros(7))
    dddqs = np.array(dddqs)


    qsdf = []
    time_ns = 0
    for i, q in enumerate(qs):
        print(q)
        row = dict(time_ns=time_ns)
        for j in range(7):
            row[f"panda_joint{j+1}"] = q[j]
        qsdf.append(row)
        time_ns += int(delta * 1e9)
    qsdf = pd.DataFrame(qsdf)
    qsdf.to_excel("/home/ros/dumps/q_generated_100.xlsx", index=False)
    qsdf.to_csv("/home/ros/dumps/q_generated_100.csv", index=False)


    fig = plt.figure()
    
    # syntax for 3-D projection
    ax = plt.axes(projection ='3d')
        
    # plotting
    ax.plot3D(xs, ys, zs, 'green')
    plt.show()

    ts = np.arange(len(zs)) * delta

    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1)
    fig.suptitle('Planned Cartesian Trajectory')

    ax1.plot(ts, xs, color="r", label="x")
    ax1.plot(ts, ys, color="g", label="y")
    ax1.plot(ts, zs, color="b", label="z")
    ax1.legend(loc="upper right")

    ax2.plot(ts, dxs, color="r", label="dx")
    ax2.plot(ts, dys, color="g", label="dy")
    ax2.plot(ts, dzs, color="b", label="dz")
    ax2.legend(loc="upper right")

    ax3.plot(ts, ddxs, color="r", label="ddx")
    ax3.plot(ts, ddys, color="g", label="ddy")
    ax3.plot(ts, ddzs, color="b", label="ddz")
    ax3.legend(loc="upper right")

    ax4.plot(ts, dddxs, color="r", label="dddx")
    ax4.plot(ts, dddys, color="g", label="dddy")
    ax4.plot(ts, dddzs, color="b", label="dddz")
    ax4.legend(loc="upper right")

    plt.show()

    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1)
    fig.suptitle('Planned Joint Trajectory')

    ax1.plot(ts, qs[:,0], color="r", label = "q1")
    ax1.plot(ts, qs[:,1], color="g", label = "q2")
    ax1.plot(ts, qs[:,2], color="b", label = "q3")
    ax1.plot(ts, qs[:,3], color="y", label = "q4")
    ax1.plot(ts, qs[:,4], color="k", label = "q5")
    ax1.plot(ts, qs[:,5], color="cyan", label = "q6")
    ax1.plot(ts, qs[:,6], color="orange", label = "q7")
    ax1.legend(loc="upper right")

    ax2.plot(ts, dqs[:,0], color="r", label = "dq1")
    ax2.plot(ts, dqs[:,1], color="g", label = "dq2")
    ax2.plot(ts, dqs[:,2], color="b", label = "dq3")
    ax2.plot(ts, dqs[:,3], color="y", label = "dq4")
    ax2.plot(ts, dqs[:,4], color="k", label = "dq5")
    ax2.plot(ts, dqs[:,5], color="cyan", label = "dq6")
    ax2.plot(ts, dqs[:,6], color="orange", label = "dq7")
    ax2.legend(loc="upper right")

    ax3.plot(ts, ddqs[:,0], color="r", label = "ddq1")
    ax3.plot(ts, ddqs[:,1], color="g", label = "ddq2")
    ax3.plot(ts, ddqs[:,2], color="b", label = "ddq3")
    ax3.plot(ts, ddqs[:,3], color="y", label = "ddq4")
    ax3.plot(ts, ddqs[:,4], color="k", label = "ddq5")
    ax3.plot(ts, ddqs[:,5], color="cyan", label = "ddq6")
    ax3.plot(ts, ddqs[:,6], color="orange", label = "ddq7")
    ax3.legend(loc="upper right")

    ax4.plot(ts, dddqs[:,0], color="r", label = "dddq1")
    ax4.plot(ts, dddqs[:,1], color="g", label = "dddq2")
    ax4.plot(ts, dddqs[:,2], color="b", label = "dddq3")
    ax4.plot(ts, dddqs[:,3], color="y", label = "dddq4")
    ax4.plot(ts, dddqs[:,4], color="k", label = "dddq5")
    ax4.plot(ts, dddqs[:,5], color="cyan", label = "dddq6")
    ax4.plot(ts, dddqs[:,6], color="orange", label = "dddq7")
    ax4.legend(loc="upper right")
    
    plt.show()
