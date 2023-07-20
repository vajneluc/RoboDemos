import numpy as np
import math


# h = ax + bx * cos(A) - by * sin(A) + cx * cos(A-B) - cy * sin(A-B) + dx
# r = ay + bx * sin(A) + by * cos(A) + cx * sin(A-B) + cy * cos(A-B) + dy

# H = h - ax - dx = bx * cos(A) - by * sin(A) + cx * cos(A-B) - cy * sin(A-B)
# R = r - ay - dy = bx * sin(A) + by * cos(A) + cx * sin(A-B) + cy * cos(A-B)

# H = bx * cos(A) - by * sin(A) + cx * cos(A-B) - cy * sin(A-B)
# R = bx * sin(A) + by * cos(A) + cx * sin(A-B) + cy * cos(A-B)

# ?? express A, B based on (H, R)
# Substitute: D = A-B

# H = bx * cos(A) - by * sin(A) + cx * cos(D) - cy * sin(D)
# R = bx * sin(A) + by * cos(A) + cx * sin(D) + cy * cos(D)

# Solve simpler problem: find location of point P=[x,y] where |P-O| = b and |P - (R,H)| = c, where b = sqrt(bx^2 + by^2), c=sqrt(cx^2 + cy^2)


def rotate_vec_by_angle(vec, theta):
    rot = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    return np.dot(rot, vec)

def rotate_xy_by_angle(x, y, theta):
    u = x * math.cos(theta) - y * math.sin(theta)
    v = y * math.sin(theta) + y * math.cos(theta)
    return u, v


class PandaSimpleKinematics(object):
    def __init__(self):
        self.link_a = np.array([0.333, 0])
        self.link_b = np.array([0.316, 0.0825])
        self.link_c = np.array([0.384, -0.0825])
        self.link_d = np.array([-0.107, 0.088]) 

        self.link_b_angle = math.atan2(self.link_b[1], self.link_b[0])
        self.link_c_angle = math.atan2(self.link_c[1], self.link_c[0])
        self.link_d_angle = math.atan2(self.link_d[1], self.link_d[0])

    def compute_AB_from_hr(self, h, r):
        b = np.linalg.norm(self.link_b)
        c = np.linalg.norm(self.link_c)
        H = h - self.link_a[0] - self.link_d[0]
        R = r - self.link_a[1] - self.link_d[1]

        # position of point P in hr plane...
        px = (-H*(H**3 + H * R**2 + H * b**2 - H * c**2 + math.sqrt(-H**4 * R**2 - 2 * H**2 * R**4 + 2* H**2 * R**2 * b**2 + 2 * H**2 * R**2 * c**2 - R**6 + 2 * R**4 * b**2 + 2 * R**4 * c**2 - R**2 * b**4 + 2 * R**2 * b**2 * c**2 - R**2 * c**4))/(H**2 + R**2) + H**2 + R**2 + b**2 - c**2)/(2*R)
        # y = math.sqrt(b**2 - x**2) 
        py = (H**3 + H * R**2 + H * b**2 - H*c**2 + math.sqrt(-H**4 * R**2 - 2*H**2 * R**4 + 2* H**2 * R**2 * b**2 + 2 * H**2 * R**2 * c**2 - R**6 + 2 * R**4 * b**2 + 2 * R**4 * c**2 - R**2 * b**4 + 2 * R**2 * b**2 * c**2 - R**2 * c**4))/(2*(H**2 + R**2))
        py += self.link_a[0]    
        
        Ahat = math.asin(px / b) - self.link_b_angle 
        Dhat = math.pi - math.asin((R - px)/c) - self.link_c_angle
        Bhat = Ahat - Dhat
        return Ahat, Bhat, px, py
    
    def compute_hr_from_AB(self, A, B):
        theta1 = A
        theta2 = A-B
        h = self.link_a[0] + self.link_b[0] * math.cos(theta1) - self.link_b[1] * math.sin(theta1) + self.link_c[0] * math.cos(theta2) - self.link_c[1] * math.sin(theta2) + self.link_d[0]
        r = self.link_a[1] + self.link_b[0] * math.sin(theta1) + self.link_b[1] * math.cos(theta1) + self.link_c[0] * math.sin(theta2) + self.link_c[1] * math.cos(theta2) + self.link_d[1]
        return h, r

    def forward_kinematics(self, alpha, A, B):
        h, r = self.compute_hr_from_AB(A, B)
        x = r * math.cos(alpha)
        y = r * math.sin(alpha)
        return x, y, h

    def inverse_kinematics(self, x, y, z):
        alpha = math.atan2(y, x)
        r = math.sqrt(x*x + y*y)
        h = z
        A, B, _, _ = self.compute_AB_from_hr(h, r)
        return alpha, A, B

    def make_qvec(self, alpha, A, B):
        C = A - B
        delta = alpha + 0.7853981633974483
        beta = 0
        gamma = 0
        return [alpha, A, beta, B, gamma, C, delta]



 
