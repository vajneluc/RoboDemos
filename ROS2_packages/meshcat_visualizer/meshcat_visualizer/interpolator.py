import numpy as np

class InterpolatorT(object):
    def __init__(self, T, a=0, b=1):
        self.T = T  # period
        self.a = a  # baseline:   f(0) = a
        self.b = b  # multiplier: f(T) = a+b

    def f(self, t):
        x = t / self.T
        if x < 0:
            return self.a
        if x > 1:
            return self.a+self.b
        return self.a + self.b*x
    
    def df(self, t):
        if t < 0 or t > self.T:
            return 0
        return self.b/self.T
    
    def ddf(self, t):
        if t == 0 or t == self.T:
            return np.nan  # not continuous!
        return 0 
    
    def dddf(self, t):
        if t == 0 or t == self.T:
            return np.nan  # not continuous!
        return 0 

    
class SinusoidInterpolatorT(InterpolatorT):
    def f(self, t):
        x = t/self.T
        return self.a + self.b*(1 - np.cos(x*np.pi))/2

    def df(self, t):
        x = t/self.T
        return self.b*(np.pi/self.T)*np.sin(x*np.pi)/2
    
    def ddf(self, t):
        x = t/self.T
        return self.b*(np.pi/self.T)**2 * np.cos(x*np.pi)/2

    def dddf(self, t):
        x = t/self.T
        return -self.b*(np.pi/self.T)**3 * np.sin(x*np.pi)/2
    
    
class HermiteQuinticInterpolatorT(InterpolatorT):
    """
    This interpolator has: f(0)=0, f(1)=1, f'(0)=f'(1)=f''(0)=f''(1)=0
    """
    def f(self, t):
        x = t/self.T
        return self.a + self.b*(10*x**3 - 15*x**4 + 6*x**5)

    def df(self, t):
        x = t/self.T
        return self.b*(30*x**2 - 60*x**3 + 30*x**4)/self.T

    def ddf(self, t):
        x = t/self.T
        return self.b*(60*x - 180*x**2 + 120*x**3)/(self.T**2)  # root: x=1/2 (with maximum of df)
    
    def dddf(self, t):
        x = t/self.T
        return self.b*(60 - 360*x + 360*x**2)/(self.T**3)  # roots: x=(3 +- sqrt(3))/6 -> find points with max and min accel
    
    def maxdf(self):
        return self.df(self.T/2)
    
    def maxddf(self):
        return self.ddf(self.T * (3-np.sqrt(3))/6)