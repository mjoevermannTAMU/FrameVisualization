import numpy as np
from math import sin, cos

def link_transform(theta, d, a, alpha):
    # return the transform matrix for a row in the D-H Table
    return np.array([[cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
                     [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
                     [0, sin(alpha), cos(alpha), d],
                     [0,0,0,1]])

def Rx(t):
    return np.array([[1, 0, 0],
                    [0, cos(t), -sin(t)],
                    [0, sin(t), cos(t)]])
def Ry(t):
    return np.array([[cos(t), 0,  sin(t)],
                    [0, 1, 0],
                    [-sin(t), 0, cos(t)]])
def Rz(t):
    return np.array([[cos(t), -sin(t), 0],
                    [sin(t), cos(t), 0],
                    [0, 0, 1]])

def quat2rot(real, vec): # quaternion to rotation matrix
    ''' quaternion resources
    https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
    https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    https://www.bing.com/videos/search?q=quaternions+explained&view=detail&mid=9D13801F1E7D9CD4FAD99D13801F1E7D9CD4FAD9&FORM=VIRE
    '''
    qr = real
    qi = vec[0]
    qj = vec[1]
    qk = vec[2]
    s = 1/(np.sqrt(qr**2+ qi**2 + qj**2 + qk**2 ))**2
    r11 = 1 - 2*s*(qj**2 + qk**2)
    r12 = 2*s*(qi*qj - qk*qr)
    r13 = 2*s*(qi*qk + qj*qr)
    r21 = 2*s*(qi*qj + qk*qr)
    r22 = 1 - 2 * s*(qi ** 2 + qk ** 2)
    r23 = 2*s*(qj*qk - qi*qr)
    r31 = 2*s*(qi*qk - qj*qr)
    r32 = 2*s*(qj*qk + qi*qr)
    r33 = 1 - 2 * s*(qi ** 2 + qj ** 2)

    return np.array([[r11, r12, r13],
                     [r21, r22, r23],
                     [r31, r32, r33]])

class Ball:
    # class to hold the dynamic EOM of the ball
    def __init__(self, R_ball, r_pend, m_ball, m_pend, I_ball):
        self.R_ball = R_ball
        self.r_pend = r_pend
        self.m_ball = m_ball
        self.m_pend = m_pend

        self.I_ball = I_ball
        self.I_pend = self.m_pend*r_pend**2
        self.g = 32.17*12    # [lb in /s] acceleration due to gravity

    def M_matrix(self, th_2, phi_r):
        # the mass matrix with the pendulum steering angle and balls response angle phi
        M11 = self.m_ball*self.R_ball**2 + self.m_pend*self.R_ball**2 + self.m_ball*self.r_pend**2 + self.I_ball + self.I_pend + 2*self.m_pend*self.R_ball*self.r_pend*cos(th_2 - phi_r)
        M12 = -self.m_pend*self.r_pend**2 - self.I_pend - self.m_pend*self.R_ball*self.r_pend*cos(th_2 - phi_r)
        M21 = M12
        M22 = self.m_pend*self.r_pend**2 + self.I_pend
        return np.array([[M11, M12], [M12, M22]])

    def V_matrix(self, th_2, th_2_dot, phi_r, phi_r_dot):
        G1 = self.m_pend*self.R_ball*self.r_pend*sin(th_2 - phi_r)*(phi_r_dot**2 - th_2_dot**2) - self.m_pend*self.g*self.r_pend*sin(th_2 - phi_r)
        G2 = self.m_pend*self.g*self.r_pend*sin(th_2 - phi_r)
        return np.array([[G1], [G2]])

def find_th1():
    return