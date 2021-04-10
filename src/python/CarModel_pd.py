import numpy as np
from functools import partial

class CarModel_pd:

    def __init__(self, mb, mw, Iw, Rw, dT, X):
        self.mb = mb
        self.mw = mw
        self.Iw = Iw
        self.Rw = Rw
        self.X = X
        self.g = 9.81

    def set_state(self, X):
        self.X = X

    def set_input(self, U):
        self.U = U

    def get_M(self, X):
        x = X[0]
        z = X[1]
        phi = X[2]
        l = X[3]
        theta = X[4]

        mb = self.mb
        mw = self.mw
        Iw = self.Iw
        Rw = self.Rw
        mt = mb + mw
        
        M = np.zeros((5,5))
    
        M[0, 0] = mt
        M[0, 1] = 0
        M[0, 2] = 0
        M[0, 3] = mb*np.sin(theta)
        M[0, 4] = mb*l*np.cos(theta)
        M[1, 0] = 0
        M[1, 1] = mt
        M[1, 2] = 0
        M[1, 3] = mb*np.cos(theta)
        M[1, 4] = -mb*l*np.sin(theta)
        M[2, 0] = 0
        M[2, 1] = 0
        M[2, 2] = Iw
        M[2, 3] = 0
        M[2, 4] = 0
        M[3, 0] = mb*np.sin(theta)
        M[3, 1] = mb*np.cos(theta)
        M[3, 2] = 0
        M[3, 3] = mb
        M[3, 4] = 0
        M[4, 0] = mb*l*np.cos(theta)
        M[4, 1] = -mb*l*np.sin(theta)
        M[4, 2] = 0
        M[4, 3] = 0
        M[4, 4] = mb*(l**2)
        
        return M

    def get_C(self, X):
        x = X[0]
        z = X[1]
        phi = X[2]
        l = X[3]
        theta = X[4]
        x_dot = X[5]
        z_dot = X[6]
        phi_dot = X[7]
        l_dot = X[8]
        theta_dot = X[9]

        mb = self.mb
        mw = self.mw
        Iw = self.Iw
        Rw = self.Rw
        mt = mb + mw

        C = np.zeros((5,5))

        C[0, 0] = 0
        C[0, 1] = 0
        C[0, 2] = 0
        C[0, 3] = 2*mb*np.cos(theta)*theta_dot
        C[0, 4] = -mb*l*np.sin(theta)*theta_dot
        C[1, 0] = 0
        C[1, 1] = 0
        C[1, 2] = 0
        C[1, 3] = -2*mb*np.sin(theta)*theta_dot
        C[1, 4] = -mb*l*np.cos(theta)*theta_dot
        C[2, 0] = 0
        C[2, 1] = 0
        C[2, 2] = 0
        C[2, 3] = 0
        C[2, 4] = 0
        C[3, 0] = 0
        C[3, 1] = 0
        C[3, 2] = 0
        C[3, 3] = 0
        C[3, 4] = -mb*l*theta_dot
        C[4, 0] = 0
        C[4, 1] = 0
        C[4, 2] = 0
        C[4, 3] = 2*mb*l*theta_dot
        C[4, 4] = 0

        return C

    def get_G(self, X):
        x = X[0]
        z = X[1]
        phi = X[2]
        l = X[3]
        theta = X[4]

        mb = self.mb
        mw = self.mw
        Iw = self.Iw
        Rw = self.Rw
        g = self.g
        mt = mb + mw

        G = np.zeros((5))

        G[0] = 0
        G[1] = g*mt
        G[2] = 0
        G[3] = g*mb*np.cos(theta)
        G[4] = -g*mb*l*np.sin(theta)

        return G
    
    def get_S(self):

        S = np.zeros((2,5))
        S[0, 0] = 0
        S[0, 1] = 0
        S[0, 2] = 1
        S[0, 3] = 0
        S[0, 4] = -1
        S[1, 0] = 0
        S[1, 1] = 0
        S[1, 2] = 0
        S[1, 3] = 1
        S[1, 4] = 0

        return S

    def get_Jc(self):

        Rw = self.Rw

        Jc = np.zeros((2,5))
        Jc[0, 0] = 1
        Jc[0, 1] = 0
        Jc[0, 2] = -Rw
        Jc[0, 3] = 0
        Jc[0, 4] = 0
        Jc[1, 0] = 0
        Jc[1, 1] = 1
        Jc[1, 2] = 0
        Jc[1, 3] = 0
        Jc[1, 4] = 0

        return Jc

    def return_continuous_model(self, U, lam, t, X0):

        M = self.get_M(X0)
        C = self.get_C(X0)
        S = self.get_S()
        G = self.get_G(X0)
        Jc = self.get_Jc()
        
        dq = X0[5:]

        ddq = np.linalg.inv(M)@(S.T@U + Jc.T@lam - C@dq - G)

        dX = np.concatenate((dq,ddq))

        return dX

    def return_lambda(self, U, X, dX):

        M = self.get_M(X)
        C = self.get_C(X)
        S = self.get_S()
        G = self.get_G(X)
        Jc = self.get_Jc()

        dq = X[5:]
        ddq = dX[0:6]

        lam = np.linalg.pinv(Jc.T)@(M@ddq + C@dq + G - S.T@U)

        return lam

    def next_state(self, U, lam, dT):

        t, next_X = rks4(partial(self.return_continuous_model,U,lam), 0, dT, self.X, 1)

        return next_X[1]

def rks4(f, a, b, Za, M):
    
    # M + 1 steps in total
    h = (b - a)/ M
    t = np.linspace(a, b, M + 1).reshape(M+1,1)
    Z = np.zeros((M+1, Za.size))
    Z[0,:] = Za
    # print(Z.shape,Z[:,0].shape, Z[0,:].shape)
    for i in range(1,M+1):
        k1 = h * f(t[i-1], np.transpose(Z[i-1,:]))
        k2 = h * f(t[i-1] +h/2, np.transpose(Z[i-1,:] +k1/2))
        k3 = h * f(t[i-1] +h/2, np.transpose(Z[i-1,:] +k2/2))
        k4 = h * f(t[i-1] +h, np.transpose(Z[i-1,:] +k3))
        # print(k4.shape)
        Z[i,:] = Z[i-1,:] + (k1 + 2*k2 + 2*k3 + k4)/6
#     print(t.shape, Z.shape, sep='\n')
    return (t, Z)


