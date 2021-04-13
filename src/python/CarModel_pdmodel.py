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
        phi=X[0]
        l=X[1]
        theta=X[2]

        mb = self.mb
        mw = self.mw
        Iw = self.Iw
        Rw = self.Rw
        mt = mb + mw
        
        M = np.zeros((3,3))
        M[0,0]=Iw + (Rw**2)*mb + (Rw**2)*mw
        M[0,1]=Rw*mb*np.sin(theta)
        M[0,2]=Rw*l*mb*np.cos(theta)
        M[1,0]=Rw*mb*np.sin(theta)
        M[1,1]=mb
        M[1,2]=0
        M[2,0]=Rw*l*mb*np.cos(theta)
        M[2,1]=0
        M[2,2]=(l**2)*mb

        return M

    def get_C(self, X):

        phi=X[0]
        l=X[1]
        theta=X[2]
        phi_dot=X[3]
        l_dot=X[4]
        theta_dot=X[5]

        mb = self.mb
        mw = self.mw
        Iw = self.Iw
        Rw = self.Rw
        mt = mb + mw

        C = np.zeros((3))
        C[0]=Rw*mb*theta_dot*(2*l_dot*np.cos(theta) - l*theta_dot*np.sin(theta))
        C[1]=-l*mb*(theta_dot**2)
        C[2]=2*l*l_dot*mb*theta_dot


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

        G = np.zeros((3))

        G[0] = 0
        G[1] = g*mb*np.cos(theta)
        G[2] = -g*l*mb*np.sin(theta)

        return G
    
    def get_S(self):

        S = np.zeros((2,3))
        S[0, 0] = 1
        S[0, 1] = 0
        S[0, 2] = -1
        S[1, 0] = 0
        S[1, 1] = 1
        S[1, 2] = 0

        return S

    def return_continuous_model(self, U, t, X0):

        M = self.get_M(X0)
        C = self.get_C(X0)
        S = self.get_S()
        G = self.get_G(X0)
        
        dq = X0[3:]

        ddq = np.linalg.inv(M)@(S.T@U - C - G)

        dX = np.concatenate((dq,ddq))

        return dX


    def next_state(self, U, dT):

        #t, next_X = rks4(partial(self.return_continuous_model,U), 0, dT, self.X, 1)
        #self.X=next_X[1]

        dX=self.return_continuous_model(U,dT,self.X)
        X=self.X+dX*dT
        self.X=X

        return X

        #dX=self.return_continuous_model(U,dT,self.X)

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


