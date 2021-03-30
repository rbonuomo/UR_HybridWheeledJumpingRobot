from acados_template import AcadosModel
from casadi import *
#from casadi import SX, vertcat, sin, cos, Function

class CarModel:

    def __init__(self, mb, mw, Iw, Rw, dT):
        self.mb = mb
        self.mw = mw
        self.Iw = Iw
        self.Rw = Rw
        self.model = export_car_ode_model(mb, mw, Iw, Rw)


def export_car_ode_model(mb, mw, Iw, Rw):

    model_name = 'car_ode'

    #system parameters
    mt = mb + mw
    g = 9.81 # [m/s^2]

    # set up states & controls
    x1 = MX.sym('x1')
    z = MX.sym('z')
    phi = MX.sym('phi')
    l = MX.sym('l')
    theta = MX.sym('theta')
    x1_dot = MX.sym('x1_dot')
    z_dot = MX.sym('z_dot')
    phi_dot = MX.sym('phi_dot')
    l_dot = MX.sym('l_dot')
    theta_dot = MX.sym('theta_dot')

    x = vertcat(x1, z, phi, l, theta, 
                x1_dot, z_dot, phi_dot, l_dot, theta_dot)

    # controls
    tau = MX.sym('tau')
    f = MX.sym('f')
    lambda_x = MX.sym('lambda_x')
    lambda_z = MX.sym('lambda_z')
    u = vertcat(tau, f, lambda_x, lambda_z)
    
    # xdot
    x1_ddot = MX.sym('x1_ddot')
    z_ddot = MX.sym('z_ddot')
    phi_ddot = MX.sym('phi_ddot')
    l_ddot = MX.sym('l_ddot')
    theta_ddot = MX.sym('theta_ddot')

    xdot = vertcat(x1_dot, z_dot, phi_dot, l_dot, theta_dot,
                x1_ddot, z_ddot, phi_ddot, l_ddot, theta_ddot)
    
    # inertia matrix
    M = MX(5, 5)
    M[0, 0] = mt
    M[0, 1] = 0
    M[0, 2] = 0
    M[0, 3] = mb*sin(theta)
    M[0, 4] = mb*l*cos(theta)
    M[1, 0] = 0
    M[1, 1] = mt
    M[1, 2] = 0
    M[1, 3] = mb*cos(theta)
    M[1, 4] = -mb*l*sin(theta)
    M[2, 0] = 0
    M[2, 1] = 0
    M[2, 2] = Iw
    M[2, 3] = 0
    M[2, 4] = 0
    M[3, 0] = mb*sin(theta)
    M[3, 1] = mb*cos(theta)
    M[3, 2] = 0
    M[3, 3] = mb
    M[3, 4] = 0
    M[4, 0] = mb*l*cos(theta)
    M[4, 1] = -mb*l*sin(theta)
    M[4, 2] = 0
    M[4, 3] = 0
    M[4, 4] = mb*(l**2)

    # coriolis and centrifugal terms
    C = MX(5,5)
    C[0, 0] = 0
    C[0, 1] = 0
    C[0, 2] = 0
    C[0, 3] = 2*mb*cos(theta)*theta_dot
    C[0, 4] = -mb*l*sin(theta)*theta_dot
    C[1, 0] = 0
    C[1, 1] = 0
    C[1, 2] = 0
    C[1, 3] = -2*mb*sin(theta)*theta_dot
    C[1, 4] = -mb*l*cos(theta)*theta_dot
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

    # gravity term
    G = MX(5,1)
    G[0, 0] = 0
    G[1, 0] = g*mt
    G[2, 0] = 0
    G[3, 0] = g*mb*cos(theta)
    G[4, 0] = -g*mb*l*sin(theta)

    # define S
    S = MX(2,5)
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

    # define Jc
    Jc = MX(2,5)
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

    # compute acceleration
    ddq = mtimes(inv(M), (mtimes(transpose(S),vertcat(tau, f)) + mtimes(transpose(Jc),vertcat(lambda_x, lambda_z))-
                    mtimes(C,vertcat(x1_dot, z_dot, phi_dot, l_dot, theta_dot)) - G))
    print(ddq.shape)

    # dynamics
    f_expl = vertcat(x1_dot,
                    z_dot,
                    phi_dot,
                    l_dot,
                    theta_dot,
                    ddq
                    )
    print(f_expl.shape)
    f_impl = xdot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    # algebraic variables
    model.z = vertcat([])

    # parameters
    p = vertcat([])
    model.p = p
    model.name = model_name

    # activate1 = MX.sym('activate1')
    # activate2 = MX.sym('activate2')
    # activate3 = MX.sym('activate3')
    # activate4 = MX.sym('activate4')
    # activate5 = MX.sym('activate5')
    # activate6 = MX.sym('activate6')
    # activate7 = MX.sym('activate7')
    # model.p = vertcat(activate1, activate2, activate3, activate4, activate5, activate6, activate7)

    model.con_h_expr = vertcat( (lambda_z - fabs(lambda_x)),
                                (x1_dot-Rw*phi_dot))
    print(f"SHAPE: {model.con_h_expr}")

    return model


def export_car_ode_model_with_discrete_rk4(mb, mw, Iw, Rw, dT):

    model = export_car_ode_model(mb, mw, Iw, Rw)

    x = model.x
    u = model.u
    nx = x.size()[0]

    ode = Function('ode', [x, u], [model.f_expl_expr])
    # set up RK4
    k1 = ode(x,       u)
    k2 = ode(x+dT/2*k1,u)
    k3 = ode(x+dT/2*k2,u)
    k4 = ode(x+dT*k3,  u)
    xf = x + dT/6 * (k1 + 2*k2 + 2*k3 + k4)

    model.disc_dyn_expr = xf
    print("built RK4 for car model with dT = ", dT)
    # print(xf)
    # print()
    # print(k1)
    # print(k2)
    # print(k3)
    # print(k4)
    
    # print(xf.shape)
    
    #print( xf.set() )
    
    return model

