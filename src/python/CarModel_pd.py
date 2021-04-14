from acados_template import AcadosModel
from casadi import *
#from casadi import SX, vertcat, sin, cos, Function

class CarModel_pd:

    def __init__(self, mb, mw, Iw, Rw, dT):
        self.mb = mb
        self.mw = mw
        self.Iw = Iw
        self.Rw = Rw
        self.model = export_car_ode_model_with_discrete_rk4(mb, mw, Iw, Rw, dT)


def export_car_ode_model(mb, mw, Iw, Rw):

    model_name = 'car_ode'

    #system parameters
    mt = mb + mw
    g = 9.81 # [m/s^2]

    # set up states & controls
    phi = MX.sym('phi')
    l = MX.sym('l')
    theta = MX.sym('theta')
    phi_dot = MX.sym('phi_dot')
    l_dot = MX.sym('l_dot')
    theta_dot = MX.sym('theta_dot')

    x = vertcat(phi, l, theta, 
                phi_dot, l_dot, theta_dot)

    # controls
    u = vertcat([])
    
    # xdot
    phi_ddot = MX.sym('phi_ddot')
    l_ddot = MX.sym('l_ddot')
    theta_ddot = MX.sym('theta_ddot')

    xdot = vertcat(phi_dot, l_dot, theta_dot,
                phi_ddot, l_ddot, theta_ddot)
    
    # inertia matrix
    M = MX(3, 3)
    M[0,0]=Iw + (Rw**2)*mb + (Rw**2)*mw
    M[0,1]=Rw*mb*sin(theta)
    M[0,2]=Rw*l*mb*cos(theta)
    M[1,0]=Rw*mb*sin(theta)
    M[1,1]=mb
    M[1,2]=0
    M[2,0]=Rw*l*mb*cos(theta)
    M[2,1]=0
    M[2,2]=(l**2)*mb

    # coriolis and centrifugal terms
    C = MX(3,1)
    C[0, 0]=Rw*mb*theta_dot*(2*l_dot*cos(theta) - l*theta_dot*sin(theta))
    C[1, 0]=-l*mb*(theta_dot**2)
    C[2, 0]=2*l*l_dot*mb*theta_dot

    # gravity term
    G = MX(3,1)
    G[0, 0] = 0
    G[1, 0] = g*mb*cos(theta)
    G[2, 0] = -g*l*mb*sin(theta)

    # define S
    S = MX(2,3)
    S[0, 0] = 1
    S[0, 1] = 0
    S[0, 2] = -1
    S[1, 0] = 0
    S[1, 1] = 1
    S[1, 2] = 0

    tau = MX.sym('tau')
    f = MX.sym('f')
    p = vertcat(tau, f)

    # compute acceleration
    ddq = mtimes(inv(M), (mtimes(transpose(S),vertcat(tau, f)) - C - G))
    print(ddq.shape)

    # dynamics
    f_expl = vertcat(phi_dot,
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
    model.p = p
    model.name = model_name

    
    #model.con_h_expr = vertcat( (lambda_z - fabs(lambda_x)),
    #                            (x1_dot-Rw*phi_dot))
    #print(f"SHAPE: {model.con_h_expr}")

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

    
    return model

