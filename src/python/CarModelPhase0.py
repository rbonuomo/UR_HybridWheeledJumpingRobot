from acados_template import AcadosModel
from casadi import *
#from casadi import SX, vertcat, sin, cos, Function

class CarModelPhase0:

    def __init__(self, mb, mw, Iw, Rw, l, dT):
        self.mb = mb
        self.mw = mw
        self.Iw = Iw
        self.Rw = Rw
        self.l = l
        self.model = export_car_ode_model_with_discrete_rk4(mb, mw, Iw, Rw, l, dT)


def export_car_ode_model(mb, mw, Iw, Rw, l):

    model_name = 'car_phase0_ode'

    #system parameters
    mt = mb + mw
    g = 9.81 # [m/s^2]

    # set up states & controls
    phi = MX.sym('phi')
    theta = MX.sym('theta')
    phi_dot = MX.sym('phi_dot')
    theta_dot = MX.sym('theta_dot')

    x = vertcat(phi, theta, 
                phi_dot, theta_dot)

    # controls
    tau = MX.sym('tau')
    u = vertcat(tau)
    
    # xdot
    phi_ddot = MX.sym('phi_ddot')
    theta_ddot = MX.sym('theta_ddot')

    xdot = vertcat(phi_dot, theta_dot,
                phi_ddot, theta_ddot)

    phidd = tau/(Iw+mt*Rw*Rw)

    # dynamics
    f_expl = vertcat(phi_dot,
                    theta_dot,
                    phidd,
                    (1/mb*l*l)*(-mb*Rw*phidd*l*sin(theta) + mb*g*l*cos(theta) - tau)
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

    #model.con_h_expr = vertcat( (lambda_z - fabs(lambda_x)),
    #                            (x1_dot-Rw*phi_dot))
    print(f"SHAPE: {model.con_h_expr}")

    return model


def export_car_ode_model_with_discrete_rk4(mb, mw, Iw, Rw, l, dT):

    model = export_car_ode_model(mb, mw, Iw, Rw, l)

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

