clear all
close all
clc

mb = 2;
mw = 4;
Rw = 0.17;
Iw = (mw*Rw^2);

dyn = dynamics_model(mb,mw,Iw,Rw);
utils_obj=utils(Rw);

%% Swing-up and Balance

nx = 10;
ny = 10;
nu = 4;

nlobj = nlmpc(nx,ny,nu);

Duration=4;
%p = 5;
Ts = 0.08;
nlobj.Ts = Ts;
nlobj.PredictionHorizon = 10;
nlobj.ControlHorizon = 5;

nlobj.ManipulatedVariables(1).Min = -10;
nlobj.ManipulatedVariables(2).Min = -200;
nlobj.ManipulatedVariables(1).Max = 10;
nlobj.ManipulatedVariables(2).Max = 200;
nlobj.ManipulatedVariables(4).Min=0;

nlobj.States(1).Max=10;
nlobj.States(1).Min=-15;
nlobj.States(2).Max=Rw;
nlobj.States(2).Min=Rw;
nlobj.States(4).Max=1+2*Rw;
nlobj.States(4).Min=2*Rw;
nlobj.States(5).Max=pi/2;
nlobj.States(5).Min=-pi/2;

syms xi z phi l theta dxi dz dphi dl dtheta tau f lambda_x lambda_z real

x = [xi;z;phi;l;theta;dxi;dz;dphi;dl;dtheta];
u = [tau;f;lambda_x;lambda_z];

nlobj.Optimization.CustomCostFcn = @(X,U,e,data) utils.cost_func(X,U,e,data);

nlobj.Optimization.CustomEqConFcn = @(X,U,data) utils_obj.eqConfunction(X,U,data);
nlobj.Optimization.CustomIneqConFcn = @(X,U,e,data) utils_obj.ineqConfunction(X,U,e,data);

nlobj.Model.StateFcn = @(x,u) dyn.next_state(x,u);
nlobj.Model.IsContinuousTime = true;

%nlobj.Model.OutputFcn = @(x,u) [x(5)];

v0=0;
x0 = [-5;Rw;0;0.5;-pi/2; v0;0;v0/Rw;0;0];
u0 = [0;0;0;0];

validateFcns(nlobj, x0, u0, []);

W1 = [0 0 0 0 0 1 0 0 0 0];
W2 = [0 0 0 0 1 1 0 0 0 0];

x_d1=[-1 Rw 0 0.5 -pi/2 3 0 0 0 0];
x_d2=[-1 Rw 0 0.5 0 0 0 0 0 0];

nloptions = nlmpcmoveopt;

time = 0;
tf1 = 2;
tf2 = 4;
current_phase = 1;

xk=x0;
uk=u0;
xHistory = x0';
uHistory = u0';

x_d = x_d1;
nlobj.Weights.OutputVariables = W1;

for ct = 1:(Duration/Ts)

    phase = utils.temporal_phase(time,tf1,tf2);
    if current_phase ~= phase
        current_phase = phase;
        if phase == 2
        nlobj.Weights.OutputVariables = W2;
        x_d = x_d2;
        disp('Phase 2')
        elseif phase == 3
            disp('Phase 3')
        end
    end
    disp(['iteration.. ',num2str(ct),'/', num2str(Duration/Ts)])
     [uk,nloptions, info]= nlmpcmove(nlobj,xk,uk,x_d,[],nloptions);
     xk=info.Xopt(2,:);
     uk=info.MVopt(2,:);
     xHistory = [xHistory; xk];
     uHistory = [uHistory; uk];
     time = time+Ts;
end

time_int=[0:Ts:Duration];

utils.plot_state(xHistory,time_int)

utils.plot_control(uHistory,time_int)

utils.createVideo(xHistory(:,1), xHistory(:,2), xHistory(:,4), xHistory(:,5), round(1/Ts))