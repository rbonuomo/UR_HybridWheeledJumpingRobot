clear all
close all
clc

mb = 4;
mw = 2;
Rw = 0.17;
Iw = (mw*Rw^2);

dyn = dynamics_model(mb,mw,Iw,Rw);
utils_obj=utils(Rw);

%% Swing-up and Balance

nx = 10;
ny = 1;
nu = 4;

nlobj = nlmpc(nx,ny,nu);

Duration=3;
p = 10;
Ts = Duration/20;
nlobj.Ts = Ts;
nlobj.PredictionHorizon = p;
nlobj.ControlHorizon = 8;

nlobj.ManipulatedVariables(1).Min = -10;
nlobj.ManipulatedVariables(2).Min = -200;
nlobj.ManipulatedVariables(1).Max = 10;
nlobj.ManipulatedVariables(2).Max = 200;
nlobj.ManipulatedVariables(4).Min=0;


nlobj.States(1).Max=1;
nlobj.States(2).Max=Rw;
nlobj.States(2).Min=Rw;
nlobj.States(4).Max=1;
nlobj.States(4).Min=0.2;
nlobj.States(5).Max=pi/2;
nlobj.States(5).Min=-pi/2;

syms xi z phi l theta dxi dz dphi dl dtheta tau f lambda_x lambda_z real

x = [xi;z;phi;l;theta;dxi;dz;dphi;dl;dtheta];
u = [tau;f;lambda_x;lambda_z];

nlobj.Optimization.CustomCostFcn = @(X,U,e,data) utils.cost_func(X,U,e,data);

Optimization.CustomEqConFcn = @(X,U,data) utils_obj.eqConfunction(X,U,data);
Optimization.CustomIneqConFcn = @(X,U,e,data) utils_obj.ineqConfunction(X,U,e,data);

nlobj.Model.StateFcn = @(x,u) dyn.next_state(x,u);

nlobj.Model.OutputFcn = @(x,u) [x(5)];

v0=3;
x0 = [-1;Rw;0;0.3;-pi/2;v0;0;v0/Rw;0;0];
u0 = [0;0;0;0];
validateFcns(nlobj, x0, u0, []);

x_d=[0];

nloptions = nlmpcmoveopt;

xk=x0;
uk=u0;
xHistory = x0';
uHistory = u0'; 
for ct = 1:(Duration/Ts)

    disp(['iteration ..',num2str(ct)])
     [~,~,info]= nlmpcmove(nlobj,xk,uk,x_d,[],nloptions);
     xk=info.Xopt(2,:);
     uk=info.MVopt(2,:);
     xHistory = [xHistory; xk];
     uHistory = [uHistory; uk];
end

delta_t=Ts;
time_int=[0:Ts:Duration];

utils.plot_state(xHistory,time_int)

utils.plot_control(uHistory,time_int)

utils.createVideo(xHistory(:,1), xHistory(:,2), xHistory(:,4), xHistory(:,5), 1/Ts)