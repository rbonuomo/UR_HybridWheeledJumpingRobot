clear all
close all
clc

mb = 4;
mw = 2;
Rw = 0.17;
Iw = (mw*Rw^2);

dyn = dynamics_model(mb,mw,Iw,Rw);

%% Swing-up and Balance

nx = 10;
ny = 1;
nu = 2;

nlobj = nlmpc(nx,ny,nu);

p = 20;
Ts = 5;
nlobj.Ts = Ts;
nlobj.PredictionHorizon = p;
nlobj.ControlHorizon = 5;

nlobj.ManipulatedVariables(1).Min = -10;
nlobj.ManipulatedVariables(2).Min = -200;
nlobj.ManipulatedVariables(1).Max = 10;
nlobj.ManipulatedVariables(2).Max = 200;

nlobj.States(1).Max=1;
nlobj.States(2).Max=Rw;
nlobj.States(2).Min=Rw;
nlobj.States(4).Max=1;
nlobj.States(4).Min=0.2;
nlobj.States(5).Max=pi/2;
nlobj.States(5).Min=-pi/2;

syms xi z phi l theta dxi dz dphi dl dtheta tau f lambda_x lambda_z real

x = [xi;z;phi;l;theta;dxi;dz;dphi;dl;dtheta];
u = [tau;f];
% lambda = [lambda_x;lambda_z];

nlobj.Optimization.CustomCostFcn = @(X,U,e,data) utils.cost_func(X,U,e,data);


% nlobj.Model.StateFcn = @(x,u,lambda) dyn.next_state(x,u,lambda);
nlobj.Model.StateFcn = @(x,u) dyn.next_state(x,u);

nlobj.Model.OutputFcn = @(x,u) [x(5)];

%nlobj.Model.NumberOfParameters = 1;

x0 = [-1;0.17;0;0.3;0;3;0;0;0;0];
u0 = [0;0];
%validateFcns(nlobj, x0, u0, [], {lambda});
validateFcns(nlobj, x0, u0, []);

x_d=[0];

nloptions = nlmpcmoveopt;
%nloptions.Parameters = {lambda};

[~,~,info] = nlmpcmove(nlobj,x0,u0,x_d,[],nloptions);

utils.plot_state(info)

utils.plot_control(info)

