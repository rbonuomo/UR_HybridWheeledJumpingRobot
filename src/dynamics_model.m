classdef dynamics_model
    %DYNAMIC_MODEL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        mb
        mw
        mt
        Iw
        g
        Rw
    end
    
    methods
        
        function obj = dynamics_model(mb,mw,Iw,Rw)
            obj.mb = mb;
            obj.mw = mw;
            obj.mt = mb + mw;
            obj.Iw = Iw;
            obj.g = 9.81; % [m/s^2]
            obj.Rw = Rw;
        end
        
        function M = mass_matrix(obj,q)
            
            M = [obj.mt                              0                0       obj.mb*sin(q(5))  obj.mb*q(4)*cos(q(5));
                    0                            obj.mt               0       obj.mb*cos(q(5)) -obj.mb*q(4)*sin(q(5));
                    0                                0              obj.Iw            0                    0;
                 obj.mb*sin(q(5))            obj.mb*cos(q(5))         0           obj.mb                   0;
                 obj.mb*q(4)*cos(q(5)) -obj.mb*q(4)*sin(q(5))       0               0                 obj.mb*q(4)^2];
        end
        
        function C = coriolis_matrix(obj,q,dq)
            
            C = [0 0 0  2*obj.mb*cos(q(5))*dq(5) -obj.mb*q(4)*sin(q(5))*dq(5);
                 0 0 0 -2*obj.mb*sin(q(5))*dq(5) -obj.mb*q(4)*cos(q(5))*dq(5);
                 0 0 0              0                           0;
                 0 0 0              0                 -obj.mb*q(4)*dq(5);
                 0 0 0     2*obj.mb*q(4)*dq(5)                  0];
            
        end
        
        function G = gravity_vector(obj,q)
            
            G = [0 obj.g*obj.mt 0 obj.g*obj.mb*cos(q(5)) -obj.g*obj.mb*q(4)*sin(q(5))]';
            
        end
        
        function S = get_S(obj)
            
            S = [0 0 1 0 -1;
                 0 0 0 1 0];
            
        end
        
        function Jc = get_jacobian(obj)
            
            Jc = [1 0 -obj.Rw 0 0;
                  0 1     0   0 0];
            
        end
        
        function ddq = get_ddq(obj,q,dq,u,lambda)
            
            M = obj.mass_matrix(q);
            C = obj.coriolis_matrix(q,dq);
            G = obj.gravity_vector(q);
            S = obj.get_S();
            Jc = obj.get_jacobian();
            
            ddq = M^-1*(S'*u+Jc'*lambda-C*dq-G);
            
        end
        
        function dx = next_state(obj,x,u,lambda)
            
            ddq = obj.get_ddq(x(1:5),x(6:10),u,lambda);
            dx = [x(6:10);ddq];
            
        end
        
    end
end

