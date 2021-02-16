classdef utils
    %UTILS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        %null
    end
    
    methods (Static)
        
        function plot_state(info)
            figure('Name','Plot state')
            subplot(5,2,1)
            plot(info.Topt,info.Xopt(:,1))
            xlabel('time')
            ylabel('x')
             grid on
%             title('cart position')
            subplot(5,2,3)
            plot(info.Topt,info.Xopt(:,2))
            xlabel('time')
            ylabel('z')
             grid on
%             title('cart velocity')
            subplot(5,2,5)
            plot(info.Topt,info.Xopt(:,3))
            xlabel('time')
            ylabel('phi')
             grid on
           % title('pendulum angle')
            subplot(5,2,7)
            plot(info.Topt,info.Xopt(:,4))
            xlabel('time')
            ylabel('l')
             grid on
%             title('pendulum velocity')
            subplot(5,2,9)
            plot(info.Topt,info.Xopt(:,5))
            xlabel('time')
            ylabel('theta')
             grid on
            
            subplot(5,2,2)
            plot(info.Topt,info.Xopt(:,6))
            xlabel('time')
            ylabel('x dot')
             grid on
%             title('cart position')
            subplot(5,2,4)
            plot(info.Topt,info.Xopt(:,7))
            xlabel('time')
            ylabel('z dot')
             grid on
%             title('cart velocity')
            subplot(5,2,6)
            plot(info.Topt,info.Xopt(:,8))
            xlabel('time')
            ylabel('phi dot')
             grid on
           % title('pendulum angle')
            subplot(5,2,8)
            plot(info.Topt,info.Xopt(:,9))
            xlabel('time')
            ylabel('l dot')
             grid on
%             title('pendulum velocity')
            subplot(5,2,10)
            plot(info.Topt,info.Xopt(:,10))
            xlabel('time')
            ylabel('theta dot')
            grid on
        end
        
        function plot_control(info)
            figure('Name','Plot Controls')
            subplot(1,2,1)
            plot(info.Topt,info.MVopt(:,1))
            xlabel('time')
            ylabel('tau')
             grid on
%             title('pendulum velocity')
            subplot(1,2,2)
            plot(info.Topt,info.MVopt(:,2))
            xlabel('time')
            ylabel('f')
            grid on
            
        end
        
        function func=cost_func(X,U,e,data)
            W=1;
            func=0;
            for i=1:size(U,1)
                f=W*U(i,:)*U(i,:)';
                func=func+f;
            end
        end

    end
end

