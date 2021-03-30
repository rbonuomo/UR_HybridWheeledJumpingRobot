classdef utils
    %UTILS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Rw
    end
    
    methods (Static)
        
        function plot_state(XHistory,time)
            figure('Name','Plot state')
            subplot(2,5,1)
            plot(time,XHistory(:,1))
            xlabel('$t$','interpreter','latex', 'FontSize', 20)
            ylabel('$x$','interpreter','latex', 'FontSize', 20)
             grid on
            subplot(2,5,2)
            plot(time,XHistory(:,2))
            xlabel('$t$','interpreter','latex', 'FontSize', 20)
            ylabel('$z$','interpreter','latex', 'FontSize', 20)
             grid on
            subplot(2,5,3)
            plot(time,XHistory(:,3))
            xlabel('$t$','interpreter','latex', 'FontSize', 20)
            ylabel('$\phi$','interpreter','latex', 'FontSize', 20)
             grid on
            subplot(2,5,4)
            plot(time,XHistory(:,4))
            xlabel('$t$','interpreter','latex', 'FontSize', 20)
            ylabel('$l$','interpreter','latex', 'FontSize', 20)
             grid on
            subplot(2,5,5)
            plot(time,XHistory(:,5))
            xlabel('$t$','interpreter','latex', 'FontSize', 20)
            ylabel('$\theta$','interpreter','latex', 'FontSize', 20)
             grid on
            
            subplot(2,5,6)
            plot(time,XHistory(:,6))
            xlabel('$t$','interpreter','latex', 'FontSize', 20)
            ylabel('$\dot{x}$','interpreter','latex', 'FontSize', 20)
             grid on
            subplot(2,5,7)
            plot(time,XHistory(:,7))
            xlabel('$t$','interpreter','latex', 'FontSize', 20)
            ylabel('$\dot{z}$','interpreter','latex', 'FontSize', 20)
             grid on
            subplot(2,5,8)
            plot(time,XHistory(:,8))
            xlabel('$t$','interpreter','latex', 'FontSize', 20)
            ylabel('$\dot{\phi}$','interpreter','latex', 'FontSize', 20)
             grid on
            subplot(2,5,9)
            plot(time,XHistory(:,9))
            xlabel('$t$','interpreter','latex', 'FontSize', 20)
            ylabel('$\dot{l}$','interpreter','latex', 'FontSize', 20)
             grid on
            subplot(2,5,10)
            plot(time,XHistory(:,10))
            xlabel('$t$','interpreter','latex', 'FontSize', 20)
            ylabel('$\dot{\theta}$','interpreter','latex', 'FontSize', 20)
            grid on
        end
        
        function plot_control(uHistory,time)
            figure('Name','Plot Controls')
            subplot(2,2,1)
            plot(time,uHistory(:,1))
            xlabel('$t$','interpreter','latex', 'FontSize', 20)
            ylabel('$\tau$','interpreter','latex', 'FontSize', 20)
             grid on
             
            subplot(2,2,2)
            plot(time,uHistory(:,2))
            xlabel('$t$','interpreter','latex', 'FontSize', 20)
            ylabel('$f$','interpreter','latex', 'FontSize', 20)
            grid on
            
            subplot(2,2,3)
            plot(time,uHistory(:,3))
            xlabel('$t$','interpreter','latex', 'FontSize', 20)
            ylabel('$\lambda_x$','interpreter','latex', 'FontSize', 20)
            grid on
            
            subplot(2,2,4)
            plot(time,uHistory(:,4))
            xlabel('$t$','interpreter','latex', 'FontSize', 20)
            ylabel('$\lambda_z$','interpreter','latex', 'FontSize', 20)
            grid on
            
        end
        
        function func=cost_func(X,U,e,data,p)
            W=1;
            func=0;
            for i=1:size(U,1)
                f=W*U(i,1:2)*U(i,1:2)';
                func=func+f;
            end
        end
        
        function createVideo(X, Z, L, THETA, fr, path)
            j = 1;
            video_name = strcat(path,'/myVideo');
            if ismac
                % Code to run on Mac platform
                v = VideoWriter(video_name,'MPEG-4');
            elseif isunix
                % Code to run on Linux platform
                v = VideoWriter(video_name);
            elseif ispc
                % Code to run on Windows platform
                v = VideoWriter(video_name,'MPEG-4');
            else
                disp('Platform not supported')
                v = VideoWriter(video_name);
            end
            v.FrameRate = fr;
            open(v)
            shape = [500, 500];
            while j < length(X)
                %fig = figure(); % Explicitly create figure

                utils.drawRobot(X(j), Z(j), L(j), THETA(j))
                %axis([0 5 0 1000]);  % first plot, then change axis
                width = max(X)+0.5-(min(X)-0.5);
                xlim([min(X)-0.5, max(X)+0.5])
                ylim([0.8-width/2, 0.8+width/2])
                frame = getframe(gcf);
                
                if size(frame.cdata, 1)~=shape(1) || size(frame.cdata, 2)~=shape(2)
                    %new_frame = uint8(ones(shape(1), shape(2), 3)*255);
                    %new_frame(1:size(frame.cdata, 1), 1:size(frame.cdata, 2), :) = frame.cdata;
                    new_frame = imresize(frame.cdata,[shape(1) shape(2)]);
                    frame.cdata = new_frame;
                end
                writeVideo(v,frame);
                %close(fig)  % close figure explicitly.
                j = j + 1;
                % pause(0.01)
            end
            close(v)
            if ismac
                % Code to run on Mac platform
            elseif isunix
                % Code to run on Linux platform
                % If Linux, convert AVI to MP4 using ffmpeg
                command = strcat('ffmpeg -i',{' '},video_name,'.avi',{' '},video_name,'.mp4');
                command = command{1};
                system(command);
            end
       end
        function drawRobot(x, z, l, theta)
            circle([x, z], 0.17, 'color', 'black', 'LineWidth', 2);
            hold on
            theta_deg = theta*180/pi;
            utils.drawRectangleonImageAtAngle([x+sin(theta)*l/2; z+cos(theta)*l/2], 0.1, l, theta_deg);
            circle([x+sin(theta)*l, z+cos(theta)*l], 0.17, 'color', 'black', 'LineWidth', 2);
            hold off

        end
        function drawRectangleonImageAtAngle(center,width, height,angle)
            hold on;
            theta = angle*(pi/180);
            coords = [center(1)-(width/2) center(1)-(width/2) center(1)+(width/2)  center(1)+(width/2);...
                      center(2)-(height/2) center(2)+(height/2) center(2)+(height/2)  center(2)-(height/2)];
            R = [cos(theta) sin(theta);...
                -sin(theta) cos(theta)];

            rot_coords = R*(coords-repmat(center,[1 4]))+repmat(center,[1 4]);
            rot_coords(:,5)=rot_coords(:,1);
            line(rot_coords(1,:),rot_coords(2,:), 'color', 'red');

        end
        
        function phase = temporal_phase(t,tf1,tf2)
            
            if t <= tf1
                phase = 1;
            elseif t > tf1 && t <= tf2
                phase = 2;
            elseif t > tf2
                phase = 3;
            end
            
        end
        
    end
    

    methods
        
        function obj =utils(Rw)
            obj.Rw = Rw;
        end
        
       function [eq]=eqConfunction(obj,X,U,e,data)

           X6 = X(2:end,6);
           X8 = X(2:end,8);
           xd = [2 obj.Rw 0 0.5 0 0 0 0 0 0];
           
           eq = [X6-obj.Rw*X8;
               X(end,1)-xd(1);
               X(end,2)-xd(2);
               X(end,3)-xd(3);
               X(end,4)-xd(4);
               X(end,5)-xd(5);
               X(end,6)-xd(6);
               X(end,7)-xd(7);
               X(end,8)-xd(8);
               X(end,9)-xd(9);
               X(end,10)-xd(10)];
           
%             for i=1:size(X,1)
%                 eq(i)=X(i,6)-obj.Rw*X(i,8);
%             end    
       end
       
       function [ineq]=ineqConfunction(obj,X,U,e,data)
           
           U3 = U(2:end,3);
           U4 = U(2:end,4);
           
           ineq = abs(U3) - U4;
           
%            for i=1:size(U,1)
%                ineq(i)=abs(U(i,3))-U(i,4);
%            end
       end
        
    end
    
end

