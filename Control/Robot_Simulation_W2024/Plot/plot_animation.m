function plot_animation(sim)
    % Animation of resulting dynamics
    figure(1)
    
    %% Load parameters from main struct
    b                       = sim.robot.b;
    a                       = sim.robot.a;
    r_wheel                 = sim.robot.r_wheel;
    
    position                = sim.position;
    servo_angle             = sim.servo_angle;
    robot_direction         = sim.robot_direction;
    fire_piston             = sim.piston_fired;
    wheel_encoder           = sim.wheel_encoder;
    position_desired        = sim.position_desired;
    position_estimate       = sim.position_estimate;
    grid_size               = sim.grid_size;
    scores                  = sim.score;
    
    time                    = sim.time;
    dt                      = sim.dt;
    view_style              = sim.params.view_style;
    playback_speed          = sim.params.playback_speed;
    fps                     = sim.params.fps;
    
    % load control variables, just to get the list of targets for the
    % closed-loop control
    control_type            = sim.vars.control_type;
    position_desired_list   = sim.vars.c3_position_desired_list;
    
    %% Plot simulation
    frames = round(2:(1/dt)*(playback_speed/fps):length(time));
    
    wall = [[0 0 NaN 0 0]' grid_size*[-2.5 -.5 NaN .5 2.5]'];
    course = grid_size*[[-3 8 8 NaN 8 8 -3 -3]' [-2.5 -2.5 -.5 NaN .5 2.5 2.5 -2.5]'];
    b_offset = 1;
    
    for k = 1:length(frames)
        clf
        hold on
        n = frames(k);
        R = @(servo_angle) [cos(servo_angle) sin(servo_angle);-sin(servo_angle) cos(servo_angle)];
        tire = [6*cos(0:pi/100:2*pi)'+b/2-b_offset 6*sin(0:pi/100:2*pi)'];
        wheels = [[ -b/2-r_wheel                -b/2+r_wheel NaN ...
                    -b/2-r_wheel                -b/2+r_wheel NaN ...
                     b/2-r_wheel*cos(servo_angle(n))   b/2+r_wheel*cos(servo_angle(n))]'+b/2 ...
                    [a/2 a/2 NaN -a/2 -a/2 NaN ...
                    -r_wheel*sin(servo_angle(n))       r_wheel*sin(servo_angle(n))]'];
        tire = tire*R(robot_direction(n)) + repmat(position(n,:),[size(tire,1) 1]);
        wheels = wheels*R(robot_direction(n)) + repmat(position(n,:),[size(wheels,1) 1]);

        ind = time <= time(n);        
        % actual trajectory
        plot(position(ind,1),position(ind,2),'b','LineWidth',2)
        % markers for piston fire
        plot(position(logical(fire_piston.*ind),1),position(logical(fire_piston.*ind),2),'b.','MarkerSize',20)
        % estimated trajectory
        plot(position_estimate(ind,1),position_estimate(ind,2),'r','LineWidth',2)
        % reed switch ticks
        plot(position_estimate(logical(diff([0;wheel_encoder]).*ind),1),position_estimate(logical(diff([0;wheel_encoder]).*ind),2),'rx','MarkerSize',10)
        
        % targets, only if closed loop control
        if(control_type == 3)
            plot(position_desired_list(:,1),position_desired_list(:,2),'.','Color',[.7 .7 .7],'LineWidth',3,'MarkerSize',40)
            plot(position_desired(n,1),position_desired(n,2),'.','Color',[.1 .1 .1],'LineWidth',3,'MarkerSize',40)
        end
 
        % plot robot
        plot(tire(:,1),tire(:,2),'Color',[.5 .5 .5],'LineWidth',2)
        plot(wheels(:,1),wheels(:,2),'k','LineWidth',2)
        
        % plot course
        plot(wall(:,1),wall(:,2),'r','LineWidth',3)
        plot(course(:,1),course(:,2),'--','Color',[.75 .75 .75],'LineWidth',3)

        axis('equal')
        if view_style == 1
            axis(grid_size*[-4 9 -3.5 3.5]);
        elseif view_style == 2
            axis(.5*grid_size*[-4 9 -3.5 3.5]+[position(n,1)*[1 1] position(n,2)*[1 1]]);
        end
        
        
        set(gca,'PlotBoxAspectRatio',[12 7 1])
        set(gca,'Xtick',grid_size*(-3:8))
        set(gca,'Ytick',grid_size*(-2.5:2.5))
        grid on
        text(-3.5*grid_size,3*grid_size,['Score: ' num2str(scores(n)) ' | Position: (' num2str(position(n, 1), '%.0f') ', ' num2str(position(n, 2), '%.0f') ')'] , 'FontSize', 20)
        drawnow
    end
    legend('Robot Position','Piston Fires','Estimated Position','Reed Switch Tick')
end
