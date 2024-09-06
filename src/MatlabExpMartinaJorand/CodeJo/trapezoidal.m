% -------------------------------- DESCRIPTION ------------------------------- %
%   x_i     dim 1x1     Initial Value
%   x_f     dim 1x1     Final Value
%   t       dim 1x1     Time Variable
%   t_f     dim 1x1     Final Time
% ---------------------------------------------------------------------------- %
function [x, dx, ddx] = trapezoidal(x_i,x_f,t,t_f)

    % ---------------------------- Auxiliary Variables --------------------------- %
    dx_c = 2.0*(x_f-x_i)/(t_f*1.7); %Cruise Velocity
    t_c = (x_i-x_f + dx_c*t_f)/dx_c; %Cruise Time
    ddx_c = (dx_c^2)/(x_i-x_f + dx_c*t_f); %Cruise Accelaration
    % ---------------------------------------------------------------------------- %

    % ----------------------------------- Body ----------------------------------- %
    if x_i == x_f
        x = x_i;
        dx = 0.0;
        ddx = 0.0;
    else
        if t >= 0 && t <= t_c
            x = x_i + (0.5*ddx_c*t^2);
            dx = ddx_c*t;
            ddx = ddx_c;
        elseif t > t_c && t <= (t_f-t_c)
            x = x_i + (ddx_c*t_c*(t-(0.5*t_c)));
            dx = ddx_c*t_c;
            ddx = 0.0;
        elseif t > (t_f-t_c) && t <= t_f
            x = x_f - (0.5*ddx_c*(t_f-t)^2);
            dx = -(ddx_c*t) + (ddx_c*t_f);
            ddx = -ddx_c;
        elseif t > t_f
            x = x_f;
            dx = 0.0;
            ddx = 0.0;
        end
    end
    % ---------------------------------------------------------------------------- %
    
end