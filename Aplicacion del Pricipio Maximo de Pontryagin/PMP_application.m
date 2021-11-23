close all;
% Initial Conditions
lambda_1 = 0;
lambda_2 = 1;

theta = -3*pi/4;

x = 0;
y = 0;
x_T = 1000;
V_p = 2;
V_e = 1;
rho = V_e/V_p;
x_e = inf;
L = 1;

dx = 0.01;
epsilon = 0.001;
for j = 1:1:(x_T/dx)
    % Chose alpha(j) that minimizes the Hamiltonian
    alpha(j) = 0;
    d_alpha = 0.001;
    H_last = inf;
    for a = -pi/2:d_alpha:pi/2
    %for a = 0:d_alpha:2*pi
        H = lambda_1*tan(a) + (1 + lambda_2(j))*((-rho * sin(a - theta(j)) + sqrt(1 - (rho*cos(a - theta(j))).^2))./(L*rho*cos(a)));
        if H < H_last
            H_last = H;
            a0 = a;
        end
    end
    alpha(j) = a0;
    
    % Integrate to get y(j+1) and theta(j+1)
    f1 = tan(alpha(j));
    
    dtheta_dx = (-rho*sin(alpha(j) - theta(j)) + sqrt(1-(rho*cos(alpha(j)-theta(j))).^2))./(L*rho*cos(alpha(j)));
    if dtheta_dx > 0
        f2 = dtheta_dx;
    else
        f2 = (-rho*sin(alpha(j) - theta(j)) - sqrt(1-(rho*cos(alpha(j)-theta(j))).^2))./(L*rho*cos(alpha(j)));
    end
    
    y(j+1) = y(j) + f1*dx;
    x(j+1) = x(j) + dx;
    theta(j+1) = theta(j) + f2*dx;
    
    lambda_2(j+1) = lambda_2(j) - (1 + lambda_2(j)) * cos(alpha(j)-theta(j))/(L*cos(alpha(j))) * (1 - rho*sin(alpha(j) - theta(j))/sqrt(1-(rho*cos(alpha(j)-theta(j))).^2))*dx;
    
    if sign(y(j)) * sign(y(j+1)) < 0
        if abs(x(j+1) - x_e) < epsilon
            disp('Optimal trayectory given by alpha')
        else
            disp('Boundary condition missed')
        end
        
        break
    end
   
end


x_p = x + L*cos(theta);
y_p = y + L*sin(theta);


for i = 1:1:length(x)    
    if mod(i,5) == 0 || i == 1
        X = [x(i),x_p(i)];
        Y = [y(i),y_p(i)];
        plot(X,Y, '-k')
        hold on
        grid on
        plot(x_p(i), y_p(i), 'sb', 'MarkerSize',10, 'MarkerEdgeColor','blue','MarkerFaceColor',[0.5 0.5 1])
        plot(x(i), y(i), 'or', 'MarkerSize',10, 'MarkerEdgeColor','red','MarkerFaceColor',[1 0.5 0.5])
        
        legend('L','Pursuer', 'Evader')
    else
        plot(x_p(i), y_p(i), 'ob', 'MarkerSize',1, 'MarkerEdgeColor','blue','MarkerFaceColor',[0.5 0.5 1])
        plot(x(i), y(i), 'or', 'MarkerSize',1, 'MarkerEdgeColor','red','MarkerFaceColor',[1 0.5 0.5])
    end
end