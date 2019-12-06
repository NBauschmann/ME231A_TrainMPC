function x_ = train_dynamics(x, u, param)
%% calculates the system dynamics
%% inputs
% states x = [s(k), v(k), F(k)] at time k
%   position s
%   velocity v
%   integrated driving/ braking force F
% controlled driving/ breaking force u
% struct param containing the system parameters
%% outputs 
% states x at time k+1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dynamics:
% s(k+1) = s(k) + delta_t * v(k) ;
% v(k+1) = v(k) + delta_t * (-A -B*v(k) -C*v(k)^2 - Fe + F(k))/M ;
% F(k+1) = (1 - delta_t/tau ) * F(k) + delta_t/tau * u(k) ;

% extract states x at time k
s = x(1) ;
v = x(2) ;
F = x(3) ;

% paramters 
delta_t = param.delta_t ;
A = param.A ;
B = param.B ;
C = param.C ;
M = param.M ;
tau = param.tau ;
g = param.g ;

% computation of external force 
Fg = - M * g * slope(s) ; % gravity force component
Fr = - M * 6 / radius(s) ;  % curving resistance
Fe = Fg + Fr ;

% computation of states at time k+1 
s_ = s + delta_t * v ;
v_ = v + delta_t * (-A -B*v -C*v^2 + Fe + F)/M ;
F_ = (1 - delta_t/tau ) * F + delta_t/tau * u ;

x_ = [s_; v_; F_] ;

end