function [Vref]= maxspeed(x)
% % function Vel_obj= Uref(Y)
global veloc

%%%%%%%%%%%%%%%%%%%%%%%%%
% Borrellis version:
% l0= size(veloc,1);
% for k=1:length(x)
%     iactual=1;
%     Vref(k)=veloc(iactual,2)/3.6;
%     
%     for i= 1:l0-1
%         if x(k) > veloc(i,1) && x(k) <= veloc(i+1,1)
%             Vref(k)= veloc(i,2)/3.6;
%         end
%     end
%     if x(k) >= veloc(l0,1)
%         Vref(k)= veloc(l0,2)/3.6;
%     end
    
%%%%%%%%%%%%%%

% alternative way: use linear interpolation for velocity
vel = @(v) 1/3.6 * interp1(veloc(:,1),veloc(:,2),v,'previous');
Vref = vel(x);
end

