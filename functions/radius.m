function r= radius(X)
    
    global profile

%%%%%%%%%%%%%%%%
% Borrellis version:
%     l0=size(profile,1);
%     
%     r=profile(1,3);
%     for i=1:l0-1
%         if X >= profile(i,1) && X < profile(i+1,1)
%             r= profile(i,3);
%             break
%         end
%     end
%     if X >= profile(l0,1)
%        r= profile(l0,3);
%     end
%%%%%%%%%%%%%%%%%%%%%


% alternative way: use linear interpolation for slope
rad = @(v) interp1(profile(:,1),profile(:,3),v,'linear');
r = rad(X);
end

