function s = slope(X)
    
      global profile
%%%%%%%%%%%%%%%%
% Borrellis version:
%     l0=size(profile,1);
%     
%     s=profile(1,2)/1000;
%     for i=1:l0-1
%         if X >= profile(i,1) && X < profile(i+1,1)
%             s= profile(i,2)/1000;
%             break
%         end
%     end
%     if X >= profile(l0,1)
%        s= profile(l0,2)/1000;
%     end
%%%%%%%%%%%%%%%

% alternative way: use linear interpolation for slope
slope = @(v) 0.001 * interp1(profile(:,1),profile(:,2),v,'previous');
s = slope(X);

% using spline
% pp = spline(profile(:,1),profile(:,2));
% s = ppval(pp,X);

end

