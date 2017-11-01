function [ loc_final ] = getLocationPoint( circCenters, circRadii )
%getLocationPoint Finds the most probable location of tag with three
%intersecting circles
%   circCenters : Matrix 3×2 
%       Centers of three circles
%   circRadii   : Vector of 3 
%       Radii of three circles
%   loc_final   : Vector of 2
%       Computed tag position

offset = circCenters(1,:);

% Move points by offset so center1 is in (0,0)
center2 = circCenters(2,:) - offset;
center3 = circCenters(3,:) - offset;

% get rotation angle of system
fi = atan2(center2(2), center2(1));

% create rotational matrix
Rz = [cos(fi), -sin(fi);
      sin(fi), cos(fi)];

% rotate centers around z axis by angle fi, so center2 will be positioned on x axis
center2 = (Rz' * center2')';
center3 = (Rz' * center3')';

x2 = center2(1);

r1 = circRadii(1);
r2 = circRadii(2);
r3 = circRadii(3);

% get x coordinate of both intersection points between circles 1 and 2
xi = (r1^2 - r2^2 + x2^2) / (2*x2);

% if circles do not intersect, argument r1^2 - x^2 is negative. In this
% case set y of intersection point to 0
arg_temp = r1^2 - xi^2;
if arg_temp < 0
    yi = 0;
else
    yi = sqrt(arg_temp);
end

% locations of both possible intersection points
loc = [xi, yi; xi, -yi];

% use circle 3 to determine more probable location by selecting point,
% which is closer to circle 3
[~,ind] = min([abs(norm(center3-loc(1,:)) - r3), abs(norm(center3-loc(2,:)) - r3)]);
realLoc = loc(ind,:);

% additionally correct intersection point with third distance measurement
circ3toIntersectVect = center3 + (realLoc - center3) / norm(realLoc - center3) * r3;
correctedLoc = circ3toIntersectVect + (realLoc - circ3toIntersectVect) * 0.5;

% rotate computed location back again by rotation angle and translate it by offset
loc_final = (Rz * correctedLoc' + offset')';

end

