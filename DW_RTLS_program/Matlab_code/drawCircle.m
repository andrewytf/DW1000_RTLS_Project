function drawCircle( iCent, iRad )
%drawCircle Draws a circle with center iCent and radius iRad
%   iCent : Array of 2 elements
%       Center position (x,y) of circle
%   iRad : float
%       Radius of circle
    
    pos = [iCent(1)-iRad, iCent(2)-iRad, 2*iRad, 2*iRad];
    rectangle('Position',pos,'Curvature',[1 1],'Linestyle', '--', 'LineWidth',1);

end

