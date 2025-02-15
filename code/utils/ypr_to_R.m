
% *** Internal Use Only - Do Not Distribute ***

function R = ypr_to_R(ypr)

    y = ypr(1);
    p = ypr(2);
    r = ypr(3);        
      
    Rz = [cos(y) -sin(y)   0; ...
          sin(y)  cos(y)   0; ...
          0            0   1];       

    Ry = [cos(p)  0   sin(p); ...
          0       1        0; ...
         -sin(p)  0   cos(p)];  
     
    Rx = [1      0         0; ...
          0   cos(r) -sin(r); ...
          0   sin(r)  cos(r)];     
           
    R = Rz*Ry*Rx;
    
end
