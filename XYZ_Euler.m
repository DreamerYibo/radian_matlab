
function R_xyz = XYZ_Euler(alpha,beta,gamma)

%   R_xyz = Rot_x(alpha)*Rot_y(beta)*Rot_z(gamma);

  sa = sin(alpha);  ca = cos(alpha);
  sb = sin(beta);   cb = cos(beta);
  sr = sin(gamma);  cr = cos(gamma);  
  
 R_xyz = [ cb*cr,            -cb*sr,           sb;
           sa*sb*cr+ca*sr,  -sa*sb*sr+ca*cr,  -sa*cb;
	       -ca*sb*cr+sa*sr,  ca*sb*sr+sa*cr,   ca*cb];

  
              
 
  
  
        
   