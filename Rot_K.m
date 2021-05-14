
%绕固定轴旋转某一角度后的变换矩阵，需要先对矢量K进行单位化
%与Inverse_Rot_K(R)配对
function R = Rot_K(K,q)

        cq = cos(q); 
        sq = sin(q);
        vq = 1-cos(q);
               
        k_det = sqrt(K(1)^2 + K(2)^2 +K(3)^2);
        k_unit = K/k_det;   %----单位化
        
        kx = k_unit(1);  
        ky = k_unit(2);  
        kz = k_unit(3);
       
        
        R = [kx*kx*vq + cq, ky*kx*vq - kz*sq, kz*kx*vq + ky*sq
             kx*ky*vq + kz*sq, ky*ky*vq + cq, kz*ky*vq - kx*sq
             kx*kz*vq - ky*sq, ky*kz*vq + kx*sq, kz*kz*vq + cq];