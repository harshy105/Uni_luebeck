function O_inf = MaxPosInvar(pre,X)

Omega = X;
while true
    Omega_new = Omega.intersect(pre*Omega);
    
    if Omega_new >= Omega
        O_inf = Omega_new;
        break;
    end   
    Omega = Omega_new; 

end

end