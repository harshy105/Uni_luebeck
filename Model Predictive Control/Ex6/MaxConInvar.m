function C_inf = MaxConInvar(A,B,X,U)

Omega = X;
while true
    Omega_new = Omega.intersect(inv(A)*(Omega+(-B*U)));
    
    if Omega_new >= Omega
        C_inf = Omega_new;
        break;
    end   
    Omega = Omega_new; 
end

end