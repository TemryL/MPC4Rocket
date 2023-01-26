% Runge-Kutta 4 integration
function [x_next] = RK4(rocket,X,U)
    h = rocket.Ts;
    k1 = rocket.f(X,        U);
    k2 = rocket.f(X+h/2*k1, U);
    k3 = rocket.f(X+h/2*k2, U);
    k4 = rocket.f(X+h*k3,   U);
    x_next = X + h/6*(k1+2*k2+2*k3+k4);
end