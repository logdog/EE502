function x2_tilde = controlLaw(x,g,L,e0)

c = -(sqrt(g/L)+e0);
u0 = c * (sqrt(g/L)*x(3) + x(4));
u0_dot = c * (sqrt(g/L)*x(4) + g/L*x(3) - g/L*x(1));

x2_tilde = x(2) + 1/e0*x(1) + L/(e0*g)*u0 + L/g*u0_dot;
end

