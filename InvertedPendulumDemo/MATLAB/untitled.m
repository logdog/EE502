syms p z
M(z) = p*z / (1-z*(1-p));

% calculate M'(1) = E[X]
tmp = simplify( diff(M,z) );
m1 = subs(tmp, z, 1)

% calculate M''(1) = E[X^2] - E[X]
tmp = simplify( diff(diff(M,z),z) );
m2 = subs(tmp, z, 1)

% calculate variance
mu2 = simplify(m2 + m1)
sigma2 = simplify(mu2 - m1^2)

