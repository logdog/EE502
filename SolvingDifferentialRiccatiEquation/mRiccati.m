function dPdt = mRiccati(P,Q,Ri,A,B)
P = reshape(P, size(A)); % convert P into a matrix
dPdt = -Q + P*B*Ri*B.'*P - P*A - A.'*P; % DRE
dPdt = dPdt(:); % convert dP to column vector