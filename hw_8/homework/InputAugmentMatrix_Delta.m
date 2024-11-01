function [Aa, Ba, Ca, Sa, Qa, R] = InputAugmentMatrix_Delta(A, B, Ad, S, Q, R)
    n = size(A,1);
    p = size(B,2);
    Aa = [A zeros(n) B; zeros(n) Ad zeros(n,p); zeros(p,n) zeros(p,n) eye(p)];
    Ba = [B; zeros(n,p); eye(p)];
    Ca = [eye(n) -eye(n) zeros(n,p)];
    Sa = Ca'*S*Ca;
    Qa = Ca'*Q*Ca;
    R = R;
end