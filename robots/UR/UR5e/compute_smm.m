function smm = compute_smm(T,u,N)

u = u/norm(u);

R = T(1:3,1:3);

d_theta = 2*pi/(N-1);
smm = zeros(N*8,6);
counter = 1;

theta = -pi;
for i=1:N
    T(1:3,1:3) = R*(cos(theta)*eye(3)+sin(theta)*[0 -u(3) u(2);u(3) 0 -u(1);-u(2) u(1) 0]+(1-cos(theta))*u*(u'));
    qinv = ikUR5e(T);
    for j=1:size(qinv,2)
        if isreal(qinv(:,j))
            smm(counter,:) = qinv(:,j)';
            counter = counter + 1;
        end
    end
    theta = theta + d_theta;
end

smm = smm(1:counter-1,:);

end