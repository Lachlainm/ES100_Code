clear all;

Fo = [-4; -4; 18];
data = zeros(360, 3);
for i =1:360
    F = rotz(i) * Fo;
    f = norm(F);
    
    theta = atan2(-F(2),F(3));
    
    phi = asin(F(1)/f);
    data(i, :) = [i, theta, phi];
    Fz = F(3);
end

figure(1);
plot(data(:,1), data(:,2));
title("Theta");
grid('on')

figure(2)
plot(data(:,1), data(:,3));
title("Phi");
grid('on')