function A = roty(degree)
A = [cosd(degree),0,sind(degree);
    0,1,0;
    -sind(degree),0,cosd(degree)];
end