pos_x = 0;
pos_y = 0;
new_pos_x = -1; 
new_pos_y = -1;
v1 = 0;
v2 = 0;
v3 = 1;

theta = atan((new_pos_y-pos_y)/(new_pos_x-pos_x))
theta2 = atan2((new_pos_y-pos_y),(new_pos_x-pos_x))
x = v1 * sin(theta2/2)
y = v2 * sin(theta2/2)
z = v3 * sin(theta2/2)
w = cos(theta2/2)