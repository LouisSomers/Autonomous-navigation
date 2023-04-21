%% Calculation moment of Inertia

%dimensions and weight thrusters
l = 0.113;
r = 0.05;
m = 0.344;
Ixx_thrust = (1/12)*m*(3*r^2+l^2+0.075^2)
Iyy_thrust =0.5*m*(r^2+0.09^2+0.075^2)
Izz_thrust = (1/12)*m*(3*r^2+l^2+0.09^2)
%dimensions and weight boogieboard
l = 0.845;
w = 0.46;
h = 0.05;
m = 0.1;
Ixx_board = (1/12)*m*(l^2+h^2)
Iyy_board = (1/12)*m*(w^2+h^2)
Izz_board = (1/12)*m*(l^2+h^2)

%dimensions and weight watertight box
l = 0.3;
w = 0.235;
h = 0.18;
m = 3;
Ixx_box = (1/12)*m*(l^2+h^2+0.115^2)
Iyy_box = (1/12)*m*(w^2+h^2+0.115^2)
Izz_box = (1/12)*m*(l^2+h^2)

Ixx_tot = Ixx_box+Ixx_thrust*2+Ixx_board
Iyy_tot = Iyy_box+Iyy_thrust*2+Iyy_board
Izz_tot = Izz_box+Izz_thrust*2+Izz_board




