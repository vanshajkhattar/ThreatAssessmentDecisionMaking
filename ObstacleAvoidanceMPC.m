%Initial Conditions
m = 2270;
x0 = [-15; 17; -4; 0; 0; 0];
u0 = [0.5*m; 0];
% x = x0;

%Defining the system
Ts = 0.2;

[Ad,Bd,Cd,Dd,U,Y,X,DX] = egoVehicle(x0,u0);  %System matrices for the initial condition
dsys = ss(Ad,Bd,Cd,Dd,'Ts',Ts);
dsys.InputName = {'Throttle','Steering angle'};
dsys.StateName = {'X','V_x','Y','V_y','Theta','ThetaDot'};
dsys.OutputName = dsys.StateName;

%% MPC Design at the nominal operating point
status = mpcverbosity('off');
mpcobj = mpc(dsys,Ts);

mpcobj.PredictionHorizon = 5;
mpcobj.ControlHorizon = 1;

%Setting state constraints
mpcobj.OutputVariables(1).Min = 0;
mpcobj.OutputVariables(1).Max = 150;
mpcobj.OutputVariables(2).Min = 0;
mpcobj.OutputVariables(2).Max = 42;
mpcobj.OutputVariables(3).Min = -2;
mpcobj.OutputVariables(3).Max = 2;
mpcobj.OutputVariables(4).Min = -20;
mpcobj.OutputVariables(4).Max = 20;
mpcobj.OutputVariables(5).Min = -pi/2;
mpcobj.OutputVariables(5).Max = pi/2;
mpcobj.OutputVariables(6).Min = -pi/10;
mpcobj.OutputVariables(6).Max = pi/10;


%Min and max of control inputs
mpcobj.ManipulatedVariables(1).Min = -3*m;
mpcobj.ManipulatedVariables(1).Max = 4*m;

mpcobj.ManipulatedVariables(2).Min = -pi/6;
mpcobj.ManipulatedVariables(2).Max = pi/6;

%Min and max rate of control inputs
mpcobj.ManipulatedVariables(1).RateMin = -0.3*m*Ts;
mpcobj.ManipulatedVariables(1).RateMax = 0.3*m*Ts;

mpcobj.ManipulatedVariables(2).RateMin = -pi/10*Ts;
mpcobj.ManipulatedVariables(2).RateMax = pi/10*Ts;

mpcobj.ManipulatedVariables(1).ScaleFactor = 2;
mpcobj.ManipulatedVariables(2).ScaleFactor = 0.2;

% Setting weights for the outputs to be tracked
mpcobj.Weights.OutputVariables = [0 1 1e5 0 0 0];

% Setting weights for the inputs
mpcobj.Weights.ManipulatedVariables = [0.1 0.1];

%Updating the controller with the nominal operating conditions
[Ad0,Bd0,Cd0,Dd0,U0,y0,X0] = egoVehicle(x0, u0);
Y0 = Cd0*x0 + Dd0*u0;
DX0 = Ad0*X0 + Bd0*u0 - x0;
mpcobj.Model.Nominal = struct('U',U0,'Y',Y0,'X',X0,'DX',DX0);

%Specifying avoidance maneuver constraints

%% Code for finding the closest convex hull and its constraints
T = 0:Ts:2;  % Simulation time
P_surr1 = zeros(4,2,size(T,2));
P_surr1(:,:,1) = [-10 -1.5;-10  1.5; 150 -12.7;150 -15.7];

[A,b] = vert2con(P_surr1(:,:,1));
len = size(A,1);
Anew = [A(:,1) zeros(len,1) A(:,2) zeros(len,3)];
A_dash = Anew;
b_dash = b;

E = zeros(len,2);
F = A_dash;
G = b_dash;
%     [E,F,G,safe_hull] = ConstraintEvaluate(k,x);
V = [0;0];
setconstraint(mpcobj,E,F,G,V);

refSignal = [0 15 -2 0 0 0];

%Initialize plant and controller states.
x = x0;
u = u0;
egoStates = mpcstate(mpcobj);

% for i = 2:(size(T,2)-3)
%     P_surr1(:,:,i) = P_surr1(:,:,1);
% end
% % for i = (size(T,2)-9):(size(T,2)-3)
% %     P_surr1(:,:,i) = [0 -1.5; 0 1.5; 150 -4.7;150 -7.7];
% % end
% 
% for i = (size(T,2)-2):size(T,2)
%     P_surr1(:,:,i) = [20 0; 20 -6; 150 -3; 150 -8];
% end


ympc = zeros(length(T),size(Cd,1));
umpc = zeros(length(T),size(Bd,2));

for k = 1:length(T)
    
    [Ad,Bd,Cd,Dd,U,Y,X,DX] = egoVehicle(x, u);
    measurements = Cd*x + Dd*u;
    ympc(k,:) = measurements';
    
    %     Finding constraints at each time instant
    slope = (-6-(ympc(k,3)+2))/(13 - (ympc(k,1)-3));
    
    if ympc(k,1)<-11
%         if slope <0
            k1 = (150-(ympc(k,1)-3))*slope + (ympc(k,3)+2);
            k2 = (150-(ympc(k,1)-3))*slope + (ympc(k,3)-2);
%         else
%             k1 = ympc(k,3)+1.5-6;
%             k2 = ympc(k,3)-1.5-6;
%         end
    else
        k1 = ympc(k,3)+1.5-6;
        k2 = ympc(k,3)-1.5-6;
    end
    
%     P_surr1(:,:,k) = [(1) -2; x(1) x(3)+1.5; 150 k1;150 k2];
    
    P_surr1(:,:,k) = [ympc(k,1)-3  ympc(k,3)+1.5;ympc(k,1)-3  ympc(k,3)-1.5; 150 k1; 150 k2];
    
    [A,b] = vert2con(P_surr1(:,:,k));
    len = size(A,1);
    Anew = [A(:,1) zeros(len,1) A(:,2) zeros(len,3)];
    A_dash = Anew;
    b_dash = b;
    
    E = zeros(len,2);
    F = A_dash;
    G = b_dash;
    %     [E,F,G,safe_hull] = ConstraintEvaluate(k,x);
    V = [0;0];
    setconstraint(mpcobj,E,F,G,V);
    
    newPlant = ss(Ad,Bd,Cd,Dd,'Ts',Ts);
    newNominal = struct('U',U,'Y',Y,'X',X,'DX',DX);
    
    % Prepare new mixed I/O constraints.
    options = mpcmoveopt;
    options.CustomConstraint = struct('E',E,'F',F,'G',G);
    
    [u,Info] = mpcmoveAdaptive(mpcobj,egoStates,newPlant,newNominal,...
        measurements,refSignal,[],options);
    umpc(k,:) = u';
    
    % Update the plant state for the next iteration |k+1|.
    x = Ad * x + Bd * u;
end

hold on;plot(ympc(:,1),ympc(:,3));

%% Ego Vehicle Dynamics
function [Ad,Bd,Cd,Dd,U,Y,X,DX] = egoVehicle(x, u)

Cf = 127000;
Cr = 130000;
lf = 1.421;
lr = 1.434;
Iz = 4600;
m = 2270;

%Initial vector
%State vector
x_ego = x;
% x_ego(:,1) = [0 25 0 0 0 0];

%Input vector
% u_ego = zeros(2,5);   %Tells the size of the input vector for prediction
% horizon N
% u_ego = [[m*1 ;0] [m*1.2; 0.1] [m*1.3; 0.2] [m*1.4; -0.1] [m*1.5; 0]];



%Defining velocity vector
vx = x_ego(2);

%System matrices

A_ego = [0 1 0 0 0 0;
    zeros(1,6);
    zeros(1,3) 1 vx 0;
    zeros(1,3)  -(Cf+Cr)/(m*vx)  0 ((lr*Cr - lf*Cf)/(m*vx))-vx;
    zeros(1,5) 1;
    zeros(1,3) (lr*Cr - lf*Cf)/(Iz*vx) 0 -(lf^2*Cf+lr^2*Cr)/(Iz*vx)];

B_ego = [0 1/m 0 0 0 0;
    0 0 0 Cf/m 0 (lf*Cf)/Iz]';

C_ego = eye(6);

D_ego = zeros(6,2);


egoCont = ss(A_ego,B_ego,C_ego,D_ego);
egoDiscr = c2d(egoCont,0.2);

Ad = egoDiscr.A;
Bd = egoDiscr.B;
Cd = egoDiscr.c;
Dd = egoDiscr.D;

x_ego = egoDiscr.A*x_ego + egoDiscr.B*u;
Y = x_ego;
U = u;
X = x;
DX = Ad*x+Bd*u-x;
end
