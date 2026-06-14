clear variables; clc; close all;

% Anthropometric Data utilizing Winter's Biomechanics and Motor Control of 
% Human Movement tables
M = 56.7;
g = 9.81;

lfoot = 12.2e-2;
mfoot = 0.0145 * M;
Ifoot = mfoot * (0.475 * lfoot)^2;

lleg = 42.5e-2;
mleg = 0.0465 * M;
Ileg= mleg * (0.302 * lleg)^2;

lthigh= 31.4e-2;
mthigh = 0.1 * M;
Ithigh = mthigh * (0.323 * lthigh)^2;


load('Project2Data.mat');
smoothing_value = 7;
Frame = MarkerData(:,1);
N  = length(Frame);
Time = MarkerData(:,2);
DT = Time(end) / (N - 1);

%shmooth the data
for col = 3:18
    MarkerData(:,col) = movmean(MarkerData(:,col), smoothing_value);
end

%%DEFINE ALL THE SPECIFIC COLUMNS OF THE DATA SET
Hipx   = MarkerData(:,5);  Hipy   = MarkerData(:,6);
Kneex  = MarkerData(:,7);  Kneey  = MarkerData(:,8);
Fibx   = MarkerData(:,9);  Fiby   = MarkerData(:,10);
Anklex = MarkerData(:,11); Ankley = MarkerData(:,12);
%metatarsals head(the bony bump at the outer edge of the ball of your
%foot
Metax  = MarkerData(:,15); Metay  = MarkerData(:,16);

Ground_x  = ForceData(:,3);
Ground_y  = ForceData(:,4);
CoP = ForceData(:,5);


%%FOOT
%foot angles and storing it in a matrix
ThetaFoot = zeros(N,1);
for i = 1:N

    ThetaFoot(i) = atan2(Metay(i)-Ankley(i), Metax(i)-Anklex(i));
    %checks if the angle is negative and then corrects it 
    
end

%the unwrap function scans through and whenever it sees a jump larger
    %than pi it adds or subtracts the appropriate multiple of pi in order to
    %smooth it out
ThetaFoot = unwrap(ThetaFoot);  

%creates the matrices of x and y cords of the com of the foot
FootComx = zeros(N,1); FootComy = zeros(N,1);
for i = 1:N
    ufoot = [Metax(i)-Anklex(i), Metay(i)-Ankley(i)];
    ufoot = ufoot / norm(ufoot);
    FootComx(i) = Anklex(i) + ufoot(1) * 0.5 * lfoot;
    FootComy(i) = Ankley(i) + ufoot(2) * 0.5 * lfoot;
end


%calculating the linear and angular acceleration

%this one calculates the linear acceleration and velocity while also
%smoothing the data once again after each time we take the derivative
[FootComvx, FootComvy] = finiteDiff(FootComx, FootComy, DT, N);
FootComvx = movmean(FootComvx,smoothing_value);
FootComvy = movmean(FootComvy,smoothing_value);
[FootComax, FootComay] = finiteDiff(FootComvx, FootComvy, DT, N);


%this one calculates the angular velocity and smooths the data
FootOmega = centralDiff1D(ThetaFoot, DT, N);
FootAlpha = centralDiff1D(FootOmega,  DT, N);
FootComax = movmean(FootComax,smoothing_value);
FootComay = movmean(FootComay,smoothing_value);
FootOmega = movmean(FootOmega, smoothing_value);
FootAlpha = movmean(FootAlpha,smoothing_value);

%calculates the forces and moments of the foot
Fanklex = zeros(N,1); Fankley = zeros(N,1); Mankle = zeros(N,1);
for i = 1:N
    Fanklex(i) = mfoot * FootComax(i) - Ground_x(i);
    Fankley(i) = mfoot * (FootComay(i) + g) - Ground_y(i);
    ufoot   = [Metax(i)-Anklex(i), Metay(i)-Ankley(i)];
    ufoot   = ufoot / norm(ufoot);
    %starting at the center of the foot you go back to the ankle
    rankle  = -0.5 * lfoot * ufoot;
    rground = [CoP(i) - FootComx(i), -FootComy(i)];
    Mgnd    = rground(1)*Ground_y(i) - rground(2)*Ground_x(i);
    Mank    = rankle(1)*Fankley(i) - rankle(2)*Fanklex(i);
    Mankle(i) = Ifoot * FootAlpha(i) - Mgnd - Mank;
end



%just repeat the steps from the foot and translate that the leg and the
%thigh
%%LEG
ThetaLeg = zeros(N,1);
for i = 1:N
    ThetaLeg(i) = atan2(Fiby(i)-Ankley(i), Fibx(i)-Anklex(i));
end
 ThetaLeg = unwrap(ThetaLeg);

LegComx = zeros(N,1); LegComy = zeros(N,1);
for i = 1:N
    uleg = [Fibx(i)-Anklex(i), Fiby(i)-Ankley(i)];
    uleg = uleg / norm(uleg);
    LegComx(i) = Anklex(i) + uleg(1) * 0.567 * lleg;
    LegComy(i) = Ankley(i) + uleg(2) * 0.567 * lleg;
end

[LegComvx, LegComvy] = finiteDiff(LegComx,  LegComy,  DT, N);
LegComvx = movmean(LegComvx,smoothing_value);
LegComvy = movmean(LegComvy,smoothing_value);
[LegComax, LegComay] = finiteDiff(LegComvx, LegComvy, DT, N);
LegOmega = centralDiff1D(ThetaLeg, DT, N);
LegAlpha = centralDiff1D(LegOmega,  DT, N);
LegComax = movmean(LegComax,smoothing_value);
LegComay = movmean(LegComay,smoothing_value);
LegOmega = movmean(LegOmega, smoothing_value);
LegAlpha = movmean(LegAlpha,smoothing_value);

Fkneex = zeros(N,1); Fkneey = zeros(N,1); Mknee = zeros(N,1);
for i = 1:N
    Fkneex(i) = mleg * LegComax(i) + Fanklex(i);
    Fkneey(i) = mleg * (LegComay(i) + g) + Fankley(i);

    uleg = [Fibx(i)-Anklex(i), Fiby(i)-Ankley(i)];
    uleg = uleg / norm(uleg);

    % vectors from leg CoM to ankle and knee
    %need the negative here because uleg is going from the ankle to the
    %knee, so when you start at the com you need to go back down to the
    %ankle in the opposite direction of the unit vector, thus the negative.
    rankle_leg = -0.567 * lleg * uleg;
    rknee_leg  =  0.433 * lleg * uleg;

    Mank_onleg = rankle_leg(1)*(-Fankley(i)) - rankle_leg(2)*(-Fanklex(i));
    Mknee_r    = rknee_leg(1)*Fkneey(i)      - rknee_leg(2)*Fkneex(i);

    Mknee(i) = Ileg * LegAlpha(i) - Mknee_r - Mank_onleg + Mankle(i);
end

%% Thigh
ThetaThigh = zeros(N,1);
for i = 1:N
    ThetaThigh(i) = atan2(Hipy(i)-Kneey(i), Hipx(i)-Kneex(i));
end
ThetaThigh = unwrap(ThetaThigh);

ThighComx = zeros(N,1); ThighComy = zeros(N,1);
for i = 1:N
    uthigh = [-Kneex(i)+Hipx(i), -Kneey(i)+Hipy(i)];
    uthigh = uthigh / norm(uthigh);
    ThighComx(i) = Hipx(i) - uthigh(1) * 0.433 * lthigh;
    ThighComy(i) = Hipy(i) - uthigh(2) * 0.433 * lthigh;
end

[ThighComvx, ThighComvy] = finiteDiff(ThighComx,  ThighComy,  DT, N);
ThighComvx = movmean(ThighComvx,smoothing_value);
ThighComvy = movmean(ThighComvy,smoothing_value);
[ThighComax, ThighComay] = finiteDiff(ThighComvx, ThighComvy, DT, N);

ThighOmega = centralDiff1D(ThetaThigh, DT, N);
ThighAlpha = centralDiff1D(ThighOmega,  DT, N);
ThighComax = movmean(ThighComax,smoothing_value);
ThighComay = movmean(ThighComay,smoothing_value);
ThighOmega = movmean(ThighOmega, smoothing_value);
ThighAlpha = movmean(ThighAlpha,smoothing_value);

Fhipx = zeros(N,1); 
Fhipy = zeros(N,1); 
Mhip = zeros(N,1);
for i = 1:N
    Fhipx(i) = mthigh * ThighComax(i) + Fkneex(i);
    Fhipy(i) = mthigh * (ThighComay(i) + g) + Fkneey(i);

    uthigh = [-Kneex(i)+Hipx(i), -Kneey(i)+Hipy(i)];
    uthigh = uthigh / norm(uthigh);

    % vectors from thigh CoM to knee and hip
    %similar thing to the knee except we are doing it in the opposite order
    % the unit vector is starting at the hip and ending at the knee
    %therefore going back up the hip means a negative sign
    rknee_th =  -0.567 * lthigh * uthigh;
    rhip_th  = 0.433 * lthigh * uthigh;

    Mknee_onth = rknee_th(1)*(-Fkneey(i)) - rknee_th(2)*(-Fkneex(i));
    Mhip_r = rhip_th(1)*Fhipy(i) - rhip_th(2)*Fhipx(i);

    Mhip(i) = Ithigh * ThighAlpha(i) - Mhip_r - Mknee_onth + Mknee(i);
end


%plots

%plots the moments of the hip knee and ankle on the same graph
figure('Name','Joint Moments','NumberTitle','off');
plot(Time, Mankle,'b-', 'LineWidth',2); 
hold on;
plot(Time, Mknee,'r-', 'LineWidth',2);
plot(Time, Mhip,   'g-', 'LineWidth',2);
legend('Ankle','Knee','Hip', 'Location','best');
xlabel('Time (s)'); ylabel('Moment (N\cdotm)');
title('Joint Moments vs Time');
grid on;

% this sequence makes 6 subplots. Each with either the x or y component of
% a joint force with relation to time
figure('Name','Intersegmental Forces','NumberTitle','off');
subplot(3,2,1); plot(Time, Fanklex,'b-','LineWidth',2); grid on;
xlabel('Time (s)'); ylabel('Fx (N)'); title('Ankle Fx');
subplot(3,2,2); plot(Time, Fankley,'b-','LineWidth',2); grid on;
xlabel('Time (s)'); ylabel('Fy (N)'); title('Ankle Fy');
subplot(3,2,3); plot(Time, Fkneex,'r-','LineWidth',2); grid on;
xlabel('Time (s)'); ylabel('Fx (N)'); title('Knee Fx');
subplot(3,2,4); plot(Time, Fkneey,'r-','LineWidth',2); grid on;
xlabel('Time (s)'); ylabel('Fy (N)'); title('Knee Fy');
subplot(3,2,5); plot(Time, Fhipx,'g-','LineWidth',2); grid on;
xlabel('Time (s)'); ylabel('Fx (N)'); title('Hip Fx');
subplot(3,2,6); plot(Time, Fhipy,'g-','LineWidth',2); grid on;
xlabel('Time (s)'); ylabel('Fy (N)'); title('Hip Fy');

% HELPER FUNCTIONS
%these functions essentially operate the same way. They use a loop that iterates 
%through the second point all the way through the
%second to last point by looking at the point just before and just
%after and calculating the difference between those two and dividing
%this by two. this essentially calculates a rough estimate of the
%current slope

%this calculates the linear derivatives because it smooths both the x and y
%component
function [vx, vy] = finiteDiff(x, y, DT, N)
    vx = zeros(N,1); vy = zeros(N,1);
    vx(1) = (x(2) - x(1)) / DT;
    vy(1) = (y(2) - y(1)) / DT;
    for i = 2:N-1
        vx(i) = (x(i+1) - x(i-1)) / (2*DT);
        vy(i) = (y(i+1) - y(i-1)) / (2*DT);
    end
    vx(N) = (x(N) - x(N-1)) / DT;
    vy(N) = (y(N) - y(N-1)) / DT;
end

%this calculates the angular derivatives because it only has one thing to
%smooth
function dz = centralDiff1D(z, DT, N)
    dz = zeros(N,1);
    %this specifically calculates the first points velocity
    dz(1) = (z(2) - z(1)) / DT;
   
    for i = 2:N-1
        dz(i) = (z(i+1) - z(i-1)) / (2*DT);
    end
    dz(N) = (z(N) - z(N-1)) / DT;
end