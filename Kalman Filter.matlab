%%Code in Matlab
function [ x,P ] = ekf(f,x,P,h,z,Q,R )
%This is tentative test of Extended Kalman Filter used for quaternion
%attitude estimation
n=3; %number of trials
q=[0,0.00045332,0.00043076,0.00001588,0.03729,0.03460,0.03512]; % std of process
r=[0.00045332,0.00043076,0.00001588]; %std of measurement
Q=diag(q)^2; % covariance of process
R=diag(r)^2; % covariance of measurement
f= % this is the state transition matrix
h= % this is the measurement matrix used to transfer direct measurement into Euler angle
s=[1,0,0,0,0.180776,0.654646,-0.91071]; %initial state
x=s+q*diag(randn(1,7)); %initial state with noises
P=10^-7*eye(7); %initial state covariance
m=[ax,ay,az,p,q,r,bx,by,bz] %measurement of current state from IMU sensor
N=20;                                     % total dynamic steps
xV = zeros(n,N);          %estmate        % allocate memory
sV = zeros(n,N);          %actual
zV = zeros(1,N);
for k=1:N
    z=h(ax,ay,az,bx,by,bz)+r*randn; %measurement process
    sV(:,k)=s;                      %save actual state
    zV(k)= z;                     %save measurment
   % [x,P]=ekf %this function is confusing here, why we can call this function without completely define it?
    x=f(m(4)-0.180776,m(5)-0.654646,m(6)-0.91071,s(1),s(2),s(3),s(4)) %this is prediction about next state
    xV(:,k) = x;                            % save estimate
    s=x+q*diag(randn(1,7)) %updated state with process noises
    P=f(m(4)-0.180776,m(5)-0.654646,m(6)-0.91071,s(1),s(2),s(3),s(4))*p*((f(m(4)-0.180776,m(5)-0.654646,m(6)-0.91071,s(1),s(2),s(3),s(4))'))+Q
end
end
Supporting Function (Matrix operation)
%this function is used to linearize the measurement matrix, which converts
%the state variable into Euler angle
function[matrix]=Hk(qs,qx,qy,qz)
syms A B C
A=atan(2*(qy*qz+qs*qx)/(1-2*(qx^2+qy^2)));
B=-1*asin(2*(qx*qz-qs*qy));
C=atan(2*(qx*qy+qs*qz)/(1-2*(qy^2+qz^2)));
matrix=[diff(A,qs),diff(A,qx),diff(A,qy),diff(A,qz);
        diff(B,qs),diff(B,qx),diff(B,qy),diff(B,qz);
        diff(C,qs),diff(C,qx),diff(C,qy),diff(C,qz);]
end
% This function f is used to predict the next state
function[matrix1]=f(p,q,r,qs,qx,qy,qz,t)
matrix1=eye(7)+0.5*t*[0,-1*(p-0.180776),-1*(q-0.654646),-1*(r+0.91071),qx,qy,qz;
                      (p-0.180776),0,(r+0.91071),-1*(q-0.654646),-1*qs,qz,-1*qy;
                     (q-0.654646),-1*(r+0.91071),0,(p-0.180776),-1*qz,-1*qs,qx;
                     (r+0.91071),(q-0.654646),-1*(p-0.180776),0,qy,-1*qx,-1*qs;
                     0,0,0,0,0,0,0;
                     0,0,0,0,0,0,0;
                     0,0,0,0,0,0,0;]
end
%This is a function used to convert current measurement output from
%accelerometer and magnetometer into the Euler angles
function[matrix]=h(ax,ay,az,bx,by,bz)
matrix=[atan(-1*ay/az);
       asin(-1*ax/sqrt(ax^2+ay^2+az^2));
       atan(by*cos(atan(ay/az)-bz*sin(atan(ay/az))))/bx*cos(atan(-1*ax/sqrt(ay^2+az^2)))+by*sin(atan(ay/az))*sin(atan(-1*ax/sqrt(ay^2+az^2)))+bz*cos(atan(ay/az))*sin(atan(-ax/sqrt(ay^2+az^2)));]
end
