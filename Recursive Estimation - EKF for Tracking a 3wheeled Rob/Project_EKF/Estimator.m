function [posEst,oriEst,radiusEst, posVar,oriVar,radiusVar,estState] = Estimator(estState,actuate,sense,tm,knownConst,designPart)
% [posEst,oriEst,posVar,oriVar,baseEst,baseVar,estState] =
% 	Estimator(estState,actuate,sense,tm,knownConst,designPart)
%
% The estimator.
%
% The Estimator function shall be used for both estimator design parts; the
% input argument designPart is used to distinguish the two:
%   designPart==1  -> Part 1
%   designPart==2  -> Part 2
%
% The function will be called in two different modes:
% If tm==0, the estimator is initialized; otherwise the estimator does an
% iteration step (compute estimates for the time step k).
%
% Inputs:
%   estState        previous estimator state (time step k-1)
%                   May be defined by the user (for example as a struct).
%   actuate         control input u(k-1), [1x2]-vector
%                   actuate(1): u_v(k-1), drive wheel angular velocity
%                   actuate(2): u_r(k-1), drive wheel angle
%   sense           sensor measurements z(k), [1x2]-vector, INF if no
%                   measurement
%                   sense(1): z_d(k), distance measurement
%                   sense(2): z_r(k), orientation measurement
%   tm              time, scalar
%                   If tm==0 initialization, otherwise estimator
%                   iteration step.
%   knownConst      known constants (from KnownConstants.m)
%   designPart      variable to distinguish the estimator design part
%                       designPart==1  -> Part 1
%                       designPart==2  -> Part 2
%
% Outputs:
%   posEst          position estimate (time step k), [1x2]-vector
%                   posEst(1): x position estimate
%                   posEst(2): y position estimate
%   oriEst          orientation estimate (time step k), scalar
%   radiusEst       estimate of wheel radius W (time step k), scalar
%   posVar          variance of position estimate (time step k), [1x2]-vector
%                   posVar(1): x position variance
%                   posVar(2): y position variance
%   oriVar          variance of orientation estimate (time step k), scalar
%   radiusVar       variance of wheel radius estimate (time step k), scalar
%   estState        current estimator state (time step k)
%                   Will be input to this function at the next call.

%% Mode 1: Initialization

if (tm == 0)
    % Do the initialization of your estimator here!
    
    % Replace the following:
    posEst = [0 0];
    oriEst = 0;
    posVar = [knownConst.TranslationStartBound.^2 /3 knownConst.TranslationStartBound.^2 / 3];
    oriVar = knownConst.RotationStartBound.^2 / 6;
    radiusEst = knownConst.NominalWheelRadius;
    radiusVar = knownConst.WheelRadiusError.^2 / 3;
    estState.P = diag([posVar,oriVar,radiusVar]);
    estState.X = [0;0;0;radiusEst];    
    return;
end


%% Mode 2: Estimator iteration.
% If we get this far tm is not equal to zero, and we are no longer
% initializing.  Run the estimator.

% Replace the following:
% W0 = knownConst.NominalWheelRadius;
Qv = knownConst.VelocityInputPSD;
Qr = knownConst.AngleInputPSD;
B = knownConst.WheelBase;
P = estState.P;
X0 = estState.X;
dt = 0.1;

X = X0 + [X0(4) * actuate(1) *cos(actuate(2)) * cos(X0(3)) * 0.1;X0(4) * actuate(1) *cos(actuate(2)) * sin(X0(3)) * 0.1;-X0(4) * actuate(1)*sin(actuate(2)) * 0.1/B; 0];
ds = sense(1) - sqrt(X(1)^2 + X(2)^2);
dr = sense(2) - X(3);

A = [   
0, 0, -X0(4) * actuate(1) *cos(actuate(2)) * sin(X0(3)), actuate(1) * cos(actuate(2)) * cos(X0(3));
0, 0, X0(4) * actuate(1) *cos(actuate(2)) * cos(X0(3)), actuate(1) * cos(actuate(2)) * sin(X0(3));
0, 0, 0 , -X0(4) * sin(actuate(2)) * actuate(1) / B;
0, 0, 0, 0;
];
A = [   
1, 0, -X0(4) * actuate(1) *cos(actuate(2)) * sin(X0(3)) * 0.1, actuate(1) * cos(actuate(2)) * cos(X0(3)) * 0.1;
0, 1, X0(4) * actuate(1) *cos(actuate(2)) * cos(X0(3)) * 0.1, actuate(1) * cos(actuate(2)) * sin(X0(3)) * 0.1;
0, 0, 1 , -X0(4) * sin(actuate(2)) * actuate(1) * 0.1 / B;
0, 0, 0, 1;
];
if designPart == 1
    T = eye(4);
    Q = diag([0.01,0.01,0.0002,0]);
else
    T = [
    X0(4) * actuate(1) * cos(actuate(2)) * cos(X0(3)), -X0(4) * actuate(1) * sin(actuate(2)) * cos(X0(3));
    X0(4) * actuate(1) * cos(actuate(2)) * sin(X0(3)), -X0(4) * actuate(1) * sin(actuate(2)) * sin(X0(3));
    -X0(4) * actuate(1) * sin(actuate(2))/B, -X0(4) * actuate(1)*cos(actuate(2)) * 0.1/B;
    0, 0;
    ];
    Q = diag([Qv, Qr]);
end

% P = P + (A * P + P * A' + T * Q * T').* dt;
P = (A * P * A' + T * Q * T');

if sense(1) ~= inf && sense(2) ~= inf
    H = [
    X(1)/sqrt(X(1).^2 + X(2).^2), X(2)/sqrt(X(1).^2 + X(2).^2), 0, 0;
    0, 0, 1, 0;
    ];
    R = [knownConst.DistNoise^2 / 6, 0; 0, knownConst.CompassNoise^2;];
    K = P * H' /(H  * P * H' + R);        
    X = X + K * [ds;dr];
    P = (eye(4) - K * H) * P;
elseif sense(1) ~= inf
    H = [
    X(1)/sqrt(X(1).^2 + X(2).^2), X(2)/sqrt(X(1).^2 + X(2).^2), 0, 0;
    ];
    R = knownConst.DistNoise^2 / 6;
    K = P * H' /(H  * P * H' + R);        
    X = X + K * ds;
    P = (eye(4) - K * H) * P;
elseif sense(2) ~= inf
    H = [
    0, 0, 1, 0;
    ];
    R = knownConst.CompassNoise^2;
    K = P * H' /(H  * P * H' + R);        
    X = X + K * dr;
    P = (eye(4) - K * H) * P;
end

if tm < 10
%     X
%     actuate
    sense
end

posEst = [X(1) X(2)];
oriEst = X(3);
posVar = [P(1,1) P(2,2)];
oriVar = P(3,3);
radiusEst = X(4);
radiusVar = P(4,4);

estState.P = P;
estState.X = X;
end