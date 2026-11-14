dt = 0.1;
t = 0:dt:40;
v = zeros(size(t));

v(t<10) = 0.25;
v(t>=10 & t<30) = 0;
v(t>=30) = -0.25;

h = zeros(size(t));
for k  = 2:numel(t)
    h(k) = h(k-1) + v(k-1)*dt
end

plot(t, h, 'lineWidth', 1.5)
xlabel('Time (s)')
ylabel('Height (m)')
title('True Flight')
grid on

%%
rng(1)                      % same random numbers every run, so runs compare
sigma_baro = 0.3;           % barometer noise (m)
h_baro = h + sigma_baro*randn(size(h));   % measured height

a_true = [v(1), diff(v)]/dt;  % true acceleration each step
% (first entry: spin-up from rest to climb speed)
sigma_acc = 0.5;            % accelerometer noise (m/s^2)
a_meas = a_true + sigma_acc*randn(size(a_true));

% easy answer 1: use the barometer reading as your height (that is h_baro)

% easy answer 2: integrate the accelerometer twice
v_acc = zeros(size(t)); 
h_acc = zeros(size(t));
for k = 2:numel(t)
    v_acc(k) = v_acc(k-1) + a_meas(k-1)*dt;
    h_acc(k) = h_acc(k-1) + v_acc(k-1)*dt;
end

plot(t, h, 'LineWidth', 1.5); hold on
plot(t, h_baro, '.')
plot(t, h_acc, 'LineWidth', 1)
legend('Truth', 'Baro Only', 'Accel Only')
xlabel('Time (s)'), ylabel('Height (m)')

% step3_filter.m - keep your step 1 and 2 code above (t, dt, h, h_baro, a_meas, h_acc)
F = [1 dt; 0 1];        % constant-velocity model
B = [0; dt];            % accelerometer input adds a*dt to velocity
H = [1 0];              % barometer measures height only
Q = [0.01 0; 0 0.01];   % model distrust, start here (step 4 plays with it)
R = sigma_baro^2;       % barometer noise variance, from step 2

%Q = [0.0001 0; 0 0.0001];
%R = 100;

%Q = [100 0; 0 100];
%R = 0.0001;

x = [5; 0];             % state: [height; velocity], starts on the ground
P = 0.0001*eye(2);             % state uncertainty
h_est = zeros(size(t));

%h_baro(201:250) = NaN;

for k = 1:numel(t)
    % predict
    x = F*x + B*a_meas(k);
    P = F*P*F' + Q;

    % correct with the barometer
  %  y = h_baro(k) - H*x;   % measurement minus predicted measurement
   % S = H*P*H' + R;         % innovation uncertainty
   % K = P*H'/S;               % Kalman gain
   % x = x + K*y;           % corrected state
   % P = (eye(2) - K*H)*P;

    if ~isnan(h_baro(k))          % only correct when a reading actually arrived
        y = h_baro(k) - H*x;
        S = H*P*H' + R;
        K = P*H' / S;
        x = x + K*y;
        P = (eye(2) - K*H)*P;
    end

    h_est(k) = x(1);

end

plot(t, h, 'LineWidth', 1.5); hold on
plot(t, h_baro, '.')
plot(t, h_est, 'LineWidth', 1.5)
legend('Truth', 'Baro only', 'Kalman')
xlabel('Time (s)'), ylabel('Height (m)')

% the number that proves it
rmse_baro = sqrt(mean((h_baro - h).^2));
rmse_acc  = sqrt(mean((h_acc  - h).^2));
rmse_kf   = sqrt(mean((h_est  - h).^2));

figure;
plot(t,h,'LineWidth',1.5); hold on;
plot(t,h_baro,'.');
plot(t,h_est,'LineWidth',1.5);
legend('Truth','Baro only','Kalman');
xlabel('Time (s)'); ylabel('Height (m)');
