% Parameter Motor DC
Ra = 1;       % Armature Resistance (Ohm)
La = 0.5;     % Armature Inductance (H)
Km = 0.01;    % Motor Torque Constant (Nm/A)
Kb = 0.01;    % Back EMF Constant (V.s/rad)
J = 0.01;     % Moment of Inertia (kg.m^2)
B = 0.1;      % Damping Coefficient (N.m.s/rad)

% Fungsi alih motor DC dari input tegangan V(s) ke posisi sudut Theta(s)
% Model motor DC:
% (La*s + Ra)*I(s) + Kb*Omega(s) = V(s)
% J*s*Omega(s) + B*Omega(s) = Km*I(s)
% Omega = s*Theta

% Bentuk fungsi alih Omega(s)/V(s):
num_omega = Km;
den_omega = [J*La, J*Ra + B*La, B*Ra + Km*Kb];

G_omega = tf(num_omega, den_omega);

% Fungsi alih posisi Theta(s)/V(s) = Omega(s)/V(s) * 1/s
G_theta = G_omega / tf([1 0],1);

disp('Fungsi alih motor DC (Theta(s)/V(s)):');
G_theta

% Analisis Open-Loop
figure;
step(G_theta);
title('Respons Open-Loop Motor DC (Posisi Sudut)');
grid on;

% Hitung parameter waktu respon open-loop
S = stepinfo(G_theta);
fprintf('Open-Loop Rise Time: %.3f s\n', S.RiseTime);
fprintf('Open-Loop Overshoot: %.2f %%\n', S.Overshoot);
fprintf('Open-Loop Settling Time: %.3f s\n', S.SettlingTime);

% Steady-state error untuk step input (posisi)
% Karena sistem tipe 1 (ada integrator 1/s), error steady-state seharusnya 0
ess_open = abs(1 - dcgain(G_theta));
fprintf('Open-Loop Steady-State Error: %.5f\n', ess_open);

%% Rancang Kontroler PID untuk memenuhi spesifikasi
% Spesifikasi:
% Rise time <= 1.5 s
% Overshoot <= 10%
% Settling time <= 3 s
% Steady-state error = 0

% Gunakan PID dengan tuning manual atau metode Ziegler-Nichols sebagai awal
% Kemudian tuning parameter agar memenuhi spesifikasi

% Contoh tuning PID manual (dapat disesuaikan)
Kp = 100;
Ki = 200;
Kd = 10;

C = pid(Kp, Ki, Kd);

% Sistem tertutup
sys_cl = feedback(C*G_theta,1);

% Simulasi respons closed-loop
figure;
step(sys_cl);
title('Respons Closed-Loop dengan Kontroler PID');
grid on;

% Hitung parameter waktu respon closed-loop
S_cl = stepinfo(sys_cl);
fprintf('\nClosed-Loop Rise Time: %.3f s\n', S_cl.RiseTime);
fprintf('Closed-Loop Overshoot: %.2f %%\n', S_cl.Overshoot);
fprintf('Closed-Loop Settling Time: %.3f s\n', S_cl.SettlingTime);

% Steady-state error closed-loop
ess_cl = abs(1 - dcgain(sys_cl));
fprintf('Closed-Loop Steady-State Error: %.5f\n', ess_cl);

% Jika spesifikasi belum terpenuhi, lakukan tuning PID lebih lanjut
% Bisa menggunakan fungsi pidtune atau tuning manual

% Contoh tuning otomatis PID menggunakan pidtune
[C_auto, info] = pidtune(G_theta, 'PID', 1.5); % target bandwidth sesuai rise time

sys_cl_auto = feedback(C_auto*G_theta,1);

figure;
step(sys_cl_auto);
title('Respons Closed-Loop dengan PID Tuning Otomatis');
grid on;

S_auto = stepinfo(sys_cl_auto);
fprintf('\nClosed-Loop (PID Tuning Otomatis) Rise Time: %.3f s\n', S_auto.RiseTime);
fprintf('Closed-Loop (PID Tuning Otomatis) Overshoot: %.2f %%\n', S_auto.Overshoot);
fprintf('Closed-Loop (PID Tuning Otomatis) Settling Time: %.3f s\n', S_auto.SettlingTime);

ess_auto = abs(1 - dcgain(sys_cl_auto));
fprintf('Closed-Loop (PID Tuning Otomatis) Steady-State Error: %.5f\n', ess_auto);
