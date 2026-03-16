%% ============================================================
% PHASE 1 — VECV BMS CAN Log Replay
% Reads a BusMaster .asc log, decodes via VECV_BMS.dbc,
% runs the EKF+CC hybrid, plots results.
%
% Hardware confirmed from log header:
%   Kvaser USBcan Pro 2xHS v2, Channel 1, 500 kbps, BusMaster ASC
%
% Requirements: MATLAB Vehicle Network Toolbox (R2021a+)
%% ============================================================

clear; clc; close all;

%% ============================================================
% CONFIG
%% ============================================================

LOG_FILE = 'CAN3_S0_1.asc';      % your BusMaster .asc log filename
DBC_FILE = 'VECV_BMS_V2.dbc';    % your DBC filename

% ── PATH FIX ─────────────────────────────────────────────────
% MATLAB looks for files relative to the Current Folder (shown in toolbar).
% Either:
%   (a) cd to the folder in the Command Window:
%         cd('C:\VECV\logs')
%   (b) set absolute paths here:
%         LOG_FILE = 'C:\VECV\logs\CAN3_S0_1.asc';
%         DBC_FILE = 'C:\VECV\VECV_BMS_V2.dbc';
%   (c) use the file picker — uncomment the 4 lines below:
%
% [fn,fd] = uigetfile('*.asc','Select BusMaster log'); LOG_FILE = fullfile(fd,fn);
% [fn,fd] = uigetfile('*.dbc','Select DBC');           DBC_FILE = fullfile(fd,fn);
%
% Quick check before running:
%   exist(LOG_FILE,'file')   % must return 2
%   exist(DBC_FILE,'file')   % must return 2
% ─────────────────────────────────────────────────────────────

%% ============================================================
% VECV BMS SIGNAL REFERENCE (confirmed from VECV_BMS_CANDBC.xml)
%
%  Signal                  Message            CAN_ID       bitpos  bits  factor  offset  unit
%  BMS_PackCurrent         BMS_V_I_RISO       0x18FF04F4   16      12    1       0       A  Signed -500..+500
%  BMS_PackVoltage         BMS_V_I_RISO       0x18FF04F4   32      10    1       0       V
%  BMS_SOC                 BMS_Err_SOC        0x18FF00F4   56       7    1       0       %  integer 0-100
%  BMS_MaxCellTemp         BMS_T              0x18FF03F4    8       8    1     -40       C
%  BMS_MinCellTemp         BMS_T              0x18FF03F4   16       8    1     -40       C
%  BMS_PackMaxCellVoltage  BMS_Vcell          0x18FF02F4   16      12    0.001   0       V
%  BMS_PackMinCellVoltage  BMS_Vcell          0x18FF02F4   40      12    0.001   0       V
%  Battery_SOH             BMS_States         0x18FF01F4   40       8    0.5     0       %
%  BMS_PackState           BMS_States         0x18FF01F4   12       4    1       0       enum
%  AverageCellVoltage      Battery_AvgCell..  0x18FF09F4    0      16    0.001   0       V
%  AverageCellTemperature  Battery_AvgCell..  0x18FF09F4   16       8    1     -40       C
%  BMS_RealSOC             BMS_Coolant_T      0x18FF0CF4   56       8    1       0       %
%
%  All frames: CAN Extended 29-bit.
%  Factor + offset applied automatically by canSignalImport via DBC.
%% ============================================================

%% ============================================================
% LOAD DBC
%% ============================================================

% DBC path is kept for reference only — decoder uses hardcoded signal definitions.
% Only the log file must be present.
assert(exist(LOG_FILE,'file')==2, ...
    'Log not found: %s\nRun: cd(''folder'') or set absolute path above.', LOG_FILE);

% DBC is embedded in busmaster_asc_decoder.m — no canDatabase() call needed.
% (canDatabase is only required when using canSignalImport, which we bypass)

%% ============================================================
% DECODE BUSMASTER .ASC LOG
% MATLAB's canMessageImport does NOT support BusMaster .asc format
% (accepts only 'Vector' .asc/.blf or 'Kvaser' .txt — not BusMaster).
% busmaster_asc_decoder() reads the file directly and decodes all
% VECV BMS signals from the raw hex bytes using the DBC parameters.
% Make sure busmaster_asc_decoder.m is in the same folder or on path.
%% ============================================================

decoded = busmaster_asc_decoder(LOG_FILE);


%% ============================================================
% RESAMPLE TO UNIFORM 10 Hz GRID
%% ============================================================

% Build time grid from the decoded current signal (most frequent message)
t_start = decoded.BMS_PackCurrent.Time(1);
t_end   = decoded.BMS_PackCurrent.Time(end);
dt      = 0.1;
time_abs = (t_start:dt:t_end)';   % absolute timestamps for interp1
time     = time_abs - t_start;     % zero-based for plotting
N       = length(time);

rsmp = @(name) interp1( ...
    decoded.(name).Time, decoded.(name).Data, ...
    time_abs, 'previous', 'extrap');  % use absolute time to match decoded data

% Resample each signal to uniform 10 Hz grid.
% fillmissing forward-fills NaN gaps (BMS messages arrive at different rates).
ffill   = @(v) fillmissing(v, 'previous');
I       = ffill(rsmp('BMS_PackCurrent'));       % [A]  negative = discharge
Vt      = ffill(rsmp('BMS_PackVoltage'));       % [V]
SOC_bms = ffill(rsmp('BMS_SOC')) / 100;        % [0-1]
T_max   = ffill(rsmp('BMS_MaxCellTemp'));       % [C]
T_min   = ffill(rsmp('BMS_MinCellTemp'));       % [C]
T       = (T_max + T_min) / 2;
Vc_max  = ffill(rsmp('BMS_PackMaxCellVoltage')); % [V]
Vc_min  = ffill(rsmp('BMS_PackMinCellVoltage')); % [V]
SOH_bms = ffill(rsmp('Battery_SOH'));            % [%]

fprintf('\nData summary:\n');
fprintf('  Samples:      %d  at %.1f Hz  (%.1f min)\n', N, 1/dt, (t_end-t_start)/60);
fprintf('  SOC:          %.0f%% to %.0f%%\n', SOC_bms(1)*100, SOC_bms(end)*100);
fprintf('  Pack voltage: %.0f to %.0f V\n', Vt(1), Vt(end));
fprintf('  Temperature:  %.1f to %.1f C\n', T(1), T(end));
fprintf('  BMS SOH:      %.1f%%\n', mean(SOH_bms(~isnan(SOH_bms))));

%% ============================================================
% PACK CONFIGURATION — auto-detected from signals
%% ============================================================

% n_series: pack voltage / average cell voltage at rest
rest_mask = abs(I(1:min(200,N))) < 1.0;
try
    avg_cell = rsmp('AverageCellVoltage');
    if sum(rest_mask) > 5
        n_series = round(mean(Vt(rest_mask)) / mean(avg_cell(rest_mask)));
        src = 'AverageCellVoltage';
    else
        n_series = round(Vt(1) / avg_cell(1));
        src = 'AverageCellVoltage (t=0)';
    end
catch
    if sum(rest_mask) > 5
        vcell_avg = mean((Vc_max(rest_mask)+Vc_min(rest_mask))/2);
        n_series  = round(mean(Vt(rest_mask)) / vcell_avg);
    else
        n_series = 108;
    end
    src = 'Vcell max/min';
end
fprintf('  n_series:     %d  (from %s)\n', n_series, src);

% Current diagnostic — print before Q calculation to catch sign/scale issues
fprintf('  Current:      min=%.1f A  max=%.1f A  mean=%.1f A\n', ...
        min(I), max(I), mean(I,'omitnan'));
fprintf('  Sign check:   negative mean = discharge (expected for this bus)\n');

% Q_capacity: integrate signed current over full discharge
% Discharge current is NEGATIVE, so net_Ah = -sum(I)*dt/3600 is positive
net_Ah    = -sum(I,'omitnan') * dt / 3600;
delta_soc = SOC_bms(find(~isnan(SOC_bms),1,'first')) - ...
            SOC_bms(find(~isnan(SOC_bms),1,'last'));
delta_soc = max(delta_soc, 0.05);
Q_Ah      = net_Ah / delta_soc;

% Sanity check: Q must be physically plausible (50-200 Ah for this pack)
% If outside range the V2 DBC may use a different current factor
if isnan(Q_Ah) || Q_Ah < 50 || Q_Ah > 250
    warning(['Q_Ah=%.1f is outside plausible range (50-250 Ah).\n' ...
             'Check BMS_PackCurrent factor in VECV_BMS_V2.dbc.\n' ...
             'Using rated Q=90 Ah (nameplate).'], Q_Ah);
    Q_Ah = 89;
end
Q_cap = Q_Ah * 3600;
fprintf('  Q_capacity:   %.1f Ah  (SOH %.0f%% vs 90 Ah nameplate)\n', ...
        Q_Ah, Q_Ah/90*100);

%% ============================================================
% BATTERY MODEL PARAMETERS
% Use optimized values from ekf_cc_hybrid_v3 or fixed defaults.
%% ============================================================

R0 = 0.06;   % [Ohm]  DC internal resistance (pack level)
R1 = 0.04;   % [Ohm]  Diffusion resistance
C1 = 1200;   % [F]    tau = R1*C1 = 48 s

%% ============================================================
% 2D OCV TABLE (Table 3-2-1, scaled to detected n_series)
%% ============================================================

soc_bp  = (0:5:100)/100;
temp_bp = [0 10 25 45 60];
ocv_cell = [
    2.706,2.706,2.706,2.693,2.693; 3.160,3.160,3.160,3.143,3.143;
    3.200,3.200,3.200,3.202,3.202; 3.221,3.221,3.221,3.220,3.220;
    3.247,3.247,3.247,3.245,3.245; 3.263,3.263,3.263,3.261,3.261;
    3.280,3.280,3.280,3.279,3.279; 3.286,3.286,3.286,3.293,3.293;
    3.287,3.287,3.287,3.294,3.294; 3.288,3.288,3.288,3.294,3.294;
    3.289,3.289,3.289,3.295,3.295; 3.292,3.292,3.292,3.297,3.297;
    3.310,3.310,3.310,3.309,3.309; 3.327,3.327,3.327,3.329,3.329;
    3.327,3.327,3.327,3.330,3.330; 3.327,3.327,3.327,3.330,3.330;
    3.328,3.328,3.328,3.330,3.330; 3.328,3.328,3.328,3.331,3.331;
    3.328,3.328,3.328,3.331,3.331; 3.330,3.330,3.330,3.333,3.333;
    3.451,3.451,3.451,3.448,3.448;
];
ocv_pack    = ocv_cell * n_series;
dOCV_matrix = zeros(size(ocv_pack));
for c = 1:length(temp_bp)
    dOCV_matrix(:,c) = gradient(ocv_pack(:,c), soc_bp(2)-soc_bp(1));
end

%% ============================================================
% EKF + CC HYBRID LOOP
%% ============================================================

x         = [SOC_bms(1); 0];
P         = diag([1e-4, 1e-4]);
Qk        = diag([1e-7, 1e-4]);
CC_WEIGHT = 0.70;
soc_cc    = SOC_bms(1);
soc_est   = zeros(N,1);
soc_est(1) = x(1);

for k = 2:N
    u   = I(k);
    T_k = min(60, max(0, T(k)));
    sc  = min(soc_bp(end), max(soc_bp(1), x(1)));

    soc_cc   = max(0, min(1, soc_cc + (u*dt)/Q_cap));
    SOC_pred = max(0, min(1, x(1)  + (u*dt)/Q_cap));
    V1_pred  = x(2) + dt*(-x(2)/(R1*C1) + u/C1);
    x_pred   = [SOC_pred; V1_pred];
    A        = [1,0; 0,1-dt/(R1*C1)];
    P        = A*P*A' + Qk;

    OCV  = interp2(temp_bp, soc_bp, ocv_pack,    T_k, sc, 'linear');
    dOCV = interp2(temp_bp, soc_bp, dOCV_matrix, T_k, sc, 'linear');
    V_pred = OCV + u*R0 + x_pred(2);
    H      = [dOCV, 1];
    Rk_eff = 25*(abs(dOCV)<1) + 1*(abs(dOCV)>=1);
    S      = H*P*H' + Rk_eff;
    K      = P*H'/S;
    x      = x_pred + K*(Vt(k) - V_pred);
    x(1)   = CC_WEIGHT*soc_cc + (1-CC_WEIGHT)*x(1);
    x(1)   = max(0, min(1, x(1)));
    P      = (eye(2)-K*H)*P;
    soc_est(k) = x(1);
end

%% ============================================================
% METRICS
%% ============================================================

err  = SOC_bms - soc_est;
% Use nanmean/nanmax - NaN gaps in BMS SOC are excluded from metrics
RMSE = sqrt(mean(err.^2,'omitnan'))*100;
MAE  = mean(abs(err),'omitnan')*100;
MAX  = max(abs(err(~isnan(err))))*100;
fprintf('\n=== SOC Performance ===\n');
fprintf('  RMSE = %.2f%%  |  MAE = %.2f%%  |  MAX = %.2f%%\n', RMSE, MAE, MAX);

%% ============================================================
% PLOTS
%% ============================================================

figure('Name','VECV CAN Replay','Position',[80 300 1100 420]);
plot(time, SOC_bms*100, 'b-',  'LineWidth',2); hold on;
plot(time, soc_est*100, 'r--', 'LineWidth',2);
legend('BMS SOC','EKF+CC Hybrid','Location','northeast');
xlabel('Time from start (s)'); ylabel('SOC (%)');
title(sprintf('VECV BMS CAN Replay  |  RMSE=%.2f%%  MAE=%.2f%%  |  n=%dS  Q=%.0fAh', ...
              RMSE, MAE, n_series, Q_Ah));
grid on;

figure('Name','SOC Error','Position',[80 100 1100 260]);
plot(time, err*100, 'k', 'LineWidth',1.2);
yline(0,'b--','LineWidth',1.5);
yline( 3,'r:','LineWidth',1.5,'Label','+3%');
yline(-3,'r:','LineWidth',1.5,'Label','-3%');
xlabel('Time (s)'); ylabel('Error (%)');
title('BMS SOC - EKF SOC'); ylim([-8 8]); grid on;

fname = sprintf('replay_%s.mat', datestr(now,'yyyymmdd_HHMMSS'));
save(fname,'time','I','Vt','SOC_bms','soc_est','T','SOH_bms','n_series','Q_Ah','R0','R1','C1');
fprintf('Saved: %s\n', fname);
