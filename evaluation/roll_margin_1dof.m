%% ロール制御の安定余裕評価（1DOF）
% ロール軸の運動方程式を手で立てて、開ループ伝達関数から余裕を求める。
% Simulink モデルを線形化するのではなく、式が見える形で追えるようにしている。
%
% 前提: run_main.m でパラメータが定義済みであること。

%% 1) ロール軸の運動方程式
% 機体をロール軸まわりの 1 自由度とみなすと
%
%     Ixx * dp/dt = Lp * p + Ldelta * delta
%     dphi/dt     = p
%
%   p      : ロール角速度 [rad/s]
%   phi    : ロール角     [rad]
%   delta  : フィン舵角   [rad]
%   Lp     : ロール減衰   [Nm/(rad/s)]  ← 負。回ると止めようとする
%   Ldelta : 舵効き       [Nm/rad]      ← 正。舵を切ると回る
%
% Lp と Ldelta は動圧に比例するので、設計点 roll_design_qbar での値を使う。
% 6DOF 側は動圧正規化が入っているため、設計点での設計がそのまま成立する。

Ixx    = roll_design_Ixx;
Lp     = roll_Lp;
Ldelta = roll_Ldelta;

s = tf("s");

% 舵角 -> ロールレート
P_p   = Ldelta / (Ixx * s - Lp);

% ロールレート -> ロール角（純粋な積分）
P_phi = 1 / s;

fprintf("--- ロール軸の素の特性 ---\n");
fprintf("  時定数   Ixx/|Lp|   = %.2f s\n", Ixx / abs(Lp));
fprintf("  定常ゲイン Ldelta/|Lp| = %.1f (rad/s)/rad = %.2f (deg/s)/deg\n", ...
    Ldelta / abs(Lp), Ldelta / abs(Lp));

%% 2) アクチュエータとセンサ
% どちらも一次遅れ。位相を食うので余裕を決める主因になる。
A     = 1 / (fin_actuatorTau   * s + 1);   % フィンアクチュエータ
G_p   = 1 / (sensor_gyroTau    * s + 1);   % レートジャイロ
G_phi = 1 / (sensor_attitudeTau * s + 1);  % 姿勢センサ

%% 3) 制御則
% 内側: ロールレートの PI 制御（Kd = 0）
% 外側: ロール角の P 制御
C_in  = controller_inner_Kp + controller_inner_Ki / s;
K_out = controller_outer_Kp;

%% 4) 開ループ伝達関数
% --- 4a) フィン指令点で開いたループ ---
% delta_cmd から一巡して delta_cmd に戻る経路をたどると
%
%   delta   = A * delta_cmd
%   p       = P_p * delta,      phi = P_phi * p
%   pHat    = G_p * p,          phiHat = G_phi * phi
%   pCmd    = K_out * (phiCmd - phiHat)
%   delta_cmd = C_in * (pCmd - pHat)
%
% phiCmd = 0 として戻り分を集めると
%
%   L_fin = C_in * A * P_p * ( K_out * G_phi * P_phi + G_p )
L_fin = C_in * A * P_p * (K_out * G_phi * P_phi + G_p);

% --- 4b) レート指令点で開いたループ（内側は閉じたまま）---
% 内側閉ループ  p / pCmd = C_in*A*P_p / (1 + C_in*A*P_p*G_p)
T_in  = feedback(C_in * A * P_p, G_p);
L_out = K_out * G_phi * P_phi * T_in;

%% 5) 安定余裕
loopNames = ["フィン指令ループ", "外側ロールループ"];
loops     = {L_fin, L_out};

fprintf("\n--- 安定余裕 ---\n");
fprintf("  %-18s %8s %9s %11s\n", "ループ", "GM[dB]", "PM[deg]", "wc[rad/s]");
for k = 1:numel(loops)
    [gm, pm, ~, wc] = margin(loops{k});
    fprintf("  %-18s %8.2f %9.2f %11.3f\n", loopNames(k), 20*log10(gm), pm, wc);
end

%% 6) 閉ループ応答
T_phi = feedback(K_out * P_phi * T_in, G_phi);   % phiCmd -> phi
fprintf("\n--- 閉ループ（phiCmd -> phi）---\n");
info = stepinfo(T_phi);
fprintf("  立上り時間 %.2f s / 整定時間 %.2f s / オーバーシュート %.1f %%\n", ...
    info.RiseTime, info.SettlingTime, info.Overshoot);
fprintf("  帯域幅 %.2f rad/s\n", bandwidth(T_phi));

%% 7) 図
figure("Name", "1DOF Roll Stability Margin", "Color", "w", "Position", [120 120 1000 720]);
tiledlayout(2, 2, "TileSpacing", "compact", "Padding", "compact");

nexttile;
margin(L_fin); grid on; title("フィン指令ループ");

nexttile;
margin(L_out); grid on; title("外側ロールループ");

nexttile;
nichols(L_fin, L_out); grid on;
legend(loopNames, "Location", "best"); title("ニコルス線図");

nexttile;
step(T_phi, 5); grid on; title("ロール角のステップ応答");

%% 8) ゲインを変えて余裕がどう動くか
% 教材用: 内側ゲインを振って位相余裕の変化を見る。
fprintf("\n--- 内側ゲインを振ったときのフィン指令ループ ---\n");
fprintf("  %-10s %8s %9s %11s\n", "inner_Kp", "GM[dB]", "PM[deg]", "wc[rad/s]");
for kp = controller_inner_Kp * [0.5 1 2 4]
    Ck = kp + controller_inner_Ki / s;
    Lk = Ck * A * P_p * (K_out * G_phi * P_phi + G_p);
    [gm, pm, ~, wc] = margin(Lk);
    fprintf("  %-10.3f %8.2f %9.2f %11.3f\n", kp, 20*log10(gm), pm, wc);
end
