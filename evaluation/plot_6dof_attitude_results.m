function plot_6dof_attitude_results(simOut, figureName)
%PLOT_6DOF_ATTITUDE_RESULTS  6DOF のオイラー角と機体角速度を描く
%
%   モデルは rad / rad/s で計算するため、この関数が deg / deg/s に換算して表示する。
%   期待するログ信号: euler [rad]、pqr [rad/s]（ともに 3 成分、サフィックス無し）
%
%   注意: 321 オイラー角は機首が水平に近いと phi と psi が縮退する。
%   UEN 基準では鉛直姿勢が pitch = 0 なので上昇中は安全だが、
%   機首が倒れる領域では phi の値を物理的なロールとみなさないこと。

if nargin < 2
    figureName = "6DOF Attitude Result";
end

euler = get_matrix(simOut, "euler");
pqr   = get_matrix(simOut, "pqr");

if isempty(euler) && isempty(pqr)
    warning("plot_6dof_attitude_results:NoData", ...
        "euler / pqr がログに見つかりません。");
    return
end

fig = figure("Name", figureName, "Color", "w", "Position", [150 150 1100 700]);
tiledlayout(fig, 2, 1, "TileSpacing", "compact", "Padding", "compact");

ax1 = nexttile;
hold(ax1, "on"); grid(ax1, "on");
plot_components(ax1, euler, ["\phi", "\theta", "\psi"]);
ylabel(ax1, "Euler [deg]");
title(ax1, figureName);
legend(ax1, "Location", "best");

ax2 = nexttile;
hold(ax2, "on"); grid(ax2, "on");
plot_components(ax2, pqr, ["p", "q", "r"]);
xlabel(ax2, "Time [s]");
ylabel(ax2, "Rate [deg/s]");
legend(ax2, "Location", "best");
end


function plot_components(ax, sig, names)
%PLOT_COMPONENTS  rad のベクトル信号を deg に換算して成分ごとに描く
if isempty(sig)
    text(ax, 0.5, 0.5, "no data", "Units", "normalized", ...
        "HorizontalAlignment", "center", "Color", [0.6 0.6 0.6]);
    return
end
for k = 1:min(numel(names), size(sig.data, 2))
    plot(ax, sig.time, rad2deg(sig.data(:, k)), ...
        "LineWidth", 1.2, "DisplayName", names(k));
end
end


function sig = get_matrix(simOut, signalName)
%GET_MATRIX  [nSample x nComponent] に整形して取り出す
sig = [];
try
    names = string(simOut.logsout.getElementNames);
catch
    return
end
if ~any(names == string(signalName))
    return
end
try
    ts = simOut.logsout.get(char(signalName)).Values;
catch
    return
end
data = squeeze(ts.Data);
if isvector(data)
    data = data(:);
elseif size(data, 1) ~= numel(ts.Time)
    data = data.';
end
sig = struct("time", ts.Time(:), "data", data);
end
