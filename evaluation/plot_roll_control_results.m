function plot_roll_control_results(simOut, figureName)
%PLOT_ROLL_CONTROL_RESULTS  ロール角・ロール角速度・フィン舵角の応答を描く
%
%   モデルは rad / rad/s で計算するため、この関数が deg / deg/s に換算して表示する。
%   期待するログ信号（サフィックス無し、すべて rad 系）:
%     phiCmd, delta, deltaCmd   [rad]
%     pCmd                      [rad/s]
%     euler                      [rad]   ロール角は第 1 成分
%     pqr                        [rad/s] ロール角速度は第 1 成分
%     1DOF モデルでは euler / pqr の代わりに phi / p が使われる。

if nargin < 2
    figureName = "Roll Control Result";
end

phi      = pick_angle(simOut, "phi",    "euler", 1);
phiCmd   = pick_angle(simOut, "phiCmd", "",      1);
p        = pick_angle(simOut, "p",      "pqr",   1);
pCmd     = pick_angle(simOut, "pCmd",   "",      1);
delta    = pick_angle(simOut, "delta",  "",      1);
deltaCmd = pick_angle(simOut, "deltaCmd", "",    1);

fig = figure("Name", figureName, "Color", "w", "Position", [120 120 1100 780]);
tiledlayout(fig, 3, 1, "TileSpacing", "compact", "Padding", "compact");

ax1 = nexttile;
hold(ax1, "on"); grid(ax1, "on");
plot_deg(ax1, phi,    "-",  1.4, "\phi");
plot_deg(ax1, phiCmd, "--", 1.2, "\phi cmd");
ylabel(ax1, "\phi [deg]");
title(ax1, figureName);
legend(ax1, "Location", "best");

ax2 = nexttile;
hold(ax2, "on"); grid(ax2, "on");
plot_deg(ax2, p,    "-",  1.4, "p");
plot_deg(ax2, pCmd, "--", 1.2, "p cmd");
ylabel(ax2, "p [deg/s]");
legend(ax2, "Location", "best");

ax3 = nexttile;
hold(ax3, "on"); grid(ax3, "on");
plot_deg(ax3, delta,    "-",  1.4, "\delta_f");
plot_deg(ax3, deltaCmd, "--", 1.2, "\delta cmd");
xlabel(ax3, "Time [s]");
ylabel(ax3, "\delta_f [deg]");
legend(ax3, "Location", "best");
end


function plot_deg(ax, sig, style, width, name)
%PLOT_DEG  rad のログを deg に換算して描く
if isempty(sig)
    return
end
plot(ax, sig.time, rad2deg(sig.data), style, "LineWidth", width, "DisplayName", name);
end


function sig = pick_angle(simOut, primaryName, vectorName, component)
%PICK_ANGLE  スカラー信号を優先し、無ければベクトル信号の指定成分を取り出す
sig = get_scalar(simOut, primaryName);
if ~isempty(sig) || strlength(vectorName) == 0
    return
end
sig = get_component(simOut, vectorName, component);
end


function sig = get_scalar(simOut, signalName)
sig = [];
ts = get_timeseries(simOut, signalName);
if isempty(ts)
    return
end
data = squeeze(ts.Data);
sig = struct("time", ts.Time(:), "data", data(:));
end


function sig = get_component(simOut, signalName, component)
sig = [];
ts = get_timeseries(simOut, signalName);
if isempty(ts)
    return
end
data = squeeze(ts.Data);
if isvector(data)
    column = data(:);
elseif size(data, 2) >= component
    column = data(:, component);
else
    column = data(component, :).';
end
sig = struct("time", ts.Time(:), "data", column);
end


function ts = get_timeseries(simOut, signalName)
ts = [];
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
end
end
