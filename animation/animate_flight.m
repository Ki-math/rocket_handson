function animate_flight(data, varargin)
%ANIMATE_FLIGHT  6DOF ロケット飛行アニメーション（UEN 基準・座標系を分離表示）
%
%   ANIMATE_FLIGHT(SIMOUT) は Simulink の出力 SIMOUT に記録された飛行を
%   アニメーション表示する。ANIMATE_FLIGHT(LOGSTRUCT) は同名フィールドを
%   持つ struct も受け付ける。
%
%   前提とする座標系
%     慣性基準 : UEN（x = Up, y = East, z = North）。鉛直姿勢が theta = 0 に
%                なるため、鉛直上昇中に 321 オイラー角の特異点に近づかない。
%                特異点は「機首が水平」の位置に移る。
%     機体軸   : 姿勢ゼロで xb = U, yb = E, zb = N。
%     表示     : MATLAB の 3D 軸は Z 上向きが自然なので、内部で ENU に
%                並べ替えて描画する。
%
%   必要なログ
%     positionUEN : 6DOF ブロックの Xe 出力（U, E, N 順）
%     quaternionBI  : 6DOF (Quaternion) ブロックの q 出力
%                     （[q0 q1 q2 q3]、quat2dcm(q) = DCM_body_from_UEN）
%   任意のログ
%     velocityBody, pqr, euler, delta,
%     phiCmd, pCmd, deltaCmd   （すべて rad / rad/s）
%
%   ANIMATE_FLIGHT(..., NAME, VALUE) のオプション
%     PlaybackSpeed (8)        実時間に対する再生倍率
%     FrameRate     (30)       再生フレームレート [fps]。ログはこの間隔に
%                              一様リサンプルされるため、モデルの刻み幅や
%                              ログ間引きに関係なくコマ間隔が一定になる
%     BodyLength    (2.8)      機体長 [m]
%     CgFraction    (0.5)      positionUEN が機体のどこか（0 = 尾部, 1 = 機首）
%     WorldScale    ([])       鳥観図での機体の強調倍率（[] = 自動）
%     Trail         (true)     軌跡の表示 ON/OFF
%     Layout        ("full")   "full"      : 鳥観図 + 座標系パネル + 時間履歴
%                              "worldonly" : 鳥観図のみ
%     WorldView     ([40 18])  鳥観図の [方位 仰角] deg
%     FrameView     ([42 20])  座標系パネルの [方位 仰角] deg
%     ShowTrends    (true)     時間履歴パネルを表示
%     SaveVideo     (false)
%     VideoFile     ("flight_animation.mp4")
%
%   画面構成（Layout = "full"）
%     左列   : UEN 鳥観図。軌跡・地面・落下線・地面投影影・固定 ENU 三軸
%     右上   : 座標系パネル。慣性軸（灰）と機体軸 Xb/Yb/Zb（赤緑青）、数値表示
%     右下   : roll / p / delta_f の時間履歴と再生カーソル
%
%   ロール角について
%     表示する roll は四元数の swing-twist 分解で求めた、初期姿勢基準の
%     機首軸まわりのねじれ角。321 オイラー角の phi と違い特異点を持たない。
%
%   例
%     out = sim("rocket_6dof_with_roll_controller");
%     animate_flight(out, "BodyLength", 2.8, "PlaybackSpeed", 5);
%     animate_flight(out, "Layout", "worldonly");

opts = parse_inputs(varargin{:});
[logs, logsFull] = prepare_logs(data, opts);
ui   = build_ui(logs, logsFull, opts);
run_animation(ui, logs, opts);
end


%% ======================================================================
%  Input parsing
%  ======================================================================
function opts = parse_inputs(varargin)
isPosScalar = @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0;
isFlag      = @(x) (islogical(x) || isnumeric(x)) && isscalar(x);
isPair      = @(x) isnumeric(x) && numel(x) == 2;

parser = inputParser;
parser.FunctionName = "animate_flight";
parser.PartialMatching = false;   % 省略名を許さず、名前の取り違えを明示的なエラーにする
parser.addParameter("PlaybackSpeed", 8,        isPosScalar);
parser.addParameter("FrameRate",     30,       isPosScalar);
parser.addParameter("BodyLength",    2.8,      isPosScalar);
parser.addParameter("CgFraction",    0.5,      @(x) isnumeric(x) && isscalar(x) && x >= 0 && x <= 1);
parser.addParameter("ChuteDiameter", 2.0,      isPosScalar);
parser.addParameter("ChuteRiser",    3.0,      isPosScalar);   % 取付点から傘までの距離 [m]
parser.addParameter("WorldScale",    [],       @(x) isempty(x) || isPosScalar(x));
parser.addParameter("Trail",         true,     isFlag);
parser.addParameter("Layout",        "full",   @(x) any(strcmpi(x, ["full", "worldonly"])));
parser.addParameter("WorldView",     [40 18],  isPair);
parser.addParameter("FrameView",     [42 20],  isPair);
parser.addParameter("ShowTrends",    true,     isFlag);
parser.addParameter("SaveVideo",     false,    isFlag);
parser.addParameter("VideoFile",     "flight_animation.mp4", @(x) isstring(x) || ischar(x));
parser.parse(varargin{:});

opts = parser.Results;
opts.Layout     = string(lower(opts.Layout));
opts.VideoFile  = string(opts.VideoFile);
opts.Trail      = logical(opts.Trail);
opts.ShowTrends = logical(opts.ShowTrends);
opts.SaveVideo  = logical(opts.SaveVideo);
end


%% ======================================================================
%  Log preparation
%  ======================================================================
function [logs, logsFull] = prepare_logs(data, opts)
logs = extract_logs(data);

if isempty(logs.t) || isempty(logs.positionUEN) || isempty(logs.quaternionBI)
    error("animate_flight:MissingLogs", ...
        "positionUEN と quaternionBI が必要です（logsout に含まれているか確認してください）。");
end

% アニメーションは間引いた logs で回すが、時間履歴は元の分解能で描きたいので
% 間引き前の logsFull も作って返す。
logsFull = derive_all(logs);
logs     = derive_all(resample_logs(logs, opts));
end


function logs = derive_all(logs)
%DERIVE_ALL  表示に使う派生量をまとめて作る
logs = derive_display_position(logs);
logs = derive_display_units(logs);
logs = derive_extras(logs);
logs = derive_attitude(logs);
end

function logs = extract_logs(data)
scalarNames = ["phiCmd", "pCmd", "deltaCmd", "delta", "qbar", "Mach", "chuteOpen"];
vectorNames = ["positionUEN", "velocityBody", "quaternionBI", "euler", "pqr"];

logs = struct("t", []);
for name = [vectorNames, scalarNames]
    logs.(name) = [];
end

if isstruct(data)
    logs = merge_structs(logs, data);
    if isempty(logs.t) && ~isempty(logs.positionUEN)
        logs.t = 0:size(logs.positionUEN, 2) - 1;
    end
    return
end

% 有効化サブシステム内の信号は、有効になるまでの区間が記録されないため
% 他の信号より点数が少なくなる。基準の時間軸へ揃えてから使う。
for name = vectorNames
    sig = get_logged_signal(data, name);
    if isempty(sig)
        continue
    end
    logs.(name) = to_columns(sig.Data);
    if isempty(logs.t)
        logs.t = sig.Time(:).';
    end
    logs.(name) = align_to_reference_time(logs.(name), sig.Time(:).', logs.t);
end

for name = scalarNames
    sig = get_logged_signal(data, name);
    if ~isempty(sig)
        logs.(name) = align_to_reference_time(sig.Data(:).', sig.Time(:).', logs.t);
    end
end

% 成分ごとの別名を用意しておくと下流の参照が素直になる
logs = split_triplet(logs, "euler", ["phi", "theta", "psi"]);
logs = split_triplet(logs, "pqr",   ["p", "q", "r"]);
end

function value = align_to_reference_time(value, tSource, tRef)
%ALIGN_TO_REFERENCE_TIME  信号を基準の時間軸に張り直す
%   記録区間が短い信号は端の値で外挿する（有効化前は最初の値で埋まる）。
if isempty(value) || isempty(tRef) || numel(tSource) == numel(tRef)
    return
end
if numel(tSource) < 2
    value = repmat(value(:, 1), 1, numel(tRef));
    return
end
value = interp1(tSource, value.', tRef, "linear", "extrap").';
end


function sig = get_logged_signal(out, name)
sig = [];
try
    sig = out.logsout.get(name).Values;
    return
catch
end
try
    sig = out.(name);
catch
end
end

function logs = split_triplet(logs, sourceName, targetNames)
source = logs.(sourceName);
for k = 1:numel(targetNames)
    if size(source, 1) >= k
        logs.(targetNames(k)) = source(k, :);
    else
        logs.(targetNames(k)) = [];
    end
end
end

function out = to_columns(data)
% 各時刻を列に並べた [nComponent x nSample] に整形する
data = squeeze(data);
if isvector(data)
    out = data(:).';
elseif size(data, 1) >= size(data, 2)
    out = data.';
else
    out = data;
end
end

function logs = resample_logs(logs, opts)
%RESAMPLE_LOGS  再生用に一様な時間格子へ張り直す
%   コマ間隔をモデルの刻み幅やログ間引きから切り離すことで、再生の
%   なめらかさが FrameRate だけで決まるようにする。
tSource  = logs.t;
duration = tSource(end) - tSource(1);
if numel(tSource) < 2 || duration <= 0
    return
end

nFrame = max(2, round(duration / opts.PlaybackSpeed * opts.FrameRate));
tFrame = linspace(tSource(1), tSource(end), nFrame);

% 四元数は補間前に符号を揃える。二重被覆のまま線形補間すると
% 符号が反転した箇所で姿勢が飛ぶ。
logs.quaternionBI = align_quaternion_signs(logs.quaternionBI);

for name = string(fieldnames(logs)).'
    value = logs.(name);
    if isempty(value) || name == "t" || size(value, 2) ~= numel(tSource)
        continue
    end
    logs.(name) = interp1(tSource, value.', tFrame).';
end

logs.t = tFrame;
logs.quaternionBI = logs.quaternionBI ./ vecnorm(logs.quaternionBI, 2, 1);
end

function q = align_quaternion_signs(q)
for k = 2:size(q, 2)
    if dot(q(:, k), q(:, k - 1)) < 0
        q(:, k) = -q(:, k);
    end
end
end

function logs = derive_display_position(logs)
% ログは UEN。MATLAB の 3D 軸は Z 上向きが自然なので ENU に並べ替えて保持する
logs.position_m = enu_from_uen() * logs.positionUEN;
end

function logs = derive_display_units(logs)
%DERIVE_DISPLAY_UNITS  モデルは rad / rad/s で計算するので、表示用に deg へ換算する
radToDeg = ["phi", "theta", "psi", "phiCmd", "delta", "deltaCmd"];
degName  = ["phi_deg", "theta_deg", "psi_deg", "phiCmd_deg", "delta_deg", "deltaCmd_deg"];
for k = 1:numel(radToDeg)
    if has_log(logs, radToDeg(k))
        logs.(degName(k)) = rad2deg(logs.(radToDeg(k)));
    else
        logs.(degName(k)) = [];
    end
end

rateToDps = ["p", "q", "r", "pCmd"];
dpsName   = ["p_dps", "q_dps", "r_dps", "pCmd_dps"];
for k = 1:numel(rateToDps)
    if has_log(logs, rateToDps(k))
        logs.(dpsName(k)) = rad2deg(logs.(rateToDps(k)));
    else
        logs.(dpsName(k)) = [];
    end
end
end

function logs = derive_extras(logs)
logs.altitude_m = logs.position_m(3, :);
if ~isempty(logs.velocityBody)
    logs.speed_mps = vecnorm(logs.velocityBody, 2, 1);
else
    logs.speed_mps = [];
end
end

function logs = derive_attitude(logs)
%DERIVE_ATTITUDE  表示用の姿勢量をすべて四元数から求める
%   euler_deg には頼らない。321 オイラー角は特異点近傍で phi と psi が
%   縮退するため、そのまま表示すると実際の運動を誤って伝える。
nSample = numel(logs.t);
qRef    = normalize_quaternion(logs.quaternionBI(:, 1));

logs.rollNose_deg = zeros(1, nSample);
logs.swing_deg    = zeros(1, nSample);
logs.noseElev_deg = zeros(1, nSample);
logs.noseAzim_deg = zeros(1, nSample);

for k = 1:nSample
    q = normalize_quaternion(logs.quaternionBI(:, k));
    [logs.rollNose_deg(k), logs.swing_deg(k)] = roll_about_nose(q, qRef);

    % 機首方向は表示 ENU で見た Xb。仰角は鉛直でも特異点にならない
    nose = enu_from_body(q) * [1; 0; 0];
    logs.noseElev_deg(k) = asind(min(1, max(-1, nose(3))));
    logs.noseAzim_deg(k) = atan2d(nose(1), nose(2));   % 北基準・東正
end

logs.rollNose_deg = rad2deg(unwrap(deg2rad(logs.rollNose_deg)));
end

function out = merge_structs(defaults, data)
out   = defaults;
names = fieldnames(data);
for k = 1:numel(names)
    out.(names{k}) = data.(names{k});
end
end

function tf = has_log(logs, name)
tf = isfield(logs, name) && ~isempty(logs.(name));
end

function value = log_value(logs, name, k)
if has_log(logs, name)
    value = logs.(name)(k);
else
    value = nan;
end
end


%% ======================================================================
%  Frames and attitude
%  ======================================================================
function R = enu_from_uen()
%ENU_FROM_UEN  モデルの UEN から表示用 ENU への並べ替え: (u, e, n) -> (e, n, u)
R = [0 1 0; 0 0 1; 1 0 0];
end

function R = enu_from_body(q)
%ENU_FROM_BODY  機体軸ベクトルを表示用 ENU に写す回転行列
%   quat_to_rotm(q) は「UEN <- body」の能動回転行列を返す。6DOF (Quaternion)
%   ブロックの q 出力は quat2dcm(q) = DCM_body_from_UEN なので転置は不要。
%   機体軸（x 前方 / y 左 / z 上）とローカルメッシュの軸は UEN 基準では
%   一致するため、メッシュ用の追加変換も要らない。
R = enu_from_uen() * quat_to_rotm(q);
end

function R = quat_to_rotm(q)
q = normalize_quaternion(q);
w = q(1); x = q(2); y = q(3); z = q(4);
R = [1 - 2*(y^2 + z^2), 2*(x*y - z*w),     2*(x*z + y*w);
     2*(x*y + z*w),     1 - 2*(x^2 + z^2), 2*(y*z - x*w);
     2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x^2 + y^2)];
end

function [rollDeg, swingDeg] = roll_about_nose(q, qRef)
%ROLL_ABOUT_NOSE  基準姿勢からの機首軸まわりロールを swing-twist 分解で求める
%   321 オイラー角の phi と違い、機首が基準から 180 deg 離れない限り一意で
%   特異点を持たない。ROLLDEG がねじれ成分、SWINGDEG が機首方向のずれ。
%   （Simulink の MATLAB Function ブロックで同じ計算を使う場合は、
%     codegen 対応版の quat_roll_about_nose.m を参照）
qErr = quaternion_product(quaternion_conjugate(qRef), q);
if qErr(1) < 0
    qErr = -qErr;   % 二重被覆の短い側を選ぶ
end
rollDeg  = 2 * atan2d(qErr(2), qErr(1));
swingDeg = 2 * atan2d(hypot(qErr(3), qErr(4)), hypot(qErr(1), qErr(2)));
end

function q = normalize_quaternion(q)
q = reshape(double(q), 1, 4);
magnitude = norm(q);
if magnitude < eps
    q = [1 0 0 0];
else
    q = q / magnitude;
end
end

function q = quaternion_conjugate(q)
q = [q(1), -q(2), -q(3), -q(4)];
end

function q = quaternion_product(a, b)
q = [a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4), ...
     a(1)*b(2) + a(2)*b(1) + a(3)*b(4) - a(4)*b(3), ...
     a(1)*b(3) - a(2)*b(4) + a(3)*b(1) + a(4)*b(2), ...
     a(1)*b(4) + a(2)*b(3) - a(3)*b(2) + a(4)*b(1)];
end


%% ======================================================================
%  Figure / layout construction
%  ======================================================================
function ui = build_ui(logs, logsFull, opts)
ui.fig = create_figure();
worldLimits = world_axis_limits(logs.position_m, opts.BodyLength);
worldScale  = resolve_world_scale(worldLimits, opts);

[ui.axWorld, ui.axInfo, ui.axFrame, layout, trendSpan] = build_layout(ui.fig, opts);

ui.world = build_world_view(ui.axWorld, logs, opts, worldLimits, worldScale);
if isempty(ui.axFrame)
    ui.info  = struct([]);
    ui.frame = struct([]);
else
    ui.info  = build_info_panel(ui.axInfo);
    ui.frame = build_frame_panel(ui.axFrame, logs, opts);
end

if opts.ShowTrends
    ui.trends = build_trend_panel(layout, logsFull, trendSpan);
else
    ui.trends = struct([]);
end
end

function fig = create_figure()
screen = get(groot, "ScreenSize");
width  = min(1500, screen(3) - 80);
height = min(900,  screen(4) - 140);
fig = figure( ...
    "Name", "Rocket Flight Animation", ...
    "Color", "w", ...
    "NumberTitle", "off", ...
    "Position", [screen(1) + 40, screen(2) + 70, width, height]);

end


function [axWorld, axInfo, axFrame, layout, trendSpan] = build_layout(fig, opts)
%BUILD_LAYOUT  左列に鳥観図、右列に数値表示・座標系パネル・時間履歴を積む
nTrendRows = 3 * double(opts.ShowTrends);
nRows      = 3 + nTrendRows;
nCols      = 5;
nWorldCols = 2;
nInfoCols  = 1;
nRightCols = nCols - nWorldCols;

if opts.Layout == "worldonly"
    axInfo  = gobjects(0);
    axFrame = gobjects(0);
    if opts.ShowTrends
        layout    = tiledlayout(fig, nTrendRows, nCols, "TileSpacing", "compact", "Padding", "compact");
        axWorld   = nexttile(layout, [nTrendRows, 3]);
        trendSpan = [1 2];
    else
        layout    = tiledlayout(fig, 1, 1, "TileSpacing", "compact", "Padding", "compact");
        axWorld   = nexttile(layout);
        trendSpan = [1 1];
    end
    return
end

layout    = tiledlayout(fig, nRows, nCols, "TileSpacing", "compact", "Padding", "compact");
axWorld   = nexttile(layout, [nRows, nWorldCols]);
axInfo    = nexttile(layout, [3, nInfoCols]);
axFrame   = nexttile(layout, [3, nRightCols - nInfoCols]);
trendSpan = [1 nRightCols];
end


% --- 鳥観図 -------------------------------------------------------------
function world = build_world_view(ax, logs, opts, limits, worldScale)
prepare_3d_axis(ax, limits);
view(ax, opts.WorldView(1), opts.WorldView(2));
title(ax, "鳥観図（慣性系 UEN）");

triadLength = triad_length(limits, [0; 0; 0]);

world.ground = create_ground_plane(ax, mean(limits(1:2, :), 2), ...
    0.5 * max(diff(limits(1:2, :), 1, 2)));
world.enuTriad = create_triad(ax, triadLength, ["E", "N", "U"], reference_triad_colors(), 2.4);
update_triad(world.enuTriad, eye(3), [0; 0; 0], triadLength);

world.pathFull = plot3(ax, logs.position_m(1, :), logs.position_m(2, :), logs.position_m(3, :), ...
    "Color", [0.80 0.82 0.86], "LineWidth", 0.8);
world.trail    = plot3(ax, nan, nan, nan, "Color", [0.15 0.45 0.80], "LineWidth", 1.6);
world.shadow   = plot3(ax, nan, nan, nan, "Color", [0.55 0.55 0.55], "LineWidth", 0.8);
world.dropLine = plot3(ax, nan, nan, nan, ":", "Color", [0.45 0.45 0.45], "LineWidth", 0.8);
world.marker   = plot3(ax, nan, nan, nan, ".", "MarkerSize", 14, "Color", [0.85 0.20 0.20]);
set([world.trail, world.shadow], "Visible", on_off(opts.Trail));

% 強調倍率はここでジオメトリに焼き込む。姿勢更新側では掛けないこと
% （両方で掛けると倍率が二乗になる）。
world.rocket    = create_rocket_graphics(ax, worldScale * opts.BodyLength, opts.CgFraction, false);
world.chute     = create_chute_graphics(ax, opts.ChuteDiameter, opts.ChuteRiser, worldScale);
world.noseOffset = worldScale * opts.BodyLength * (1 - opts.CgFraction);
world.showTrail = opts.Trail;

% カメラは固定なので光源も一度置けばよい（毎フレーム更新は無駄な再計算）
light(ax, "Style", "infinite", "Position", [1 -1 2]);
end

% --- 座標系パネル（慣性軸と機体軸を分けて見せる） ------------------------
function frame = build_frame_panel(ax, logs, opts)
hold(ax, "on");
grid(ax, "off");
axis(ax, "off");
daspect(ax, [1 1 1]);   % vis3d は付けない（鳥観図と同じくはみ出しの原因になる）
set(ax, "XLim", [-1.5 1.5], "YLim", [-1.5 1.5], "ZLim", [-1.5 1.5], ...
    "Projection", "orthographic");
view(ax, opts.FrameView(1), opts.FrameView(2));
title(ax, "座標系（慣性軸 UEN / 機体軸 Xb Yb Zb）");

% 水平面リング（傾きの基準）
ring = linspace(0, 2 * pi, 80);
plot3(ax, cos(ring), sin(ring), zeros(size(ring)), "Color", [0.80 0.80 0.80], "LineWidth", 0.8);

% 初期姿勢の Yb / Zb を薄く固定表示 → 累積ロールが目で読める
R0 = enu_from_body(logs.quaternionBI(:, 1));
for k = 2:3
    tip = 1.05 * R0(:, k);
    plot3(ax, [0 tip(1)], [0 tip(2)], [0 tip(3)], "--", ...
        "Color", [0.72 0.74 0.78], "LineWidth", 1.2);
end

% 慣性軸は灰色の細線、機体軸は色付きの太線 → 一目で区別できる
frame.enuTriad = create_triad(ax, 1.25, ["E", "N", "U"], world_triad_colors(), 1.2);
update_triad(frame.enuTriad, eye(3), [0; 0; 0], 1.25);
frame.bodyTriad = create_triad(ax, 1.05, ["Xb", "Yb", "Zb"], body_triad_colors(), 3.0);

frame.rocket = create_rocket_graphics(ax, 1.1, 0.5, true);
frame.chute  = create_chute_graphics(ax, 0.9, 0.9, 1);
frame.noseOffset = 0.55;
light(ax, "Style", "infinite", "Position", [1 -1 1.5]);
end

% --- 数値表示（座標系パネルとは別タイル） --------------------------------
function info = build_info_panel(ax)
axis(ax, "off");
title(ax, "状態量");
info.readout = text(ax, 0, 0, "", ...
    "Units", "normalized", ...
    "Position", [0.04 0.96], ...
    "VerticalAlignment", "top", ...
    "HorizontalAlignment", "left", ...
    "FontName", "Consolas", ...
    "FontSize", 9);
end

% --- 時間履歴パネル -----------------------------------------------------
function trends = build_trend_panel(layout, logs, span)
spec = trend_specification();
trends = struct("ax", {}, "cursor", {});

for k = 1:numel(spec)
    ax = nexttile(layout, span);
    hold(ax, "on");
    grid(ax, "on");
    box(ax, "on");
    ylabel(ax, spec(k).label);

    names = string.empty;
    names = plot_trend_line(ax, logs, spec(k).field,    names, spec(k).legend(1), "-",  [0.10 0.35 0.70], 1.4);
    names = plot_trend_line(ax, logs, spec(k).cmdField, names, spec(k).legend(2), "--", [0.85 0.35 0.10], 1.0);

    if isempty(names)
        text(ax, 0.5, 0.5, "no data", "Units", "normalized", ...
            "HorizontalAlignment", "center", "Color", [0.6 0.6 0.6]);
    else
        legend(ax, names, "Location", "northeast", "Box", "off", "FontSize", 8, ...
            "AutoUpdate", "off");
    end

    % xline (ConstantLine) は値を変えるたびに軸全体の再描画と凡例の自動更新を
    % 誘発して重い。見た目は同じなので 2 点の線を XData だけ動かす。
    xlim(ax, [logs.t(1) logs.t(end)]);
    ylim(ax, ylim(ax));
    set(ax, "XLimMode", "manual", "YLimMode", "manual");
    cursor = plot(ax, [logs.t(1) logs.t(1)], ylim(ax), ...
        "Color", [0.80 0.20 0.20], "LineWidth", 1.0);
    cursor.Annotation.LegendInformation.IconDisplayStyle = "off";

    if k == numel(spec)
        xlabel(ax, "Time [s]");
    else
        set(ax, "XTickLabel", []);
    end

    trends(k).ax     = ax;
    trends(k).cursor = cursor;
end
end

function names = plot_trend_line(ax, logs, field, names, label, style, color, width)
if strlength(field) == 0 || ~has_log(logs, field)
    return
end
plot(ax, logs.t, logs.(field), style, "LineWidth", width, "Color", color);
names(end + 1) = label;
end

function spec = trend_specification()
% roll は四元数の swing-twist 由来の 1 本のみ。オイラー角の phi とは
% UEN 基準では一致するので、重ねても情報が増えない。
spec = struct( ...
    "field",    {"rollNose_deg", "p_dps",     "delta_deg"}, ...
    "cmdField", {"phiCmd_deg",   "pCmd_dps",  "deltaCmd_deg"}, ...
    "label",    {"roll [deg]",   "p [deg/s]", "\delta_f [deg]"}, ...
    "legend",   {["roll", "roll cmd"], ...
                 ["p",    "p cmd"], ...
                 ["\delta_f", "\delta cmd"]});
end


%% ======================================================================
%  Axis helpers
%  ======================================================================
function prepare_3d_axis(ax, limits)
hold(ax, "on");
grid(ax, "on");
box(ax, "off");
xlabel(ax, "East [m]");
ylabel(ax, "North [m]");
zlabel(ax, "Up [m]");
set(ax, ...
    "XLim", limits(1, :), ...
    "YLim", limits(2, :), ...
    "ZLim", limits(3, :), ...
    "Projection", "orthographic", ...
    "XLimMode", "manual", "YLimMode", "manual", "ZLimMode", "manual");
% axis equal は手動 XLim と競合するので使わない。daspect を manual にすれば
% stretch-to-fill は自動的に無効になり、比率は保たれる。
% axis vis3d は付けないこと。CameraViewAngleMode が manual に固定され、
% 斜め視点で投影が軸の描画領域からはみ出しても縮小されなくなる。
daspect(ax, [1 1 1]);
lighting(ax, "flat");
end

function len = triad_length(limits, origin)
%TRIAD_LENGTH  三軸が軸範囲からはみ出さない長さを選ぶ
%   三軸は原点から正方向にだけ伸びるので、各軸の上限までの余裕の最小値で
%   決まる。ラベルは軸先端の 1.14 倍の位置に置かれるため、その分を見込む。
labelMargin = 1.14;
room = min(limits(:, 2) - origin(:));
len  = 0.70 * max(room, 0) / labelMargin;
end

function limits = world_axis_limits(position, bodyLength)
low  = min(position, [], 2);
high = max(position, [], 2);
low(3) = min(low(3), 0);

verticalSpan  = max(high(3) - low(3), 8 * bodyLength);
minHorizontal = max(0.30 * verticalSpan, 10 * bodyLength);
for k = 1:2
    if high(k) - low(k) < minHorizontal
        center  = 0.5 * (low(k) + high(k));
        low(k)  = center - 0.5 * minHorizontal;
        high(k) = center + 0.5 * minHorizontal;
    end
end

pad = 0.05 * verticalSpan;
limits = [low(1) - pad, high(1) + pad;
          low(2) - pad, high(2) + pad;
          low(3),       high(3) + pad];
end

function scale = resolve_world_scale(limits, opts)
if ~isempty(opts.WorldScale)
    scale = opts.WorldScale;
    return
end
% 鳥観図で機体が全高の約 5% になるように丸めた倍率を選ぶ
scale = max(1, round(0.05 * diff(limits(3, :)) / opts.BodyLength));
end

function state = on_off(flag)
if flag
    state = "on";
else
    state = "off";
end
end

function h = create_ground_plane(ax, centerEN, halfSpan)
halfSpan = max(halfSpan, 1);
[e, n] = meshgrid(centerEN(1) + halfSpan * [-1 1], centerEN(2) + halfSpan * [-1 1]);
h = surface(ax, e, n, zeros(2), ...
    ... FaceAlpha は使わない。1 未満にすると深度ソートを伴う描画経路になり、
    ... フレームあたりのコストが数倍になる。薄さは色そのもので表現する。
    "FaceColor", [0.95 0.96 0.94], ...
    "EdgeColor", [0.82 0.83 0.80], ...
    "FaceLighting", "none");
end


%% ======================================================================
%  Reference-frame triads
%  ======================================================================
function colors = world_triad_colors()
colors = repmat([0.45 0.45 0.48], 3, 1);
end

function colors = reference_triad_colors()
colors = repmat([0.10 0.35 0.60], 3, 1);
end

function colors = body_triad_colors()
colors = [0.85 0.16 0.16;   % Xb 機首方向
          0.10 0.62 0.28;   % Yb（姿勢ゼロで East）
          0.15 0.35 0.90];  % Zb（姿勢ゼロで North）
end

function triad = create_triad(ax, len, labels, colors, lineWidth)
triad.line = gobjects(1, 3);
triad.text = gobjects(1, 3);
for k = 1:3
    triad.line(k) = plot3(ax, nan, nan, nan, "Color", colors(k, :), "LineWidth", lineWidth);
    triad.text(k) = text(ax, nan, nan, nan, labels(k), ...
        "Color", colors(k, :), "FontWeight", "bold", "FontSize", 9, ...
        "HorizontalAlignment", "center");
    triad.line(k).Annotation.LegendInformation.IconDisplayStyle = "off";
end
triad.length = len;
end

function update_triad(triad, R, origin, len)
% R の各列が表示座標系で見た軸方向ベクトル
for k = 1:3
    tip = origin + len * R(:, k);
    set(triad.line(k), ...
        "XData", [origin(1) tip(1)], ...
        "YData", [origin(2) tip(2)], ...
        "ZData", [origin(3) tip(3)]);
    set(triad.text(k), "Position", (origin + 1.14 * len * R(:, k)).');
end
end


%% ======================================================================
%  Rocket graphics
%  ======================================================================
function rocket = create_rocket_graphics(ax, bodyLength, cgFraction, showTailRing)
geom = rocket_geometry(bodyLength, cgFraction);

rocket.surfaces = [ ...
    surf(ax, geom.body.X, geom.body.Y, geom.body.Z, ...
        "FaceColor", [0.90 0.91 0.94], "EdgeColor", "none", ...
        "FaceLighting", "flat", "AmbientStrength", 0.55, "UserData", geom.body), ...
    surf(ax, geom.nose.X, geom.nose.Y, geom.nose.Z, ...
        "FaceColor", [0.78 0.18 0.16], "EdgeColor", "none", ...
        "FaceLighting", "flat", "AmbientStrength", 0.55, "UserData", geom.nose)];

% ロール基準ストライプ。フィンは全て同色にして、色は座標軸専用に残す
rocket.lines = plot3(ax, geom.stripe(:, 1), geom.stripe(:, 2), geom.stripe(:, 3), ...
    "LineWidth", 2.0, "Color", [0.95 0.60 0.10], "UserData", geom.stripe);

rocket.patches = gobjects(1, 4);
for k = 1:4
    rocket.patches(k) = patch(ax, ...
        "XData", geom.fin{k}(:, 1), "YData", geom.fin{k}(:, 2), "ZData", geom.fin{k}(:, 3), ...
        "FaceColor", [0.66 0.69 0.74], "EdgeColor", [0.35 0.36 0.40], ...
        "UserData", geom.fin{k});
end

if showTailRing
    rocket.lines(end + 1) = plot3(ax, geom.tail(:, 1), geom.tail(:, 2), geom.tail(:, 3), ...
        "LineWidth", 1.2, "Color", [0.25 0.25 0.28], "UserData", geom.tail);
end

for h = [rocket.surfaces, rocket.lines, rocket.patches]
    h.Annotation.LegendInformation.IconDisplayStyle = "off";
end
end

function chute = create_chute_graphics(ax, diameter, riserLength, scale)
%CREATE_CHUTE_GRAPHICS  パラシュートの半球キャノピーとライザーを作る
%   キャノピーは取付点から相対風の逆向き（進行方向の反対）に riserLength だけ
%   離れた位置に置く。展張率でスケールするので、開傘の途中も表現できる。
nu = 12; nv = 6;
theta = linspace(0, 2*pi, nu);
phi   = linspace(0, pi/2, nv).';
radius = 0.5 * diameter * scale;
chute.canopyLocal.X = radius * (sin(phi) * cos(theta));
chute.canopyLocal.Y = radius * (sin(phi) * sin(theta));
chute.canopyLocal.Z = radius * repmat(cos(phi), 1, nu);
chute.riserLocal = radius;
chute.scale = scale;
chute.riserLength = riserLength * scale;

chute.canopy = surf(ax, nan(nv, nu), nan(nv, nu), nan(nv, nu), ...
    "FaceColor", [0.90 0.35 0.20], "EdgeColor", [0.55 0.20 0.10], ...
    "FaceLighting", "flat", "AmbientStrength", 0.6, "Visible", "off");
chute.lines = plot3(ax, nan, nan, nan, "Color", [0.35 0.35 0.38], ...
    "LineWidth", 0.8, "Visible", "off");
chute.canopy.Annotation.LegendInformation.IconDisplayStyle = "off";
chute.lines.Annotation.LegendInformation.IconDisplayStyle = "off";
end


function update_chute(chute, attachPoint, upDir, openFraction)
%UPDATE_CHUTE  傘を取付点の上方（相対風の逆向き）に配置する
%   openFraction が 0 のときは非表示、1 で全開。途中は半径と距離を比例させる。
if openFraction < 0.02
    set(chute.canopy, "Visible", "off");
    set(chute.lines,  "Visible", "off");
    return
end

scale  = max(openFraction, 0.05);
center = attachPoint(:) + chute.riserLength * scale * upDir(:);

% キャノピーのローカル z 軸を upDir に向ける回転を作る
R = frame_from_axis(upDir(:));
[x, y, z] = transform_mesh(scale * chute.canopyLocal.X, ...
    scale * chute.canopyLocal.Y, scale * chute.canopyLocal.Z, R, center);
set(chute.canopy, "XData", x, "YData", y, "ZData", z, "Visible", "on");

% ライザー: 取付点からキャノピー裾の数点へ
rim = [x(1, :); y(1, :); z(1, :)];
pick = 1:3:size(rim, 2);
lx = []; ly = []; lz = [];
for k = pick
    lx = [lx, attachPoint(1), rim(1, k), nan]; %#ok<AGROW>
    ly = [ly, attachPoint(2), rim(2, k), nan]; %#ok<AGROW>
    lz = [lz, attachPoint(3), rim(3, k), nan]; %#ok<AGROW>
end
set(chute.lines, "XData", lx, "YData", ly, "ZData", lz, "Visible", "on");
end


function R = frame_from_axis(zAxis)
%FRAME_FROM_AXIS  与えた方向を z 軸とする正規直交基底を作る
zAxis = zAxis / max(norm(zAxis), eps);
ref = [1; 0; 0];
if abs(dot(ref, zAxis)) > 0.9
    ref = [0; 1; 0];
end
xAxis = cross(ref, zAxis); xAxis = xAxis / max(norm(xAxis), eps);
yAxis = cross(zAxis, xAxis);
R = [xAxis, yAxis, zAxis];
end


function geom = rocket_geometry(bodyLength, cgFraction)
% ローカルメッシュ座標: x 前方 / y 左 / z 上。UEN 基準では機体軸と一致する。
% 原点は cgFraction の位置（位置ログが指す点）に置く。
radius  = 0.05 * bodyLength;
bodyEnd = 0.82 * bodyLength;
xShift  = -cgFraction * bodyLength;
theta   = linspace(0, 2 * pi, 12);
unitRow = ones(size(theta));

geom.body.X = [zeros(size(theta)); bodyEnd * unitRow] + xShift;
geom.body.Y = radius * [cos(theta); cos(theta)];
geom.body.Z = radius * [sin(theta); sin(theta)];

geom.nose.X = [bodyEnd * unitRow; bodyLength * unitRow] + xShift;
geom.nose.Y = [radius * cos(theta); zeros(size(theta))];
geom.nose.Z = [radius * sin(theta); zeros(size(theta))];

tailTheta = linspace(0, 2 * pi, 16).';
geom.tail = [xShift * ones(size(tailTheta)), radius * cos(tailTheta), radius * sin(tailTheta)];

xStripe = linspace(0.12 * bodyLength, 0.90 * bodyEnd, 20).' + xShift;
geom.stripe = [xStripe, 0.90 * radius * ones(size(xStripe)), 0.30 * radius * ones(size(xStripe))];

rootLE = 0.04 * bodyLength + xShift;
rootTE = 0.18 * bodyLength + xShift;
tipLE  = 0.08 * bodyLength + xShift;
tipTE  = 0.15 * bodyLength + xShift;
span   = 0.16 * bodyLength;

geom.fin = { ...
    [rootLE  radius 0; rootTE  radius 0; tipTE  span 0; tipLE  span 0], ...
    [rootLE -radius 0; rootTE -radius 0; tipTE -span 0; tipLE -span 0], ...
    [rootLE 0  radius; rootTE 0  radius; tipTE 0  span; tipLE 0  span], ...
    [rootLE 0 -radius; rootTE 0 -radius; tipTE 0 -span; tipLE 0 -span]};
end

function update_rocket_pose(rocket, position, R)
for h = rocket.surfaces
    geom = h.UserData;
    [x, y, z] = transform_mesh(geom.X, geom.Y, geom.Z, R, position);
    set(h, "XData", x, "YData", y, "ZData", z);
end
for h = [rocket.lines, rocket.patches]
    pts = h.UserData * R.' + position.';
    set(h, "XData", pts(:, 1), "YData", pts(:, 2), "ZData", pts(:, 3));
end
end

function [x, y, z] = transform_mesh(xLocal, yLocal, zLocal, R, position)
pts = [xLocal(:), yLocal(:), zLocal(:)] * R.' + position.';
x = reshape(pts(:, 1), size(xLocal));
y = reshape(pts(:, 2), size(yLocal));
z = reshape(pts(:, 3), size(zLocal));
end


%% ======================================================================
%  Animation loop
%  ======================================================================
function run_animation(ui, logs, opts)
% ライブスクリプトに埋め込まれた図では、ループ中の drawnow が出力に反映されず
% 最終コマだけの静止画になる。開始と終了をアニメーションとして登録すると、
% 出力に再生コントロール付きのアニメーションとして表示される。
video = open_video(opts, logs);
cleanup = onCleanup(@() close_video(video)); %#ok<NASGU>
frameSize = [];

tic;
for k = 1:numel(logs.t)
    position = logs.position_m(:, k);
    R = enu_from_body(logs.quaternionBI(:, k));

    chuteOpen = log_value(logs, "chuteOpen", k);
    if ~isfinite(chuteOpen)
        chuteOpen = 0;
    end
    update_world_view(ui.world, logs, k, position, R, chuteOpen);
    if ~isempty(ui.frame)
        update_frame_panel(ui.frame, R, chuteOpen);
        set(ui.info.readout, "String", format_readout(logs, k));
    end
    update_trends(ui.trends, logs.t(k));

    % limitrate は描画を間引いてコマ落ちさせるので使わない。
    % タイトルの毎フレーム更新も軸のレイアウト計算を誘発するため行わない
    % （時刻は状態量パネルに出ている）。
    drawnow;

    if opts.SaveVideo
        [video, frameSize] = write_frame(video, ui.fig, frameSize);
    else
        pause(max(0, (k - 1) / opts.FrameRate - toc));
    end
end
end

function update_world_view(world, logs, k, position, R, chuteOpen)
update_rocket_pose(world.rocket, position, R);
if world.showTrail
    set(world.trail, ...
        "XData", logs.position_m(1, 1:k), ...
        "YData", logs.position_m(2, 1:k), ...
        "ZData", logs.position_m(3, 1:k));
    set(world.shadow, ...
        "XData", logs.position_m(1, 1:k), ...
        "YData", logs.position_m(2, 1:k), ...
        "ZData", zeros(1, k));
end
set(world.dropLine, ...
    "XData", [position(1) position(1)], ...
    "YData", [position(2) position(2)], ...
    "ZData", [0 position(3)]);
set(world.marker, "XData", position(1), "YData", position(2), "ZData", position(3));

% 傘は機首側の取付点から、機体軸の前方（＝降下中は上方）へ伸ばす
noseDir = R * [1; 0; 0];
update_chute(world.chute, position + world.noseOffset * noseDir, noseDir, chuteOpen);
end

function update_frame_panel(frame, R, chuteOpen)
update_triad(frame.bodyTriad, R, [0; 0; 0], 1.05);
update_rocket_pose(frame.rocket, [0; 0; 0], R);
noseDir = R * [1; 0; 0];
update_chute(frame.chute, frame.noseOffset * noseDir, noseDir, chuteOpen);
end

function update_trends(trends, tNow)
for k = 1:numel(trends)
    set(trends(k).cursor, "XData", [tNow tNow]);
end
end

function txt = format_readout(logs, k)
spec = readout_specification();
txt  = strings(numel(spec), 1);
for n = 1:numel(spec)
    lineFormat = "%-6s" + spec(n).format + " %s";
    txt(n) = sprintf(lineFormat, spec(n).label, log_value(logs, spec(n).field, k), spec(n).unit);
end
end

function spec = readout_specification()
% すべて四元数由来。roll は機首軸まわりのねじれ、swing は機首方向のずれ。
spec = struct( ...
    "field",  {"t",     "altitude_m", "speed_mps", "rollNose_deg", "swing_deg", "noseElev_deg", "noseAzim_deg", "p_dps", "delta_deg"}, ...
    "label",  {"t",     "alt",        "V",         "roll",         "swing",     "elev",         "azim",         "p",     "delta"}, ...
    "unit",   {"s",     "m",          "m/s",       "deg",          "deg",       "deg",          "deg",          "deg/s", "deg"}, ...
    "format", {"%7.2f", "%7.1f",      "%7.1f",     "%7.1f",        "%7.1f",     "%7.1f",        "%7.1f",        "%7.1f", "%7.1f"});
end


%% ======================================================================
%  Video output
%  ======================================================================
function video = open_video(opts, logs)
video = [];
if ~opts.SaveVideo
    return
end
video = VideoWriter(char(opts.VideoFile), "MPEG-4");
video.FrameRate = min(60, max(1, round(opts.FrameRate)));
open(video);
end

function [video, frameSize] = write_frame(video, fig, frameSize)
frame = getframe(fig);
if isempty(frameSize)
    frameSize = size(frame.cdata, [1 2]);
elseif ~isequal(size(frame.cdata, [1 2]), frameSize)
    frame.cdata = imresize(frame.cdata, frameSize);
end
writeVideo(video, frame);
end

function close_video(video)
if ~isempty(video) && isobject(video)
    close(video);
end
end