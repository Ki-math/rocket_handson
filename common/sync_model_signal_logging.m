function sync_model_signal_logging(modelName, saveModel)
%SYNC_MODEL_SIGNAL_LOGGING  信号名をサフィックス無しに統一し、ログを有効化する
%
%   SYNC_MODEL_SIGNAL_LOGGING(MODELNAME) はモデルを再配線せず、既存の名前付き線
%   に対して次を行う。
%     1. 旧名（_deg / _dps / _m など単位サフィックス付き）を新名にリネーム
%     2. 信号ログを有効化し、logging_decimation を反映
%
%   SYNC_MODEL_SIGNAL_LOGGING(MODELNAME, TRUE) は最後にモデルを保存する。
%   既定は保存しない。新しい Simulink で作成したモデルを古いリリースの
%   セッションで保存すると形式が落ちるため、保存は明示的に選ぶ。
%
%   単位はモデル全体で SI + rad に統一している。信号名に単位を付けないのは、
%   名前と実体が食い違ったときに気づけなくなるのを避けるため。

arguments
    modelName (1,1) string
    saveModel (1,1) logical = false
end

if ~bdIsLoaded(modelName)
    load_system(modelName);
end

set_param(modelName, "SignalLogging", "on", "SignalLoggingName", "logsout");

decimation = 1;
if evalin("base", "exist('logging_decimation','var')")
    decimation = evalin("base", "logging_decimation");
end

map = signal_rename_map(modelName);
for k = 1:size(map, 1)
    apply_signal(modelName, map(k, 1), map(k, 2), decimation);
end

if saveModel
    save_system(modelName);
end
end


function map = signal_rename_map(modelName)
%SIGNAL_RENAME_MAP  [旧名, 新名] の対応表。新名が既に付いていれば何もしない。
switch string(modelName)
    case "rocket_6dof_with_roll_controller"
        map = [ ...
            "positionUEN_m",    "positionUEN"; ...
            "velocityBody_mps", "velocityBody"; ...
            "quaternionBI",     "quaternionBI"; ...
            "euler",            "euler"; ...
            "pqr",              "pqr"; ...
            "qbar_Pa",          "qbar"; ...
            "Mach",             "Mach"; ...
            "forcesBody_N",     "forcesBody"; ...
            "momentsBody_Nm",   "momentsBody"; ...
            "phiCmd_deg",       "phiCmd"; ...
            "pCmd_dps",         "pCmd"; ...
            "deltaCmd_deg",     "deltaCmd"; ...
            "delta_deg",        "delta"];

    case "roll_1dof_model"
        map = [ ...
            "phiCmd_deg",   "phiCmd"; ...
            "pCmd_dps",     "pCmd"; ...
            "deltaCmd_deg", "deltaCmd"; ...
            "delta_deg",    "delta"; ...
            "phi_deg",      "phi"; ...
            "p_dps",        "p"];

    otherwise
        error("sync_model_signal_logging:UnsupportedModel", ...
            "対応していないモデルです: %s", modelName);
end
end


function apply_signal(modelName, oldName, newName, decimation)
%APPLY_SIGNAL  新名の線があればそれを、無ければ旧名の線をリネームして使う
lineHandle = find_line_by_name(modelName, newName);
if isempty(lineHandle)
    lineHandle = find_line_by_name(modelName, oldName);
end
if isempty(lineHandle)
    warning("sync_model_signal_logging:SignalNotFound", ...
        "%s に %s (旧名 %s) の線が見つかりません。", modelName, newName, oldName);
    return
end

set_param(lineHandle, "Name", char(newName));
Simulink.sdi.markSignalForStreaming(lineHandle, "on");
try
    set_param(lineHandle, "DataLoggingDecimation", num2str(decimation));
catch
end
end


function lineHandle = find_line_by_name(modelName, signalName)
%FIND_LINE_BY_NAME  名前が一致する線。分岐している場合は親の線を返す。
lineHandle = [];
allLines = find_system(modelName, "FindAll", "on", "type", "line");
for k = 1:numel(allLines)
    try
        if strcmp(get_param(allLines(k), "Name"), char(signalName))
            if get_param(allLines(k), "LineParent") == -1
                lineHandle = allLines(k);
                return
            end
            if isempty(lineHandle)
                lineHandle = allLines(k);
            end
        end
    catch
    end
end
end
