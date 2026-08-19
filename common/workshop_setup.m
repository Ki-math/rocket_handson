function workshop_setup()
%WORKSHOP_SETUP  ワークショップの下準備（学生が触らない処理部）
%
%   パスの追加、モデルの信号ログ設定、パラメータの過不足チェックを行う。
%   ライブスクリプトでパラメータを定義したあとに一度だけ呼ぶ。
%
%   パラメータはベースワークスペース（ライブスクリプトの実行空間）を見る。

projectRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(fullfile(projectRoot, "common"));
addpath(fullfile(projectRoot, "evaluation"));
addpath(fullfile(projectRoot, "animation"));

check_required_parameters();

for modelName = ["roll_1dof_model", "rocket_6dof_with_roll_controller"]
    sync_model_signal_logging(modelName, false);
end

print_flight_condition();
end


function check_required_parameters()
%CHECK_REQUIRED_PARAMETERS  モデルが参照する変数が揃っているか確認する
%   Simulink.findVars で両モデルから抽出した一覧。モデル側で変数名を変えたら
%   ここも更新すると、実行前に齟齬が分かる。
required = [ ...
    "initial_position", "initial_velocityBody", "initial_euler", "initial_pqr", ...
    "rocket_mass0", "rocket_massDry", "rocket_inertia0", "rocket_inertiaDry", ...
    "CG", "CP", "aero_refArea", "aero_refLength", "Fth", "Isp", ...
    "environment_g", "environment_wind", ...
    "Cx_0", "Cy_0", "Cz_0", "Cy_b", "Cz_a", "Cm_a", "Cm_q", "Cn_b", "Cn_r", ...
    "Cl_p", "Cl_delta", ...
    "chute_CdS", "chute_inflateTau", "chute_attachBody", "chute_riserLength", ...
    "fin_maxDeflection", "fin_maxRate", "fin_actuatorTau", ...
    "sensor_gyroTau", "sensor_attitudeTau", ...
    "controller_outer_Kp", "controller_outer_maxRateCmd", "controller_qbarMin", ...
    "controller_inner_Kp", "controller_inner_Ki", "controller_inner_Kd", "controller_inner_N", ...
    "roll_design_qbar", "roll_design_Ixx", "roll_design_phi", "roll_design_p", ...
    "roll_Lp", "roll_Ldelta", ...
    "command_rollStep", "command_stepTime", "propulsion_cutoffTime", ...
    "sim_stopTime", "sim_fixedStep"];

missing = string.empty;
for name = required
    if evalin("base", sprintf("exist('%s','var')", name)) ~= 1
        missing(end + 1) = name; %#ok<AGROW>
    end
end
if ~isempty(missing)
    error("workshop_setup:MissingParameters", ...
        "パラメータが未定義です:\n  %s", strjoin(missing, newline + "  "));
end
end


function print_flight_condition()
%PRINT_FLIGHT_CONDITION  設計値から決まる飛行条件をまとめて表示する
g = @(n) evalin("base", n);
staticMargin = g("CG(1)") - g("CP(1)");
fprintf("--- 設計条件 ---\n");
fprintf("  代表面積      : %.6f m^2 (直径 %.3f m)\n", g("aero_refArea"), g("rocket_diameter"));
fprintf("  静安定余裕    : %.3f m (%.2f caliber)\n", staticMargin, staticMargin / g("rocket_diameter"));
fprintf("  推力 / 燃焼   : %.0f N / %.1f s (mdot %.3f kg/s)\n", ...
    g("Fth"), g("propulsion_burnTime"), g("Fth") / g("Isp"));
fprintf("  ロール舵効き  : %.2f Nm/rad @ qbar %.0f Pa\n", g("roll_Ldelta"), g("roll_design_qbar"));
fprintf("  ロール指令    : %.1f deg @ %.2f s\n", rad2deg(g("command_rollStep")), g("command_stepTime"));
fprintf("  傘 CdS        : %.2f m^2 -> 終端速度 %.1f m/s\n\n", ...
    g("chute_CdS"), sqrt(2 * g("rocket_massDry") * g("environment_g") / (1.225 * g("chute_CdS"))));
end
