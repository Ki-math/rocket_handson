function workshop_plots(simOut, titleText)
%WORKSHOP_PLOTS  結果の時間履歴を描く（学生が触らない処理部）
%
%   ロール制御の応答と、6DOF の場合は姿勢・角速度も描く。
%   角度はモデル内では rad、表示は deg に換算される。

arguments
    simOut
    titleText (1,1) string = "Result"
end

plot_roll_control_results(simOut, titleText + " - Roll Control");

names = string(simOut.logsout.getElementNames());
if any(names == "euler") && any(names == "pqr")
    plot_6dof_attitude_results(simOut, titleText + " - Attitude");
end
end
