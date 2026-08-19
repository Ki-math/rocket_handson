function simOut = workshop_simulate(modelName, stopTime)
%WORKSHOP_SIMULATE  モデルを1回実行する（学生が触らない処理部）
%
%   SIMOUT = WORKSHOP_SIMULATE(MODELNAME) は sim_stopTime まで実行する。
%   SIMOUT = WORKSHOP_SIMULATE(MODELNAME, STOPTIME) で停止時刻を上書きできる。

if nargin < 2
    stopTime = evalin("base", "sim_stopTime");
end

in = Simulink.SimulationInput(char(modelName));
in = in.setModelParameter("StopTime", num2str(stopTime));
simOut = sim(in);
end
