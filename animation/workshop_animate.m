function workshop_animate(simOut, options)
%WORKSHOP_ANIMATE  飛行アニメーションを再生する（学生が触らない処理部）
%
%   機体長・重心位置・傘の寸法はベースワークスペースの設計値から取る。
%   再生の速さだけオプションで変えられる。

arguments
    simOut
    options.PlaybackSpeed (1,1) double = 4
    options.FrameRate     (1,1) double = 12
    options.SaveVideo     (1,1) logical = false
end

g = @(n) evalin("base", n);

animate_flight(simOut, ...
    "PlaybackSpeed",  options.PlaybackSpeed, ...
    "FrameRate",      options.FrameRate, ...
    "BodyLength",     g("rocket_length"), ...
    "CgFraction",     g("rocket_cg0(1)") / g("rocket_length"), ...
    "ChuteDiameter",  g("chute_diameter"), ...
    "ChuteRiser",     g("chute_riserLength"), ...
    "SaveVideo",      options.SaveVideo);
end
