function [obTraj]  = GenerateDyObTraj(xb, ub, evaldt)

global dt;

time = 0;
oneObTraj = [];
obTraj = [];
for inxOb = 1 : length(xb(1, :))
    if ub(:, inxOb) == [0;0]
        continue;
    end
    while time <= evaldt+dt
        xb(:, inxOb) = fb(xb(:, inxOb),ub(:, inxOb));
        xbi = xb( : , inxOb);
        oneObTraj = [oneObTraj xbi];
        time = time + dt;
    end
    time = 0;
    obTraj = [obTraj; oneObTraj];
    oneObTraj = [];
end

