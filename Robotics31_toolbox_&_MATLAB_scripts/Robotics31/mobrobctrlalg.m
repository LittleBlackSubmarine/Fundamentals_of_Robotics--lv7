function memNew = mobrobctrlalg(robot, ctrlparam, r, U, mem)

u = mem(1:2);
iPt = mem(4);
we = mem(5:7);

xe = we(1);
ye = we(2);
alphae = we(3);

cae = cos(alphae);
sae = sin(alphae);

RA0 = [cae -sae;
    sae cae];

Ts = ctrlparam.T;

% prediction (odometry)

Kp = robot.Km * robot.rw;

v = 0.5*Kp*(u(1) + u(2));
om = Kp*(u(2) - u(1)) / robot.d;

alphaeNew = alphae + om*Ts;
xeNew = xe + v * cae * Ts;
yeNew = ye + v * sae * Ts;


weNew = [xeNew; yeNew; alphaeNew];

% motion control

m = length(ctrlparam.path);

goal = ctrlparam.path(:,m);

E = goal - we(1:2);

distToGoal = sqrt(E'*E);

uNew(3) = 0;

if distToGoal < 0.1
    uNew(3) = 1;
end

F = ctrlparam.path(:,iPt) - we(1:2);

d = sqrt(F'*F);

if d > 1     %Define in ctrlparam parameter r 
    F = F / d;
end
   
iScl = 0;
rmin = robot.maxr;

for iS = 1:robot.nS
    if r(iS) >= 0 && r(iS) < ctrlparam.rho0
       if r(iS) < rmin || iScl == 0 
           rmin = r(iS);
           iScl = iS;
       end
    end
end

if iScl > 0
    F = F - ctrlparam.eta * (1./rmin-1/ctrlparam.rho0) / rmin^2 * RA0 * robot.S(1:2,iScl) / robot.r;
end
        
alphar = atan2(F(2), F(1));

e = alphar - we(3);
if e > pi
    e = e - 2 * pi;
else
    if e < -pi
        e = e + 2 * pi;
    end
end
    
omNew = e;

vtot = ctrlparam.vmax * sqrt(F'*F);

vNew = vtot - robot.r * abs(omNew);

if vNew < 0
    vNew = 0;
    omNew = sign(om) * vtot / robot.r;
end

uNew(1) = (vNew - 0.5 * omNew * robot.d) / Kp;
uNew(2) = (vNew + 0.5 * omNew * robot.d) / Kp;

iPtNew = iPt;

if d < 0.1
    if iPt < m
        iPtNew = iPt + 1
    end
end

uNew = uNew';


memNew = [uNew; iPtNew; weNew];

