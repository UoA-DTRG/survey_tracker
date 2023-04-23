clear;

% setup parameters
params;
init_mpc_model;

EXPORT = true;
COMPILE = false;

BEGIN_ACADO;
    
    max_vel = 2.0;  % Velocity input constraint
    max_vel_z = 0.2; % Velocity z input constraint
    max_yaw = 1.5;  % Yaw rate input constraint
    
    OnlineData tau_x;
    OnlineData tau_y;
    OnlineData tau_z;
    OnlineData tau_phi;
    
    acadoSet('problemname', 'uav_vel_mpc');
    
    DifferentialState position(3) velocity(3) yaw;
    Control velocity_setpoint(3) yaw_rate;
    
    % Set up the model
%     f = dot([position; velocity; yaw]) == ...
%         [velocity; ...
%         -1/tau * velocity + 1/tau * velocity_setpoint; ...
%         -1/tau_y * yaw + 1/tau_y * yaw_rate];
    f = [dot(position) == velocity; ...
         %dot(velocity) == -1/tau * velocity + 1/tau * velocity_setpoint; ...
         dot(velocity(1)) == -1/tau_x * velocity(1) + 1/tau_x * velocity_setpoint(1); ...
         dot(velocity(2)) == -1/tau_y * velocity(2) + 1/tau_y * velocity_setpoint(2); ...
         dot(velocity(3)) == -1/tau_z * velocity(3) + 1/tau_z * velocity_setpoint(3); ...
         dot(yaw) == -1/tau_phi * yaw + 1/tau_phi * yaw_rate];
    
    % Optimization vector
    % We want to minimise everything in this matrix
    h = [position; ...
        velocity; ...
        yaw; ...
        velocity_setpoint; ...
        yaw_rate];
    
    % Terminal optimization vector
    % We only want these terms with weighting matrix P
    hN = [position; ...
         velocity; ...
         yaw];
     
    h = [diffStates; controls];
    hN = [diffStates];
    
    % Define our optimal control problem
    ocp = acado.OCP(0.0, N*Ts, N);
    
    % Weighting matricies. We se them as identity for now
    % They will be populated at runtime
    W = acado.BMatrix(eye(length(h)));
    WN = acado.BMatrix(eye(length(hN)));

    % Set up a least squares optimization
    ocp.minimizeLSQ( W, h );
    ocp.minimizeLSQEndTerm( WN, hN );
    ocp.subjectTo(-0.2 <= velocity_setpoint(3) <= 0.2);
    ocp.subjectTo(-max_vel <= velocity_setpoint(1:2) <= max_vel);
    ocp.subjectTo(-max_yaw <= yaw_rate <= max_yaw);
    ocp.setModel(f);
    
    mpc = acado.OCPexport(ocp);
    %mpc.set('GENERATE_SIMULINK_INTERFACE',      'YES' );
    mpc.set('QP_SOLVER',                        'QP_QPOASES');
    mpc.set('DISCRETIZATION_TYPE',              'MULTIPLE_SHOOTING');
    mpc.set('SPARSE_QP_SOLUTION',               'FULL_CONDENSING_N2');
    mpc.set('CG_HARDCODE_CONSTRAINT_VALUES',    'NO');
    if EXPORT
        mpc.exportCode( 'export_MPC' );
    end
    if COMPILE
        global ACADO_;
        copyfile([ACADO_.pwd '/../../external_packages/qpoases'], 'export_MPC/qpoases')
        
        cd export_MPC
        make_acado_solver_sfunction
        copyfile('acado_solver_sfun.mex*', '../')
        cd ..
    end

END_ACADO;

    