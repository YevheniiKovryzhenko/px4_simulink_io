clc; 
generate_params_json(Parameters)

function generate_params_json(Parameters)
    api = px4io.px4API();    
    export_destination_path = fullfile(api.PX4Root, 'src', 'modules', api.PX4ModuleName,'module.yaml');

    % Initialize the root module configuration builder node
    master_module = px4io.parameter.module(api.PX4ModuleName);
    
    % GROUP 1: SIMULINK Vehicle Model Shared Parameters
    group_shared = px4io.parameter.group("SIMULINK Vehicle Model Shared Parameters");
    
    % Vehicle Mass
    group_shared.add(px4io.parameter.float(Parameters.Vehicle.Shared.Mass) ...
        .set_name("SM_MASS") ...
        .set_description("Vehicle mass") ...
        .set_unit(px4io.parameter.unit.KG) ...
        .set_min(0.0)); % Must be strictly positive
    
    % Inertia Tensor matrix flattened to scalar components due to symmetry
    I = Parameters.Vehicle.Shared.InertiaTensor;
    group_shared.add(px4io.parameter.float(I(1,1)).set_name("SM_I_XX").set_unit(px4io.parameter.unit.KG_M2).set_description("Inertia entry Ixx").set_min(0.0));
    group_shared.add(px4io.parameter.float(I(2,2)).set_name("SM_I_YY").set_unit(px4io.parameter.unit.KG_M2).set_description("Inertia entry Iyy").set_min(0.0));
    group_shared.add(px4io.parameter.float(I(3,3)).set_name("SM_I_ZZ").set_unit(px4io.parameter.unit.KG_M2).set_description("Inertia entry Izz").set_min(0.0));
    group_shared.add(px4io.parameter.float(I(1,2)).set_name("SM_I_XY").set_unit(px4io.parameter.unit.KG_M2).set_description("Inertia cross entry Ixy"));
    group_shared.add(px4io.parameter.float(I(1,3)).set_name("SM_I_XZ").set_unit(px4io.parameter.unit.KG_M2).set_description("Inertia cross entry Ixz"));
    group_shared.add(px4io.parameter.float(I(2,3)).set_name("SM_I_YZ").set_unit(px4io.parameter.unit.KG_M2).set_description("Inertia cross entry Iyz"));
        
    master_module.add_group(group_shared);

    % GROUP 2: SIMULINK Fixed-Wing Guidance & Differential Flatness
    group_guidance = px4io.parameter.group("SIMULINK Guidance Parameters");

    % Guidance Delayed Handoff
    group_guidance.add(px4io.parameter.float(Parameters.Vehicle.Guidance.AutonomusSwitchingDelay) ...
        .set_name("SM_GD_SW_DELAY") ...
        .set_description("Autonomous trajectory state handoff initialization latency delay") ...
        .set_unit(px4io.parameter.unit.S) ...
        .set_min(0.0));
    
    df = Parameters.Vehicle.Guidance.DifferentialFlatness;
    group_guidance.add(px4io.parameter.float(df.AlphaNominal).set_name("SM_DF_ALPHA_NOM").set_unit(px4io.parameter.unit.RAD).set_description("Nominal angle of attack at cruise 1g trim"));
    group_guidance.add(px4io.parameter.float(df.AlphaGain).set_name("SM_DF_ALPHA_GAIN").set_unit(px4io.parameter.unit.RAD).set_description("Load factor sensitivity feedback scaling gain").set_min(0.0));
    group_guidance.add(px4io.parameter.float(df.VelocityHover).set_name("SM_DF_VEL_HOVER").set_unit(px4io.parameter.unit.M_S).set_description("Characteristic aerodynamic airspeed threshold for hover").set_min(0.0));
    group_guidance.add(px4io.parameter.float(df.VelocityCruise).set_name("SM_DF_VEL_CRUISE").set_unit(px4io.parameter.unit.M_S).set_description("Characteristic aerodynamic airspeed threshold for cruise").set_min(0.0));
    group_guidance.add(px4io.parameter.float(df.VelocityMid).set_name("SM_DF_VEL_MID").set_unit(px4io.parameter.unit.M_S).set_description("Midpoint transition blending velocity").set_min(0.0));
    group_guidance.add(px4io.parameter.float(df.RhoSigma).set_name("SM_DF_RHO_SIGMA").set_unit(px4io.parameter.unit.NORM).set_description("Regime smooth transition tracking weighting parameter").set_min(0.0).set_max(1.0));
    group_guidance.add(px4io.parameter.float(df.RhoBound).set_name("SM_DF_RHO_BOUND").set_unit(px4io.parameter.unit.NORM).set_description("Smooth blending factor for hover/cruise spatial boundary tracking").set_min(0.0).set_max(1.0));
    group_guidance.add(px4io.parameter.float(df.RhoStage).set_name("SM_DF_RHO_STAGE").set_unit(px4io.parameter.unit.NORM).set_description("Smooth blending factor for multi-stage allocation handoffs").set_min(0.0).set_max(1.0));
    
    master_module.add_group(group_guidance);

    % GROUP 3: SIMULINK Lumped Matrix Aerodynamics
    group_aero = px4io.parameter.group("SIMULINK Lumped Matrix Aerodynamics");
    
    % Automatically scales into sub-vector blocks via num_instances rules
    group_aero.add(px4io.parameter.matrix(Parameters.Vehicle.Aerodynamics.D).set_name("SM_AERO_D").set_description("Aerodynamic lumped parameter D system grid tensor matrix"));
    group_aero.add(px4io.parameter.matrix(Parameters.Vehicle.Aerodynamics.A).set_name("SM_AERO_A").set_description("Aerodynamic lumped parameter A system grid tensor matrix"));
    group_aero.add(px4io.parameter.matrix(Parameters.Vehicle.Aerodynamics.B).set_name("SM_AERO_B").set_description("Aerodynamic lumped parameter B system grid tensor matrix"));
    
    master_module.add_group(group_aero);

    % GROUP 4: SIMULINK Pilot Stick Mapping Boundaries
    group_sticks = px4io.parameter.group("SIMULINK Stick Mapping Boundaries");
    
    stk = Parameters.Vehicle.ControlSystem.Sticks;
    group_sticks.add(px4io.parameter.float(stk.VelocityHorizonal).set_name("SM_STK_VELXY_MAX").set_unit(px4io.parameter.unit.M_S).set_description("Max horizontal velocity reference request from stick").set_min(0.0));
    group_sticks.add(px4io.parameter.float(stk.VelocityVertical).set_name("SM_STK_VEL_Z_MAX").set_unit(px4io.parameter.unit.M_S).set_description("Max vertical velocity reference request from stick").set_min(0.0));
    group_sticks.add(px4io.parameter.float(stk.Attitude).set_name("SM_STK_ATT_MAX").set_instances(3,1).set_unit(px4io.parameter.unit.RAD).set_description("Max roll pitch and yaw orientation bounding request from sticks"));
    group_sticks.add(px4io.parameter.float(stk.AngularRate).set_name("SM_STK_RATE_MAX").set_instances(3,1).set_unit(px4io.parameter.unit.RAD_S).set_description("Max roll pitch and yaw rate bounding request from sticks"));
    group_sticks.add(px4io.parameter.float(stk.Thrust).set_name("SM_STK_THR_MAX").set_unit(px4io.parameter.unit.NORM).set_description("Max manual normalized aggregate thrust command limits").set_min(0.0).set_max(1.0));
    group_sticks.add(px4io.parameter.float(stk.Moment).set_name("SM_STK_MOM_MAX").set_instances(3,1).set_unit(px4io.parameter.unit.NM).set_description("Max physical body moments allocated via stick override steps"));
    
    master_module.add_group(group_sticks);

    % GROUP 5: SIMULINK Multirotor Flight Control System
    group_mc_ctrl = px4io.parameter.group("Simulink Multirotor Control");
    
    % MC Outer Position Tracking Loops Parameters
    mc_p = [];
    if isequal(class(Parameters.Vehicle.ControlSystem.Position), 'Airframe.Control.Position.Multirotor')
        mc_p = Parameters.Vehicle.ControlSystem.Position;
    elseif isequal(class(Parameters.Vehicle.ControlSystem.Position), 'Airframe.Control.Position.TiltWing')
        mc_p = Parameters.Vehicle.ControlSystem.Position.Hover;
    end
    if ~isempty(mc_p)
        group_mc_ctrl.add(px4io.parameter.matrix(mc_p.PositionProportionalGain).set_name("SM_M_POS_KP").set_description("Multirotor position proportional feedback gain matrix"));
        group_mc_ctrl.add(px4io.parameter.matrix(mc_p.VelocityProportionalGain).set_name("SM_M_VEL_KP").set_description("Multirotor velocity proportional feedback gain matrix"));
        group_mc_ctrl.add(px4io.parameter.matrix(mc_p.VelocityIntegralGain).set_name("SM_M_VEL_KI").set_description("Multirotor velocity error integral tracker feedback loop matrix"));
        group_mc_ctrl.add(px4io.parameter.matrix(mc_p.VelocityDerivativeGain).set_name("SM_M_VEL_KD").set_description("Multirotor velocity error tracking derivative feedback gain matrix"));
        
        group_mc_ctrl.add(px4io.parameter.float(mc_p.MaxHorizontalPositionError).set_name("SM_M_ERR_POS_XY").set_unit(px4io.parameter.unit.M).set_description("Maximum allowed horizontal position tracking spatial error deviation magnitude"));
        group_mc_ctrl.add(px4io.parameter.float(mc_p.MaxVerticalPositionError).set_name("SM_M_ERR_POS_Z").set_unit(px4io.parameter.unit.M).set_description("Maximum allowed vertical position error tracking limit band"));
        group_mc_ctrl.add(px4io.parameter.float(mc_p.MaxHorizontalSpeed).set_name("SM_M_SPD_XY_MAX").set_unit(px4io.parameter.unit.M_S).set_description("Maximum targeted horizontal velocity profile envelope"));
        group_mc_ctrl.add(px4io.parameter.float(mc_p.MinMaxVerticalSpeed).set_name("SM_M_SPD_Z_LIM").set_instances(2,1).set_unit(px4io.parameter.unit.M_S).set_description("Minimum and Maximum targeted vertical speed profile bounds"));
        
        group_mc_ctrl.add(px4io.parameter.float(mc_p.MaxHorizontalVelocityError).set_name("SM_M_ERR_VEL_XY").set_unit(px4io.parameter.unit.M_S).set_description("Maximum horizontal speed profile tracking deviation error saturation limit"));
        group_mc_ctrl.add(px4io.parameter.float(mc_p.MaxVerticalVelocityError).set_name("SM_M_ERR_VEL_Z").set_unit(px4io.parameter.unit.M_S).set_description("Maximum vertical descent/climb speed profile tracking deviation error saturation limit"));
        group_mc_ctrl.add(px4io.parameter.float(mc_p.MaxHorizontalAccelerationError).set_name("SM_M_ERR_ACC_XY").set_unit(px4io.parameter.unit.M_S2).set_description("Maximum horizontal kinematic dynamic acceleration trajectory error buffer bound"));
        group_mc_ctrl.add(px4io.parameter.float(mc_p.MaxVerticalAccelerationError).set_name("SM_M_ERR_ACC_Z").set_unit(px4io.parameter.unit.M_S2).set_description("Maximum vertical kinematic dynamic acceleration error saturation threshold envelope"));
        
        group_mc_ctrl.add(px4io.parameter.float(mc_p.MaxTotalAcceleration).set_name("SM_M_ACC_MAX_G").set_unit(px4io.parameter.unit.NORM).set_description("Max total translation acceleration saturation limits specified in multiples of standard gravity"));
    
        group_mc_ctrl.add(px4io.parameter.float(mc_p.MinThrustAcceleration).set_name("SM_M_ACC_MIN_G").set_unit(px4io.parameter.unit.NORM).set_description("Min collective vertical upward thrust floor buffer scaling specified in multiples of gravity"));
        group_mc_ctrl.add(px4io.parameter.float(rad2deg(mc_p.MaxBodyTiltAngle)).set_name("SM_M_TILT_MAX").set_unit(px4io.parameter.unit.DEG).set_description("Absolute hard maximum multirotor roll/pitch airframe tilt vector angle boundary"));
    end

    % MC Inner Attitude Tracking Loops Parameters
    mc_a = [];
    if isequal(class(Parameters.Vehicle.ControlSystem.Attitude), 'Airframe.Control.Attitude.Multirotor')
        mc_a = Parameters.Vehicle.ControlSystem.Attitude;
    elseif isequal(class(Parameters.Vehicle.ControlSystem.Attitude), 'Airframe.Control.Attitude.Tiltwing')
        mc_a = Parameters.Vehicle.ControlSystem.Attitude.Hover;
    end
    if ~isempty(mc_a)
        group_mc_ctrl.add(px4io.parameter.matrix(mc_a.AttitudeGain).set_name("SM_M_ATT_KP").set_description("Multirotor core orientation quaternion tracking proportional control loop feedback matrix"));
        group_mc_ctrl.add(px4io.parameter.matrix(mc_a.AngularRateGain).set_name("SM_M_RAT_KP").set_description("Multirotor body axis angular velocity rate proportional control feedback matrix"));
        group_mc_ctrl.add(px4io.parameter.matrix(mc_a.AngularRateIntegralGain).set_name("SM_M_RAT_KI").set_description("Multirotor body axis rates error integral state tracker feedback scaling gain matrix"));
        group_mc_ctrl.add(px4io.parameter.matrix(mc_a.AngularAccelerationGain).set_name("SM_M_ACC_KP").set_description("Multirotor cross coupling angular target acceleration loop proportional scaling gain matrix"));
        group_mc_ctrl.add(px4io.parameter.float(mc_a.MaxAttitudeError).set_name("SM_M_ERR_ATT_LIM").set_unit(px4io.parameter.unit.RAD).set_description("Maximum rotation tracking deviation angle error envelope saturation limit before gain processing"));
        group_mc_ctrl.add(px4io.parameter.float(mc_a.MaxAngularRateError).set_name("SM_M_ERR_RAT_LIM").set_unit(px4io.parameter.unit.RAD_S).set_description("Maximum cross-axis angular velocity tracking rate error buffer saturation cap limits"));
        group_mc_ctrl.add(px4io.parameter.float(mc_a.MaxAngularAccelerationError).set_name("SM_M_ERR_AAC_LIM").set_unit(px4io.parameter.unit.RAD_S2).set_description("Maximum angular reference tracking frame acceleration path deviation error saturation threshold bounds"));
        group_mc_ctrl.add(px4io.parameter.float(mc_a.MaxRateLimit).set_name("SM_M_RAT_REF_MAX").set_unit(px4io.parameter.unit.RAD_S).set_description("Absolute hard maximum target body rate setpoint magnitude commanded via inner stability loops"));
        master_module.add_group(group_mc_ctrl);
    end
    
    % GROUP 6: SIMULINK Fixed-Wing Flight Control System
    group_fw_ctrl = px4io.parameter.group("Simulink Fixedwing Control");
    
    % FW Outer Position Tracking Loops Parameters
    fw_p = [];
    if isequal(class(Parameters.Vehicle.ControlSystem.Position), 'Airframe.Control.Position.FixedWing')
        fw_p = Parameters.Vehicle.ControlSystem.Position;
    elseif isequal(class(Parameters.Vehicle.ControlSystem.Position), 'Airframe.Control.Position.TiltWing')
        fw_p = Parameters.Vehicle.ControlSystem.Position.Cruise;
    end    
    if ~isempty(fw_p)
        group_fw_ctrl.add(px4io.parameter.matrix(fw_p.PositionProportionalGain).set_name("SM_F_POS_KP").set_description("Fixedwing track position path reference tracking loop error matrix"));
        group_fw_ctrl.add(px4io.parameter.matrix(fw_p.VelocityProportionalGain).set_name("SM_F_VEL_KP").set_description("Fixedwing translational airspeed velocity tracking proportional loop feedback gain matrix"));
        group_fw_ctrl.add(px4io.parameter.matrix(fw_p.VelocityIntegralGain).set_name("SM_F_VEL_KI").set_description("Fixedwing airspeed track profile integral error variable tracking calculation matrix"));
        group_fw_ctrl.add(px4io.parameter.matrix(fw_p.VelocityDerivativeGain).set_name("SM_F_VEL_KD").set_description("Fixedwing forward kinematic acceleration derivative tracking path compensation error matrix"));
        group_fw_ctrl.add(px4io.parameter.float(fw_p.MaxHorizontalPositionError).set_name("SM_F_ERR_POS_XY").set_unit(px4io.parameter.unit.M).set_description("Maximum allowed horizontal position flight path cross track spatial error drift magnitude"));
        group_fw_ctrl.add(px4io.parameter.float(fw_p.MaxVerticalPositionError).set_name("SM_F_ERR_POS_Z").set_unit(px4io.parameter.unit.M).set_description("Maximum allowed vertical glidepath glide altitude tracking error window bands"));
        group_fw_ctrl.add(px4io.parameter.float(fw_p.MaxHorizontalSpeed).set_name("SM_F_SPD_XY_MAX").set_unit(px4io.parameter.unit.M_S).set_description("Maximum cruise forward target ground speed profile configuration velocity thresholds"));
        group_fw_ctrl.add(px4io.parameter.float(fw_p.MinMaxVerticalSpeed).set_name("SM_F_SPD_Z_LIM").set_instances(2,1).set_unit(px4io.parameter.unit.M_S).set_description("Minimum and Maximum targeted cruise climb rate and dive vertical velocity tracking limits"));
        group_fw_ctrl.add(px4io.parameter.float(fw_p.MaxHorizontalVelocityError).set_name("SM_F_ERR_VEL_XY").set_unit(px4io.parameter.unit.M_S).set_description("Maximum horizontal cruising speed path deviation profile error tracking saturation threshold caps"));
        group_fw_ctrl.add(px4io.parameter.float(fw_p.MaxVerticalVelocityError).set_name("SM_F_ERR_VEL_Z").set_unit(px4io.parameter.unit.M_S).set_description("Maximum vertical climb rate path tracking reference error saturation threshold envelope limits"));
        group_fw_ctrl.add(px4io.parameter.float(fw_p.MaxHorizontalAccelerationError).set_name("SM_F_ERR_ACC_XY").set_unit(px4io.parameter.unit.M_S2).set_description("Maximum forward horizontal acceleration path planning navigation error vector profile boundaries"));
        group_fw_ctrl.add(px4io.parameter.float(fw_p.MaxVerticalAccelerationError).set_name("SM_F_ERR_ACC_Z").set_unit(px4io.parameter.unit.M_S2).set_description("Maximum pitch pull up or push over climb acceleration reference path deviation error saturation envelopes"));
        group_fw_ctrl.add(px4io.parameter.float(fw_p.MaxTotalAcceleration).set_name("SM_F_ACC_MAX_G").set_unit(px4io.parameter.unit.NORM).set_description("Max structural loading acceleration profile saturation limits specified in multiples of gravity"));
        group_fw_ctrl.add(px4io.parameter.float(fw_p.MinThrustAcceleration).set_name("SM_F_ACC_MIN_G").set_unit(px4io.parameter.unit.NORM).set_description("Min forward thrust speed profile acceleration tracking bounds specified in multiples of gravity"));
        group_fw_ctrl.add(px4io.parameter.float(fw_p.MaxRollAngle).set_name("SM_F_ROLL_MAX").set_unit(px4io.parameter.unit.RAD).set_description("Absolute hard maximum coordinated banking flight roll angle constraint limits"));
        group_fw_ctrl.add(px4io.parameter.float(fw_p.MinMaxAngleOfAttack).set_instances(2,1).set_unit(px4io.parameter.unit.RAD).set_description("Minimum and Maximum legal aerodynamic angles of attack tracking parameters"));
        group_fw_ctrl.add(px4io.parameter.float(fw_p.MaxLoadFactor).set_name("SM_F_LOAD_MAX_G").set_unit(px4io.parameter.unit.NORM).set_description("Maximum load factor feedback limit parameter for elevator pitching loops specified in multiples of gravity"));
        group_fw_ctrl.add(px4io.parameter.float(fw_p.MaxPitchAngle).set_unit(px4io.parameter.unit.RAD).set_name("SM_F_PITCH_MAX").set_description("Maximum airframe longitudinal pitching attitude climb/dive angle flight envelope orientation constraints"));
    end

    % FW Inner Attitude Control Parameters
    fw_a = [];
    if isequal(class(Parameters.Vehicle.ControlSystem.Attitude), 'Airframe.Control.Attitude.FixedWing')
        fw_a = Parameters.Vehicle.ControlSystem.Attitude;
    elseif isequal(class(Parameters.Vehicle.ControlSystem.Attitude), 'Airframe.Control.Attitude.Tiltwing')
        fw_a = Parameters.Vehicle.ControlSystem.Attitude.Cruise;
    end
    if ~isempty(fw_a)
        group_fw_ctrl.add(px4io.parameter.matrix(fw_a.AttitudeGain).set_name("SM_F_ATT_KP").set_description("Fixedwing orientation quaternion tracking proportional loop feedback matrix"));
        group_fw_ctrl.add(px4io.parameter.matrix(fw_a.AngularRateGain).set_name("SM_F_RAT_KP").set_description("Fixedwing body axis angular rate proportional control feedback loop matrix"));
        group_fw_ctrl.add(px4io.parameter.matrix(fw_a.AngularRateIntegralGain).set_name("SM_F_RAT_KI").set_description("Fixedwing body axis rates error integral state tracker feedback scaling matrix"));
        group_fw_ctrl.add(px4io.parameter.matrix(fw_a.AngularAccelerationGain).set_name("SM_F_ACC_KP").set_description("Fixedwing cross coupling angular acceleration loop proportional gain matrix"));
        group_fw_ctrl.add(px4io.parameter.float(fw_a.MaxAttitudeError).set_name("SM_F_ERR_ATT_LIM").set_unit(px4io.parameter.unit.RAD).set_description("Maximum rotation tracking angle error envelope saturation limit before gain steps"));
        group_fw_ctrl.add(px4io.parameter.float(fw_a.MaxAngularRateError).set_name("SM_F_ERR_RAT_LIM").set_unit(px4io.parameter.unit.RAD_S).set_description("Maximum cross-axis fixedwing angular velocity rate tracking error saturation boundaries"));
        group_fw_ctrl.add(px4io.parameter.float(fw_a.MaxAngularAccelerationError).set_name("SM_F_ERR_AAC_LIM").set_unit(px4io.parameter.unit.RAD_S2).set_description("Maximum angular reference tracking frame acceleration path deviation error saturation threshold bounds"));
        group_fw_ctrl.add(px4io.parameter.float(fw_a.MaxRateLimit).set_name("SM_F_RAT_REF_MAX").set_unit(px4io.parameter.unit.RAD_S).set_description("Absolute hard maximum target body rate setpoint reference magnitude commanded via stability loops"));
        master_module.add_group(group_fw_ctrl);
    end
        
    master_module.to_file(export_destination_path);
end