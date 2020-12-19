function __getObjectOrientation__(a,b)
    -- compatibility routine, wrong results could be returned in some situations, in CoppeliaSim <4.0.1
    if b==sim.handle_parent then
        b=sim.getObjectParent(a)
    end
    if (b~=-1) and (sim.getObjectType(b)==sim.object_joint_type) and (sim.getInt32Parameter(sim.intparam_program_version)>=40001) then
        a=a+sim.handleflag_reljointbaseframe
    end
    return sim.getObjectOrientation(a,b)
end

if (sim_call_type==sim.syscb_init) then 

    -- Make sure we have VREP version 2.4.13 or above (the particles are not supported otherwise)
    v=sim.getInt32Parameter(sim.intparam_program_version)
    if (v<20413) then
        sim.displayDialog('Warning','The propeller model is only fully supported from V-REP version 2.4.13 and above.&&nThis simulation will not run as expected!',sim.dlgstyle_ok,false,'',nil,{0.8,0,0,0,0,0})
    end

    -- Detatch the manipulation sphere:
    target_obj = sim.getObjectHandle('Quadricopter_target')
    sim.setObjectParent(target_obj,-1,true)

    -- I modified the original quuadrotor control to receive Twist commands from ROS, feel free to play with the parameters.

    quad_handle = sim.getObjectHandle('Quadricopter_base')

    particlesAreVisible=sim.getScriptSimulationParameter(sim.handle_self,'particlesAreVisible')
    sim.setScriptSimulationParameter(sim.handle_tree,'particlesAreVisible',tostring(particlesAreVisible))
    simulateParticles=sim.getScriptSimulationParameter(sim.handle_self,'simulateParticles')
    sim.setScriptSimulationParameter(sim.handle_tree,'simulateParticles',tostring(simulateParticles))

    propellerScripts={-1,-1,-1,-1}
    for i=1,4,1 do
        propellerScripts[i]=sim.getScriptHandle('Quadricopter_propeller_respondable'..i)
    end
    
    particlesTargetVelocities={0,0,0,0}
    
    old_velx = 0
    old_vely = 0
    ref_velx = 0
    ref_vely = 0
    old_vel_angz = 0

    pParam=2
    iParam=0.001
    dParam=0.001
    vParam=-2

    cumul=0
    lastE=0
    pAlphaE=0
    pBetaE=0
    psp2=0
    psp1=0
    height = 1
    ref_angz = 0

    prevEuler=0

    fakeShadow=sim.getScriptSimulationParameter(sim.handle_self,'fakeShadow')
    if (fakeShadow) then
        shadowCont=sim.addDrawingObject(sim.drawing_discpoints+sim.drawing_cyclic+sim.drawing_25percenttransparency+sim.drawing_50percenttransparency+sim.drawing_itemsizes,0.2,0,-1,1)
    end

    -- Prepare 2 floating views with the camera views:
    floorCam=sim.getObjectHandle('Quadricopter_floorCamera')
    frontCam=sim.getObjectHandle('Quadricopter_frontCamera')
    floorView=sim.floatingViewAdd(0.9,0.9,0.2,0.2,0)
    frontView=sim.floatingViewAdd(0.7,0.9,0.2,0.2,0)
    sim.adjustView(floorView,floorCam,64)
    sim.adjustView(frontView,frontCam,64)
end 

if (sim_call_type==sim.syscb_cleanup) then 
end 

if (sim_call_type==sim.syscb_sensing) then 
    --Odometry Covariance matrix
    odomcovariance={
        0.001,0,0,0,0,0,
        0,0.001,0,0,0,0,
        0,0,0.001,0,0,0,
        0,0,0,0.001,0,0,
        0,0,0,0,0.001,0,
        0,0,0,0,0,0.001,
        0.001,0,0,0,0,0,
        0,0.001,0,0,0,0,
        0,0,0.001,0,0,0,
        0,0,0,0.001,0,0,
        0,0,0,0,0.001,0,
        0,0,0,0,0,0.001}
   OdomCovar=sim.packFloatTable(odomcovariance)
   sim.setStringSignal('OdomCovariance',OdomCovar)
       
   -- Odom publisher
   simROS.publish(odomPub, getTransformStamped(RobotHandle, 'RobotHandle', OdomHandle, 'OdomHandle'))
end 

if (sim_call_type==sim.syscb_actuation) then
    velSub=simROS.subscribe('/cmd_vel', 'geometry_msgs/Twist', 'cmd_vel_callback')

end 

function cmd_vel_callback(msg)
    -- Process the twist data
    -- X and Y velocities (related to the heading of the quadrotor)
    ref_velx = msg.linear.x;
    ref_vely = msg.linear.y;

    -- NOTE: twist.linear.z component its not de velocity in Z axis but a height reference.
    -- This is just for convenience. Feel free to modify it.
    height = 1 + msg.linear.z;

    -- Angular velocity in Z (this changes the heading of the quadrotor)
    ref_angz = msg.angular.z;
end

if (sim_call_type==sim.syscb_cleanup) then 
    sim.removeDrawingObject(shadowCont)
    sim.floatingViewRemove(floorView)
    sim.floatingViewRemove(frontView)
end 

if (sim_call_type==sim.syscb_actuation) then 
    s=sim.getObjectSizeFactor(quad_handle)
    pos_quad=sim.getObjectPosition(quad_handle,-1)    
    if (fakeShadow) then
        itemData={pos_quad[1],pos_quad[2],0.002,0,0,1,0.2*s}
        sim.addDrawingObjectItem(shadowCont,itemData)
    end

    -- Quadcopter Global Velocity
    vel_quad_global, vel_ang = sim.getObjectVelocity(quad_handle)

    -- Transformation to obtain velocities related to the quadrotor heading
    --------------------------------------------------------------------------
    -- Orientation of the quadcopter related to the World
    eulerQuad = sim.getObjectOrientation(quad_handle,-1)

    -- We create a Transformation Matrix centered at the quadrotor and rotated only in Z
    eulerHeading = eulerQuad
    eulerHeading[1] = 0
    eulerHeading[2] = 0
    m_heading = sim.buildMatrix(pos_quad,eulerHeading)

    -- We create a Transformation Matrix for converting the global referenced velocities
    -- to a reference frame based on the heading of the quadrotor
    global_pos = {0,0,0}
    m_global = sim.buildMatrix(global_pos,eulerHeading)
    m = simGetInvertedMatrix(m_global)
    vel_quad_local=sim.multiplyVector(m,vel_quad_global)

    -- We calculate the attitude of the Quadrotor in the frame based on the heading 
    m_quad=sim.getObjectMatrix(quad_handle,-1)
    m_heading_inv = simGetInvertedMatrix(m_heading)
    m = sim.multiplyMatrices(m_heading_inv,m_quad)
    euler = sim.getEulerAnglesFromMatrix(m)

    -- Vertical control (from previous controler)
    e=(height-pos_quad[3]) 
    cumul=cumul+e
    pv=pParam*e
    thrust=5.335+pv+iParam*cumul+dParam*(e-lastE)+vel_quad_local[3]*vParam
    lastE=e
    
    -- Horizontal control 
    vel_errorx = ref_velx - vel_quad_local[1]
    vel_errory = ref_vely - vel_quad_local[2]
    
    -- Error in Alpha angle (simple PD controler)
    alphaE = euler[1]
    alphaCorr=0.25*alphaE+2.1*(alphaE-pAlphaE)
    
    -- Error in Beta angle (simple PD controler)
    betaE = -euler[2]
    betaCorr=-0.25*betaE-2.1*(betaE-pBetaE)
    pAlphaE=alphaE
    pBetaE=betaE

    -- We add the Velocity reference controller to the previous attitude control
    -- also a simple PD controler
    alphaCorr=alphaCorr+vel_errory*0.05 + (vel_quad_local[2]-old_vely)*0.05
    betaCorr=betaCorr-vel_errorx*0.05 - (vel_quad_local[1]-old_velx)*0.05
    old_velx = vel_quad_local[1]
    old_vely = vel_quad_local[2]

    -- Rotational control
    --print('Rot Velocities', ref_angz, vel_ang[3])
    vel_angz_error = ref_angz - vel_ang[3]
    rotCorr= -vel_angz_error*1.0 - (vel_ang[3]-old_vel_angz)*1.0
    old_vel_angz = vel_ang[3]

    -- CONSTANT HEADING Rotational control. Just for testing.
    --euler=__getObjectOrientation__(quad_handle,target_obj)
    --rotCorr=euler[3]*0.1+2*(euler[3]-prevEuler)
    --prevEuler=euler[3]
    
    -- Decide the motor velocities:
    particlesTargetVelocities[1]=thrust*(1-alphaCorr+betaCorr+rotCorr)
    particlesTargetVelocities[2]=thrust*(1-alphaCorr-betaCorr-rotCorr)
    particlesTargetVelocities[3]=thrust*(1+alphaCorr-betaCorr+rotCorr)
    particlesTargetVelocities[4]=thrust*(1+alphaCorr+betaCorr-rotCorr)

    -- Move Target Object (Green Sphere in the simulation)
    -- Just for visualization of the Reference Height assigned to the Quadrotor.
    pos_target=sim.getObjectPosition(target_obj,-1)
    pos_target[1] = pos_quad[1]
    pos_target[2] = pos_quad[2]
    pos_target[3] = height
    sim.setObjectPosition(target_obj,-1,pos_target)
    
    -- Send the desired motor velocities to the 4 rotors:
    for i=1,4,1 do
        sim.setScriptSimulationParameter(propellerScripts[i],'particleVelocity',particlesTargetVelocities[i])
    end
end 