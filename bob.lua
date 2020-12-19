function __getObjectPosition__(a,b)
    -- compatibility routine, wrong results could be returned in some situations, in CoppeliaSim <4.0.1
    if b==sim.handle_parent then
        b=sim.getObjectParent(a)
    end
    if (b~=-1) and (sim.getObjectType(b)==sim.object_joint_type) and (sim.getInt32Parameter(sim.intparam_program_version)>=40001) then
        a=a+sim.handleflag_reljointbaseframe
    end
    return sim.getObjectPosition(a,b)
end
function __getObjectQuaternion__(a,b)
    -- compatibility routine, wrong results could be returned in some situations, in CoppeliaSim <4.0.1
    if b==sim.handle_parent then
        b=sim.getObjectParent(a)
    end
    if (b~=-1) and (sim.getObjectType(b)==sim.object_joint_type) and (sim.getInt32Parameter(sim.intparam_program_version)>=40001) then
        a=a+sim.handleflag_reljointbaseframe
    end
    return sim.getObjectQuaternion(a,b)
end
if (sim_call_type==sim.syscb_init) then

	
    RightmotorHandle = sim.getObjectHandle('Bob_leftMotor')
    LeftmotorHandle =  sim.getObjectHandle('Bob_rightMotor')
    w = 0
    vx = 0

    RobotHandle =  sim.getObjectHandle('quad_handle')  
    OdomHandle =  sim.getObjectHandle('odom')  

    odomPub=simROS.advertise('/odom', 'nav_msgs/Odometry')
end    



if (sim_call_type==sim.syscb_actuation) then
    velSub=simROS.subscribe('/cmd_vel', 'geometry_msgs/Twist', 'cmd_vel_callback')

end

function cmd_vel_callback(msg)
    vx= msg.linear.x;
    w=  msg.angular.z;

    -- Base_controller --
    r = 1.0000e-01 -- (m) wheel radius
    L = 0.25    
           
    Vright = - ((w*L)/(2*r)) + (vx/r)
    Vleft =  ((w*L)/(2*r)) + (vx/r)
    sim.setJointTargetVelocity(LeftmotorHandle,Vleft)
    sim.setJointTargetVelocity(RightmotorHandle,Vright)
end

function getTransformStamped(objHandle,name,relTo,relToName)
    t=sim.getSystemTime()
    p=__getObjectPosition__(objHandle,relTo)
    o=__getObjectQuaternion__(objHandle,relTo)
    linearVelocity, angularVelocity = sim.getObjectVelocity(objHandle)
    return {
        header={
            stamp=t,
            frame_id=relToName
        },
        child_frame_id=name,
        pose = {
            pose = {
                position = {
                x = p[0],
                y = p[1],
                z = p[2]
                },
                orientation = {
                    x = o[1],
                    y = o[2],
                    z = o[3],
                    w = o[4],
                },
            }
        },
        twist = {
            twist = {
                linear = {
                x = linearVelocity[0],
                y = linearVelocity[1],
                z = linearVelocity[2]
                },
                angular = {
                    x = angularVelocity[0],
                    y = angularVelocity[1],
                    z = angularVelocity[2]
                },
            }
        },
    }
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
