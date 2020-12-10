-- This is a threaded script, and is just an example!

enableIk=function(enable)
    if enable then
        sim.setObjectMatrix(ikTarget,-1,sim.getObjectMatrix(ikTip,-1))
        for i=1,#jointHandles,1 do
            sim.setJointMode(jointHandles[i],sim.jointmode_ik,1)
        end

        sim.setExplicitHandling(ikGroupHandle,0)
    else
        sim.setExplicitHandling(ikGroupHandle,1)
        for i=1,#jointHandles,1 do
            sim.setJointMode(jointHandles[i],sim.jointmode_force,0)
        end
    end
end

setGripperData=function(open,velocity,force)
    if not velocity then
        velocity=0.11
    end
    if not force then
        force=20
    end
    if not open then
        velocity=-velocity
    end
    local data=sim.packFloatTable({velocity,force})
    sim.setStringSignal(modelName..'_rg2GripperData',data)
end

function sysCall_threadmain()
    -- Initialize some values:
    jointHandles={-1,-1,-1,-1,-1,-1}
    for i=1,6,1 do
        jointHandles[i]=sim.getObjectHandle('UR5_joint'..i)
    end
    ikGroupHandle=sim.getIkGroupHandle('UR5')
    ikTip=sim.getObjectHandle('UR5_ikTip')
    ikTarget=sim.getObjectHandle('UR5_ikTarget')
    modelBase=sim.getObjectAssociatedWithScript(sim.handle_self)
    modelName=sim.getObjectName(modelBase)

    -- Set-up some of the RML vectors:
    vel=180
    accel=40
    jerk=80
    currentVel={0,0,0,0,0,0,0}
    currentAccel={0,0,0,0,0,0,0}
    maxVel={vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180}
    maxAccel={accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180}
    maxJerk={jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180}
    targetVel={0,0,0,0,0,0}

    ikMaxVel={0.4,0.4,0.4,1.8}
    ikMaxAccel={0.8,0.8,0.8,0.9}
    ikMaxJerk={0.6,0.6,0.6,0.8}

    initialConfig={0,0,0,0,0,0}
    pickConfig={-70.1*math.pi/180,18.85*math.pi/180,93.18*math.pi/180,68.02*math.pi/180,109.9*math.pi/180,90*math.pi/180}
    dropConfig1={-183.34*math.pi/180,14.76*math.pi/180,78.26*math.pi/180,-2.98*math.pi/180,-90.02*math.pi/180,86.63*math.pi/180}
    dropConfig2={-197.6*math.pi/180,14.76*math.pi/180,78.26*math.pi/180,-2.98*math.pi/180,-90.02*math.pi/180,72.38*math.pi/180}
    dropConfig3={-192.1*math.pi/180,3.76*math.pi/180,91.16*math.pi/180,-4.9*math.pi/180,-90.02*math.pi/180,-12.13*math.pi/180}
    dropConfig4={-189.38*math.pi/180,24.94*math.pi/180,64.36*math.pi/180,0.75*math.pi/180,-90.02*math.pi/180,-9.41*math.pi/180}

    dropConfigs={dropConfig1,dropConfig2,dropConfig3,dropConfig4}
    dropConfigIndex=1
    droppedPartsCnt=0

    enableIk(false)
    setGripperData(true)
    sim.setInt32Parameter(sim.intparam_current_page,0)

    while droppedPartsCnt<6 do

        if sim.getSimulationState()~=sim.simulation_advancing_abouttostop then
            print("1")
            sim.rmlMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,pickConfig,targetVel)
        end

        if sim.getSimulationState()~=sim.simulation_advancing_abouttostop then
            enableIk(true)
            print("2")
            sim.setInt32Parameter(sim.intparam_current_page,1)


            pos=sim.getObjectPosition(ikTip,-1)
            quat=sim.getObjectQuaternion(ikTip,-1)
            sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{pos[1]+0.105,pos[2],pos[3]},quat,nil)
        end

        if sim.getSimulationState()~=sim.simulation_advancing_abouttostop then
            print("3")
            setGripperData(false)
            sim.wait(0.5)
        end

        if sim.getSimulationState()~=sim.simulation_advancing_abouttostop then
            print("4")
            sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{pos[1],pos[2]-0.2,pos[3]+0.2},quat,nil)
        end

        if sim.getSimulationState()~=sim.simulation_advancing_abouttostop then
            print("5")
            enableIk(false)
            sim.setInt32Parameter(sim.intparam_current_page,0)

            sim.rmlMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,dropConfigs[dropConfigIndex],targetVel)
        end

        if sim.getSimulationState()~=sim.simulation_advancing_abouttostop then
            print("6")
            sim.setInt32Parameter(sim.intparam_current_page,2)
            enableIk(true)
            pos=sim.getObjectPosition(ikTip,-1)
            quat=sim.getObjectQuaternion(ikTip,-1)
            sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{pos[1],pos[2],0.025+0.05*math.floor(0.1+droppedPartsCnt/2)},quat,nil)
        end

        if sim.getSimulationState()~=sim.simulation_advancing_abouttostop then
            print("7")
            setGripperData(true)
            sim.wait(0.5)
        end

        if sim.getSimulationState()~=sim.simulation_advancing_abouttostop then
            print("8")
            sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,pos,quat,nil)
        end

        if sim.getSimulationState()~=sim.simulation_advancing_abouttostop then
            enableIk(false)
            print("9")  
            sim.setInt32Parameter(sim.intparam_current_page,0)

            dropConfigIndex=dropConfigIndex+1
            if dropConfigIndex>4 then
                dropConfigIndex=1
            end

            droppedPartsCnt=droppedPartsCnt+1
        end
    end

    sim.rmlMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,initialConfig,targetVel)
    sim.stopSimulation()
end
    
