function sysCall_threadmain()
    nominalVel = 0.25
    nominalAcc = 0.5

    
    ik1=sim.getIkGroupHandle('IRB140_undamped')
    target=sim.getObjectHandle("IRB140_target")
    tip=sim.getObjectHandle("IRB140_tip")

    armJoints={-1,-1,-1,-1,-1,-1}
    for i=1,6,1 do
        armJoints[i]=sim.getObjectHandle('IRB140_joint'..i)
    end
    
    pickup_path_pointer=sim.generateIkPath(ik1,armJoints,2)
    
    pickup_path=sim.createPath(1)
    sim.insertPathCtrlPoints(pickup_path,0,0,2,pickup_path_pointer)

    pick_path_handle=sim.getObjectHandle('pickup_path')
    sim.followPath(target,pick_path_handle,3,0,nominalVel,nominalAcc)

    

