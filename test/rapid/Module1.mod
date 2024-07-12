MODULE Module1
        CONST robtarget Target_10:=[[499.999992196,-150.00001257,624.999987626],[0.707106781,0.000000013,0.707106781,-0.00000001],[-1,-1,0,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_20:=[[499.99999087,-248.387690305,730.747856915],[0.686438597,0.16971169,0.686438612,-0.169711601],[-1,0,-1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_30:=[[499.999991024,166.254404463,624.999979073],[0.707106773,0,0.70710679,-0.000000005],[0,0,-1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_40:=[[499.999995076,166.254390858,806.452197896],[0.695967512,0.125016908,0.695967503,-0.125016918],[0,0,-1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    LOCAL VAR egmident egmID1;
    LOCAL VAR egmstate egmSt1;
    
    PERS string test_var_str:="";
    PERS num test_var_num:=123.456;
    CONST robtarget Target_50:=[[499.999989972,-150.000098106,550.999889231],[0.000000153,0.000000278,1,-0.000000087],[-1,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_60:=[[499.999996372,-15.000099928,449.999890949],[0.000000007,-0.000000322,1,-0.000000031],[-1,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_70:=[[500.000116532,199.999895306,579.999910737],[0.000000292,-0.000000855,1,0.000000075],[0,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    PERS pose sensor_frame:= [[0,0,0],[1,0,0,0]];
    
    

    PROC main()
        !Add your code here
        StartEGMStreaming(mode);
        
        IF mode = 0 THEN
            Path_10;
        ELSEIF mode = 1 THEN
            RunEGMJointCtrl;
        ELSEIF mode = 2 THEN
            RunEGMPoseCtrl;
        ELSEIF mode = 3 THEN
            Path_EGM;
        ENDIF
            
    ENDPROC
    PROC Path_10()
        MoveJ Target_10,v1000,fine,tool0\WObj:=wobj0;
        MoveL Target_20,v1000,z100,tool0\WObj:=wobj0;
        MoveL Target_30,v1000,z100,tool0\WObj:=wobj0;
        MoveL Target_40,v1000,z100,tool0\WObj:=wobj0;
    ENDPROC
    PROC StartEGMStreaming(num mode)
        egmSt1:=EGMGetState(egmID1);
        IF egmSt1 = EGM_STATE_RUNNING THEN
            TPWrite "EGM already running";
            RETURN;
        ENDIF
        TPWrite "Starting EGM";
        EGMReset egmID1;
        WaitTime 0.005;
        EGMGetId egmID1;
        egmSt1:=EGMGetState(egmID1);
        IF mode >=0 AND MODE <=1 THEN
            EGMSetupUC ROB_1,egmID1,"default","UCdevice"\Joint\CommTimeout:=1000000;
        ELSEIF mode = 2 THEN
            EGMSetupUC ROB_1,egmID1,"default","UCdevice"\Pose\CommTimeout:=1000000;
        ELSEIF mode = 3 THEN
            EGMSetupUC ROB_1,egmID1,"pathcorr","UCdevice"\PathCorr\APTR\CommTimeout:=1000000;
        ENDIF
        EGMStreamStop egmID1;
        WaitTime 0.5;
        EGMStreamStart egmID1\SampleRate:=4;
    ENDPROC
    PROC Path_EGM()
        EGMActMove egmID1, sensor_frame\SampleRate:=24;
        MoveJ Target_50,v1000,fine,tool0\WObj:=wobj0;
        EGMMoveL egmID1,Target_60,v10,z100,tool0\WObj:=wobj0;
        EGMMoveL egmID1,Target_70,v10,fine,tool0\WObj:=wobj0;
    ENDPROC
    PROC RunEGMJointCtrl()
        VAR egm_minmax minmax:= [-1e9,1e9];
        MoveJ Target_10,v1000,fine,tool0\WObj:=wobj0;

        EGMActJoint egmID1,\Tool:=tool0\J1:=minmax
        \J2:=minmax\J3:=minmax\J4:=minmax\J5:=minmax\J6:=minmax\MaxPosDeviation:=1000\MaxSpeedDeviation:=1000;
        SetDO stop_egm,0;
        EGMRunJoint egmID1,EGM_STOP_HOLD,\NoWaitCond\J1\J2\J3\J4\J5\J6\CondTime:=1e9\RampInTime:=1;
        WaitDO stop_egm,1;
        EGMStop egmID1,EGM_STOP_HOLD\RampOutTime:=1;
    ENDPROC
    PROC RunEGMPoseCtrl()
        VAR egm_minmax minmax:= [-1e9,1e9];
        MoveJ Target_10,v1000,fine,tool0\WObj:=wobj0;

        EGMActPose egmID1\Tool:=tool0,sensor_frame,EGM_FRAME_WOBJ,sensor_frame,EGM_FRAME_WOBJ,\MaxPosDeviation:=1000\MaxSpeedDeviation:=1000;
        SetDO stop_egm,0;
        EGMRunPose egmID1,EGM_STOP_HOLD,\NoWaitCond\x\y\z\rx\ry\rz;
        WaitDO stop_egm,1;
        EGMStop egmID1,EGM_STOP_HOLD\RampOutTime:=1;
    ENDPROC
ENDMODULE
