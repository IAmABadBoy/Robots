infoOfPlatformSensors=function()

	handleSensorPlatform = {}

	handleSensorPlatform[3] = robot.simGetHandleProximitySensorPlatformLeft()

	handleSensorPlatform[4] = robot.simGetHandleProximitySensorPlatformRight()

	handleSensorPlatform[1] = robot.simGetHandleProximitySensorPlatformFront()

	handleSensorPlatform[2] = robot.simGetHandleProximitySensorPlatformBack()

	info = {{},{},{},{}}

	detectedPoint={0,0,0}

	detectedSurfaceNormalVector={0,0,0}

	furniturePos={0,0,0}

	GasTankPos={0,0,0}

	result,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector,distance = robot.simGetProximitySensorResult(handleSensorPlatform[1],detectedPoint,detectedSurfaceNormalVector)

	for i=1,4 do

		result,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector,distance = robot.simGetProximitySensorResult(handleSensorPlatform[i],detectedPoint,detectedSurfaceNormalVector)

		info[i].result = result

		info[i].distance = distance

		info[i].detectedPoint = detectedPoint

		info[i].detectedObjectHandle = detectedObjectHandle

		if(info[i].result > 0 ) then

				if(robot.ifHandleIsTask2Obj(info[i].detectedObjectHandle)) then

					info.Task2Obj = true

					info.furniturePos = robot.simGetObjectPosition(detectedObjectHandle, furniturePos)

				else

					info.Task2Obj = false

					info.furniturePos=nil

				end

				if(robot.ifHandleIsGasTank(info[i].detectedObjectHandle)) then

					info.Task4Obj = true

					info.GasTankPos = robot.simGetObjectPosition(detectedObjectHandle, GasTankPos)



				else

					info.Task4Obj = false

					info.GasTankPos=nil

				end

			end

		info[i].detectedSurfaceNormalVector = detectedSurfaceNormalVector

	end

	return info

end 
PosofCircle=function(pos,rot,DestinPos,DestinVRot)
    if(DestinPos==nil or DestinVRot==nil)then
        return nil
    end
	PosofO={0,0,0}
	--AB方向与全局坐标系X轴夹角，范围（-pi，pi）
	AtoBAngle=math.atan2(DestinPos[2]-pos[2],DestinPos[1]-pos[1])
	if(AtoBAngle==DestinVRot)
	then
		return nil--B的速度方向为AB直线方向，返回nil
	end

    midpoint={(pos[1]+DestinPos[1])/2,(pos[2]+DestinPos[2])/2}
    AngleofMidLine=AtoBAngle+math.pi*0.5
   -- simExtPrintInfo(tostring(math.deg(AngleofMidLine)))
    if(math.abs(math.abs(AngleofMidLine)%math.pi-math.pi*0.5)<0.0001)then
        slope1=nil
       -- simExtPrintInfo("1    ")
    else
        slope1=math.tan(AngleofMidLine)
       -- simExtPrintInfo("2    ")
    end
    AngleofBO=DestinVRot+math.pi*0.5--此方向为全局坐标系下方向
		--  simExtPrintInfo("AngleofBO")
		   --simExtPrintInfo(tostring(AngleofBO))
        if(math.abs(math.abs(AngleofBO)%math.pi-math.pi*0.5)<0.0001)then
            slope3=nil
         --   simExtPrintInfo("1    ")
        else
            slope3=math.tan(AngleofBO)
        --    simExtPrintInfo("2    ")
        end
      -- simExtPrintInfo(tostring(slope3))
        if(slope1==nil and slope3~=nil)then
            b3=DestinPos[2]-DestinPos[1]*slope3--BO的直线方程
            PosofO[1]=midpoint[1]
            PosofO[2]=slope3*midpoint[1]+b3
            PosofO[3]=pos[3]
        else if(slope3==nil and slope1~=nil)then
                b1=midpoint[2]-midpoint[1]*slope1--AB垂直平分线的直线方程
                PosofO[1]=DestinPos[1]
                PosofO[2]=slope1*DestinPos[1]+b1
                PosofO[3]=pos[3]
            else if(slope1==nil and slope3==nil)then
					return nil--B的速度方向为AB直线方向，返回nil
				else
					b3=DestinPos[2]-DestinPos[1]*slope3--BO的直线方程
					b1=midpoint[2]-midpoint[1]*slope1--AB垂直平分线的直线方程
					PosofO[1]=(b3-b1)/(slope1-slope3)
					PosofO[2]=slope1*PosofO[1]+b1
					PosofO[3]=pos[3]
				end
            end
        end
        return PosofO
end
GetDeltaAngle=function(Angle1,Angle2)
deltaAngle=Angle1-Angle2

	if(Angle1>Angle2)then

		if(Angle2>0)then

			deltaAngle=Angle1-Angle2

		else if(Angle1<0)then

				deltaAngle=Angle1-Angle2

			else if(Angle1-Angle2<math.pi)then

				deltaAngle=Angle1-Angle2

			else

				deltaAngle=deltaAngle-math.pi*2
			end
		end
		end
	else if(Angle1>0)then

		deltaAngle=Angle1-Angle2

		else if(Angle2<0)then

			deltaAngle=Angle1-Angle2

			else if(Angle1-Angle2>-math.pi)then

				deltaAngle=Angle1-Angle2

				else

				deltaAngle=deltaAngle+2*math.pi
				end

			end
		end
end
	return -deltaAngle
end
GetTofVehicle=function (pos,rot,PosofO,DestinPos,RotofB,V )
    T={0,0,0}
	if(PosofO~=nil)then
		AtoOAngle=math.atan2(PosofO[2]-pos[2],PosofO[1]-pos[1])
		AtoBAngle=math.atan2(DestinPos[2]-pos[2],DestinPos[1]-pos[1])
		BtoOAngle=math.atan2(-DestinPos[2]+PosofO[2],-DestinPos[1]+PosofO[1])
		if(GetDeltaAngle(BtoOAngle,DestinVRot)==math.pi*0.5)then
		
			VehicleRos=AtoOAngle+math.pi*0.5--车子速度方向
		else
			VehicleRos=AtoOAngle-math.pi*0.5--车子速度方向
		end
		T[1]=V*math.cos(VehicleRos)
		T[2]=V*math.sin(VehicleRos)
		if(DestinPos~=nil)then
			OtoBAngle=math.atan2(DestinPos[2]-PosofO[2],DestinPos[1]-PosofO[1])
			OtoAAngle=math.atan2(pos[2]-PosofO[2],pos[1]-PosofO[1])
			Angle=GetDeltaAngle(OtoBAngle,OtoAAngle)
			R=math.sqrt((DestinPos[2]-PosofO[2])*(DestinPos[2]-PosofO[2])+(DestinPos[1]-PosofO[1])*(DestinPos[1]-PosofO[1]))
			delta_T=math.abs(Angle/(V/R))
			if(RotofB==nil or math.abs(GetDeltaAngle(RotofB[3],rot[3]))<0.05)then
				T[3]=0
			else
				T[3]=GetDeltaAngle(RotofB[3],rot[3])/delta_T--自转角速度
			end

		else
			R=math.sqrt((pos[2]-PosofO[2])*(pos[2]-PosofO[2])+(pos[1]-PosofO[1])*(pos[1]-PosofO[1]))

			T[3]=V/R

		end
	else
	--1 无目标有角速度

	if(DestinPos==nil)then
		T[1]=0
		T[2]=0
		T[3]=GetDeltaAngle(RotofB[3],rot[3])/math.abs(GetDeltaAngle(RotofB[3],rot[3]))*V--自转角速度
	else		
		AtoBAngle=math.atan2(DestinPos[2]-pos[2],DestinPos[1]-pos[1])
		T[1]=V*math.cos(AtoBAngle)
		T[2]=V*math.sin(AtoBAngle)
		distance=math.sqrt((DestinPos[2]-pos[2])*(DestinPos[2]-pos[2])+(DestinPos[1]-pos[1])*(DestinPos[1]-pos[1]))
		delta_T=math.abs(distance/V)
		if(RotofB==nil)then
				T[3]=0
			else
				T[3]=GetDeltaAngle(RotofB[3],rot[3])/delta_T--自转角速度
			end
	end	
	end
	distance=math.sqrt((DestinPos[1]-pos[1])*(DestinPos[1]-pos[1])+(DestinPos[2]-pos[2])*(DestinPos[2]-pos[2]))
	if(distance<0.05)then
		T[1]=0
		T[2]=0
		T[3]=GetDeltaAngle(RotofB[3],rot[3])/math.abs(GetDeltaAngle(RotofB[3],rot[3]))*V--自转角速度
	end
	return T
end
SetWheelStation=function(pos,rot,T)
    --给轮子顺序编号，顺序为左前，右前，左后，右后
    WheelPos={{0.2444, 0.2346},{0.2444, -0.2365},{-0.2226, 0.2346},{-0.2226,-0.2365}}
    d={0,0,0,0}--轮子到车心的距离
    theta={0,0,0,0}--轮子质心与车身的夹角
    v={0,0,0,0}--各轮子此时此刻的速度
    Psy={0,0,0,0}--各轮子此时与车身正向的夹角
    for i=1,4 do
      d[i]=math.sqrt(WheelPos[i][1]*WheelPos[i][1]+WheelPos[i][2]*WheelPos[i][2])
      theta[i]=math.atan2(WheelPos[i][2],WheelPos[i][1])
      v[i]=math.sqrt((T[1]+T[3]*d[i]*math.cos(theta[i]+rot[3]))*(T[1]+T[3]*d[i]*math.cos(theta[i]+rot[3]))
                        +(T[2]+T[3]*d[i]*math.sin(theta[i]+rot[3]))*(T[2]+T[3]*d[i]*math.sin(theta[i]+rot[3])))
      Psy[i]=math.atan2((T[2]+T[3]*d[i]*math.sin(theta[i]+rot[3])),(T[1]+T[3]*d[i]*math.cos(theta[i]+rot[3])))-(rot[3]+math.pi*0.5)

	end

	
    return v,Psy
end
Motion_module=function(DestinPos,DestinVRot,RotofB,V)

	PlatformSensorsInfo=infoOfPlatformSensors()

    handleTipPlatform = robot.simGetHandleVehicle()

	pos = {0,0,0}

	pos = robot.simGetObjectPosition(handleTipPlatform, pos)

	rot = {0,0,0}

	rot = robot.simGetObjectOrientation(handleTipPlatform,rot)
	CircleCenterPos=PosofCircle(pos,rot,DestinPos,DestinVRot)
    T={0,0,0}

    T=GetTofVehicle(pos,rot,CircleCenterPos,DestinPos,RotofB,V)

	oldAngleOfFourWheels={0,0,0,0}

	oldLinearVelOfFourWheels={0,0,0,0}

    AngleOfFourWheels={0,0,0,0}

    LinearVelOfFourWheels={0,0,0,0}

    LinearVelOfFourWheels,AngleOfFourWheels=SetWheelStation(pos,rot,T)

    robot.simSetJointTargetPosition(handleJointWheelLF0, AngleOfFourWheels[1])

    robot.simSetJointTargetPosition(handleJointWheelRB0, AngleOfFourWheels[4])

    robot.simSetJointTargetPosition(handleJointWheelRF0, AngleOfFourWheels[2])

    robot.simSetJointTargetPosition(handleJointWheelLB0, AngleOfFourWheels[3])



    robot.simSetJointTargetVelocity(handleJointWheelLF, LinearVelOfFourWheels[1])

    robot.simSetJointTargetVelocity(handleJointWheelRF, LinearVelOfFourWheels[2])

    robot.simSetJointTargetVelocity(handleJointWheelLB, LinearVelOfFourWheels[3])

    robot.simSetJointTargetVelocity(handleJointWheelRB, LinearVelOfFourWheels[4])

	 return    PlatformSensorsInfo    
end
Stop=function()
	robot.simSetJointTargetPosition(handleJointWheelLF0, 0)
    robot.simSetJointTargetPosition(handleJointWheelRB0, 0)
    robot.simSetJointTargetPosition(handleJointWheelRF0, 0)
    robot.simSetJointTargetPosition(handleJointWheelLB0, 0)

    robot.simSetJointTargetVelocity(handleJointWheelLF, 0)
    robot.simSetJointTargetVelocity(handleJointWheelRF, 0)
    robot.simSetJointTargetVelocity(handleJointWheelLB, 0)
    robot.simSetJointTargetVelocity(handleJointWheelRB, 0)
end
GotoTagertByCirCle=function(DestinPos,DestinVRot,RotofB,V)
   handleTipPlatform = robot.simGetHandleVehicle()
	pos = {0,0,0}
    pos = robot.simGetObjectPosition(handleTipPlatform, pos)
    handleJointWheelLF0 = robot.simGetHandleJointTurnLeftFront()
    handleJointWheelRB0 = robot.simGetHandleJointTurnRightBack()
    handleJointWheelRF0 = robot.simGetHandleJointTurnRightFront()
    handleJointWheelLB0 = robot.simGetHandleJointTurnLeftBack()
    handleJointWheelLF = robot.simGetHandleJointWheelLeftFront()
    handleJointWheelRF = robot.simGetHandleJointWheelRightFront()
    handleJointWheelLB = robot.simGetHandleJointWheelLeftBack()
    handleJointWheelRB = robot.simGetHandleJointWheelRightBack()
    oldTime = simGetSimulationTime()
    distance=100
    repeat
         newTime=simGetSimulationTime()
        if(newTime-oldTime)then
			pos = {0,0,0}
			pos = robot.simGetObjectPosition(handleTipPlatform, pos)
			PlatformSensorsInfo=Motion_module(DestinPos,DestinVRot,RotofB,V)
            distance=math.sqrt((DestinPos[1]-pos[1])*(DestinPos[1]-pos[1])+(DestinPos[2]-pos[2])*(DestinPos[2]-pos[2]))
            oldTime=newTime
			--if(distance<0.5)then
			--	V=2
			--end
        end
    until(distance<0.05)
	Stop()
	a=0
end
GoToTargetPoint=function(DestinPos,DestinVRot,RotofB,V)
	handleTipPlatform = robot.simGetHandleVehicle()
    pos = {0,0,0}
    pos = robot.simGetObjectPosition(handleTipPlatform, pos)
    handleJointWheelLF0 = robot.simGetHandleJointTurnLeftFront()
    handleJointWheelRB0 = robot.simGetHandleJointTurnRightBack()
    handleJointWheelRF0 = robot.simGetHandleJointTurnRightFront()
    handleJointWheelLB0 = robot.simGetHandleJointTurnLeftBack()
    handleJointWheelLF = robot.simGetHandleJointWheelLeftFront()
    handleJointWheelRF = robot.simGetHandleJointWheelRightFront()
    handleJointWheelLB = robot.simGetHandleJointWheelLeftBack()
    handleJointWheelRB = robot.simGetHandleJointWheelRightBack()
	
	safe_r=0.5
	AngleofBA=math.atan2(pos[2]-DestinPos[2],pos[1]-DestinPos[1])
	AngleofAB=math.atan2(-pos[2]+DestinPos[2],-pos[1]+DestinPos[1])
	if(DestinVRot==nil)then
		DestinVRot=AngleofAB
	end
	AngleofBO=DestinVRot+math.pi*0.5--此方向为全局坐标系下方向
	if(AngleofBO>math.pi)then
		AngleofBO=AngleofBO-math.pi*2
	end
	if(math.abs(AngleofBO-AngleofBA)>math.pi*0.5)then
		AngleofBO=DestinVRot-math.pi*0.5--此方向为全局坐标系下方向
		if(AngleofBO<-math.pi)then
			AngleofBO=math.pi*2+AngleofBO
		end
	end
	
	--确定圆心的方向
	--确定圆心的位置
	PosofC={DestinPos[1]+safe_r*math.cos(AngleofBO),DestinPos[2]+safe_r*math.sin(AngleofBO),DestinPos[3]}
	--选择中间点
	AngleofCD1=AngleofAB+math.pi*0.5
	AngleofCD2=AngleofAB-math.pi*0.5

	PosofD1={PosofC[1]+safe_r*math.cos(AngleofCD1),PosofC[2]+safe_r*math.sin(AngleofCD1),PosofC[3]}
	
	PosofD2={PosofC[1]+safe_r*math.cos(AngleofCD2),PosofC[2]+safe_r*math.sin(AngleofCD2),PosofC[3]}
	
	AngleofD1B=math.atan2(-PosofD1[2]+DestinPos[2],-PosofD1[1]+DestinPos[1])
	AngleofD2B=math.atan2(-PosofD2[2]+DestinPos[2],-PosofD2[1]+DestinPos[1])
	if(math.abs(AngleofD1B-DestinVRot)<math.pi*0.5 or(AngleofD1B-math.pi*1.5>DestinVRot ) or (AngleofD1B+math.pi*1.5-DestinVRot<math.pi*0.5))then
		Target1=PosofD1--{AngleofD1B[1],AngleofD1B[2],AngleofD1B[3]}
	else
		Target1=PosofD2--{AngleofD2B[1],AngleofD2B[2],AngleofD2B[3]}
		end	
	if(math.abs(DestinVRot-AngleofBA)<0.02 or math.abs(DestinVRot-AngleofAB)<0.02)then
		Target1=DestinPos
		safe_r=0
	end
	distance=100
	V1=V
	V2=4
	    oldTime = simGetSimulationTime()
	repeat
	    newTime = simGetSimulationTime()
		if(newTime-oldTime>0.05)then
			PlatformSensorsInfo=Motion_module(Target1,nil,RotofB,V1)
			pos = {0,0,0}
			pos = robot.simGetObjectPosition(handleTipPlatform, pos)
		
			distance=math.sqrt((Target1[1]-pos[1])*(Target1[1]-pos[1])+(Target1[2]-pos[2])*(Target1[2]-pos[2]))
			if(distance<0.5)then
				V1=4
			end
			oldTime=newTime
		end
	until(distance<0.05)
	if(safe_r==0)then
	Stop()
	return 
	end
	distance=100
	    oldTime = simGetSimulationTime()

	repeat
	    newTime = simGetSimulationTime()
		if(newTime-oldTime>0.05)then

		pos = {0,0,0}
		pos = robot.simGetObjectPosition(handleTipPlatform, pos)
		rot = {0,0,0}
		rot = robot.simGetObjectOrientation(handleTipPlatform,rot)
		PlatformSensorsInfo=Motion_module(DestinPos,DestinVRot,RotofB,V2)
		distance=math.sqrt((DestinPos[1]-pos[1])*(DestinPos[1]-pos[1])+(DestinPos[2]-pos[2])*(DestinPos[2]-pos[2]))
		oldTime=newTime
		end
	until(distance<0.05 and math.abs(rot[3]-RotofB[3])<math.rad(5))
	Stop()
end
RotateBySelf=function(angle,rotVel)
	simExtPrintInfo("543")
	handleTipPlatform = robot.simGetHandleVehicle()
	handleJointWheelLF0 = robot.simGetHandleJointTurnLeftFront()
	handleJointWheelRB0 = robot.simGetHandleJointTurnRightBack()
	handleJointWheelRF0 = robot.simGetHandleJointTurnRightFront()
	handleJointWheelLB0 = robot.simGetHandleJointTurnLeftBack()
	
	handleJointWheelLF = robot.simGetHandleJointWheelLeftFront()
	handleJointWheelRF = robot.simGetHandleJointWheelRightFront()
	handleJointWheelLB = robot.simGetHandleJointWheelLeftBack()
	handleJointWheelRB = robot.simGetHandleJointWheelRightBack()
	
	angle_check=angle
	rot = {0,0,0}
    rot = robot.simGetObjectOrientation(handleTipPlatform,rot)
	rot_3=rot[3]
	if rot_3 < 0 then
		rot_3=2*math.pi+rot_3
	end
    rotTarget = (rot_3+angle)%(2*math.pi)
	rotTarget1 = (rotTarget+math.pi*0.05)%(2*math.pi)
	rotTarget2 = (rotTarget-math.pi*0.05)%(2*math.pi)
	target1Arrived = false
	target2Arrived = false
	arrived=false
	mark=-angle/math.abs(angle)
    robot.simSetJointTargetPosition(handleJointWheelLF0, -47.38/180.0*math.pi)
    robot.simSetJointTargetPosition(handleJointWheelRB0, -47.38/180.0*math.pi)
    robot.simSetJointTargetPosition(handleJointWheelRF0, 47.38/180.0*math.pi)
    robot.simSetJointTargetPosition(handleJointWheelLB0, 47.38/180.0*math.pi)
	
    repeat
	
		rot = {0,0,0}
		rot = robot.simGetObjectOrientation(handleTipPlatform,rot)
		rot_3 = rot[3]
		if rot_3 < 0 then
		rot_3=2*math.pi+rot_3
		end
		arrived = false	
			if angle_check>1.98*math.pi  then
			simExtPrintInfo(">")
			time_now=simGetSimulationTime()
			if (math.abs(rot_3-rotTarget) < 0.05)then
				repeat 
					robot.simSetJointTargetVelocity(handleJointWheelLF, mark*rotVel)
					robot.simSetJointTargetVelocity(handleJointWheelRF, -mark*rotVel)
					robot.simSetJointTargetVelocity(handleJointWheelLB, mark*rotVel)
					robot.simSetJointTargetVelocity(handleJointWheelRB, -mark*rotVel)
				until(simGetSimulationTime()-time_now>0.4)
				angle_check=angle_check+2*mark*math.pi
			end 
			
			end
		if angle_check<2*math.pi then
		simExtPrintInfo("<")
		if(math.abs(rot_3-rotTarget) < 0.05)then
			arrived = true
				Stop()
			simExtPrintInfo("0")
			end

		if(arrived == false and math.abs(rot_3-rotTarget1)<0.05) then
			target1Arrived=true
			target2Arrived=false
		end
		if(arrived == false and math.abs(rot_3-rotTarget2)<0.05) then
			target1Arrived=false
			target2Arrived=true					
		end
		if(target1Arrived==false and target2Arrived==false) then
			mark=mark
		end	
		if(target1Arrived==true and target2Arrived==false) then
			mark=1/10
		end
		if(target1Arrived==false and target2Arrived==true) then
			mark=-1/10
		end
		end
		
		robot.simSetJointTargetVelocity(handleJointWheelLF, mark*rotVel)
		robot.simSetJointTargetVelocity(handleJointWheelRF, -mark*rotVel)
		robot.simSetJointTargetVelocity(handleJointWheelLB, mark*rotVel)
		robot.simSetJointTargetVelocity(handleJointWheelRB, -mark*rotVel)
		
    until(arrived == true)
	Stop()
	simExtPrintInfo(tostring(rotTarget*180/math.pi))
	simExtPrintInfo(tostring(math.abs(rot_3-rotTarget)*180/math.pi))

end
StrightToTarget=function(length,angle,maxVMove)

	handleTipPlatform = robot.simGetHandleVehicle()
	handleJointWheelLF0 = robot.simGetHandleJointTurnLeftFront()
	handleJointWheelRB0 = robot.simGetHandleJointTurnRightBack()
	handleJointWheelRF0 = robot.simGetHandleJointTurnRightFront()
	handleJointWheelLB0 = robot.simGetHandleJointTurnLeftBack()
	
	handleJointWheelLF = robot.simGetHandleJointWheelLeftFront()
	handleJointWheelRF = robot.simGetHandleJointWheelRightFront()
	handleJointWheelLB = robot.simGetHandleJointWheelLeftBack()
	handleJointWheelRB = robot.simGetHandleJointWheelRightBack()
	
	
	robot.simSetJointTargetPosition(handleJointWheelLF0, angle)
    robot.simSetJointTargetPosition(handleJointWheelRB0, angle)
    robot.simSetJointTargetPosition(handleJointWheelRF0, angle)
    robot.simSetJointTargetPosition(handleJointWheelLB0, angle)
	--[[repeat
	ang1 = {0,0,0}
	ang1=robot.simGetObjectOrientation(handleJointWheelLF0, ang1)
	simExtPrintInfo(tostring(math.deg(ang1[3])))
	simExtPrintInfo(tostring(math.deg(angle)))
	until(math.abs(ang1[3]-angle)<0.1)--]]
	pos = {0,0,0}
    pos = robot.simGetObjectPosition(handleTipPlatform, pos)
	n=1
    repeat   
		moveVel = length*20
		if (math.abs(moveVel)>maxVMove) then
			moveVel=moveVel/math.abs(moveVel)*maxVMove--*dDistanceY/math.abs(dDistanceY)
		end
		robot.simSetJointTargetVelocity(handleJointWheelLF, moveVel)
		robot.simSetJointTargetVelocity(handleJointWheelRF, moveVel)
		robot.simSetJointTargetVelocity(handleJointWheelLB, moveVel)
		robot.simSetJointTargetVelocity(handleJointWheelRB, moveVel)
		
		newPos = {0,0,0}
		newPos = robot.simGetObjectPosition(handleTipPlatform, newPos)

		distance = {newPos[1]-pos[1], newPos[2]-pos[2], newPos[3]-pos[3] }
		distance = math.sqrt(distance[1]*distance[1]+ distance[2]*distance[2]+ distance[3]*distance[3])
		arrived = false
		if(math.abs(math.abs(length)-distance) < 0.3) then
			moveVel=4
		end
		if(math.abs(math.abs(length)-distance) < 0.05) then
			arrived = true
		end

    until (arrived == true)
Stop()
end

	--RotateBySelf(1.5*math.pi,15,1)
	simExtPrintInfo("123123")
	RotateBySelf(2.5*math.pi,10)
	