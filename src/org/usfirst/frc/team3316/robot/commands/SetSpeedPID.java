package org.usfirst.frc.team3316.robot.commands;

import org.usfirst.frc.team3316.robot.Robot;
import org.usfirst.frc.team3316.robot.RobotMap;
import org.usfirst.frc.team3316.robot.util.falcon.FalconPathPlanner;
import org.usfirst.frc.team3316.robot.util.gen.Utils;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetSpeedPID extends Command {

    private PIDController pidLeft, pidRight, pidYaw;
    private double setpointLeft, setpointRight, setpointYaw;

    private FalconPathPlanner path;

    private int i = 1;

    private long lastTime;

    public SetSpeedPID(double setpointLeft, double setpointRight, double setpointYaw, FalconPathPlanner path) {
//	System.out.println("sepoint: " + setpointLeft);

	requires(Robot.chassis);

	// Setting values
	this.setpointLeft = setpointLeft;
	this.setpointRight = setpointRight;
	this.setpointYaw = setpointYaw;

	this.path = path;
    }

    // Called just before this Command runs the first time
    protected void initialize() {

	// PID Left
	pidLeft = Robot.chassis.setSpeedPID(true, RobotMap.CHASSIS_SPEED_PID_LEFT_KP,
		RobotMap.CHASSIS_SPEED_PID_LEFT_KI, RobotMap.CHASSIS_SPEED_PID_LEFT_KD,
		RobotMap.CHASSIS_SPEED_PID_LEFT_KF);

	pidLeft.setAbsoluteTolerance(RobotMap.CHASSIS_SPEED_PID_TOLERANCE);
	pidLeft.setSetpoint(this.setpointLeft);
	pidLeft.setOutputRange(-1.0, 1.0);

	// PID Right
	pidRight = Robot.chassis.setSpeedPID(false, RobotMap.CHASSIS_SPEED_PID_RIGHT_KP,
		RobotMap.CHASSIS_SPEED_PID_RIGHT_KI, RobotMap.CHASSIS_SPEED_PID_RIGHT_KD,
		RobotMap.CHASSIS_SPEED_PID_RIGHT_KF);

	pidRight.setAbsoluteTolerance(RobotMap.CHASSIS_SPEED_PID_TOLERANCE);
	pidRight.setSetpoint(this.setpointRight);
	pidRight.setOutputRange(-1.0, 1.0);

	// PID Yaw
	pidYaw = Robot.chassis.setYawPID(RobotMap.CHASSIS_YAW_PID_KP, RobotMap.CHASSIS_YAW_PID_KI,
		RobotMap.CHASSIS_YAW_PID_KD, RobotMap.CHASSIS_YAW_PID_KF);
	pidYaw.setSetpoint(this.setpointYaw);
	pidYaw.setOutputRange(-1.0, 1.0);

	pidLeft.enable();

	pidRight.enable();
	
	pidYaw.enable();

	lastTime = System.currentTimeMillis();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
	long currentTime = System.currentTimeMillis();
	if (currentTime - lastTime >= (long) (RobotMap.pf_step_time * 1000) && i < path.smoothCenterVelocity.length) {
	    pidLeft.setSetpoint(Utils.convertFootToMeter(path.smoothLeftVelocity[i][1]));
	    pidRight.setSetpoint(Utils.convertFootToMeter(path.smoothRightVelocity[i][1]));
	    
	    pidYaw.setSetpoint(-(path.heading[i][1] - 90));

	    i++;
	    lastTime = currentTime;
	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
	return i >= path.smoothCenterVelocity.length;
    }

    // Called once after isFinished returns true
    protected void end() {
	pidLeft.disable();

	pidRight.disable();

	Robot.chassis.setMotors(0.0, 0.0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
	end();
    }
}
