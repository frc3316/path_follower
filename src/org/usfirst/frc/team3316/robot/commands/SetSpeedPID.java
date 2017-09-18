package org.usfirst.frc.team3316.robot.commands;

import org.usfirst.frc.team3316.robot.Robot;
import org.usfirst.frc.team3316.robot.RobotMap;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetSpeedPID extends Command {

    private PIDController pidLeft, pidRight;
    boolean pidLeftEnabled = false, pidRightEnabled = false;

    private double setpointLeft, setpointRight;

    public SetSpeedPID(double setpointLeft, double setpointRight) {
	requires(Robot.chassis);

	// Setting values
	this.setpointLeft = setpointLeft;
	this.setpointRight = setpointRight;

	// PID Left
	pidLeft = Robot.chassis.setSpeedPID(true, RobotMap.CHASSIS_SPEED_PID_LEFT_KP,
		RobotMap.CHASSIS_SPEED_PID_LEFT_KI, RobotMap.CHASSIS_SPEED_PID_LEFT_KD,
		RobotMap.CHASSIS_SPEED_PID_LEFT_KF);

	pidLeft.setAbsoluteTolerance(RobotMap.CHASSIS_SPEED_PID_TOLERANCE);
	pidLeft.setSetpoint(this.setpointLeft);
	pidLeft.setInputRange(-1.0, 1.0);

	// PID Right
	pidRight = Robot.chassis.setSpeedPID(false, RobotMap.CHASSIS_SPEED_PID_RIGHT_KP,
		RobotMap.CHASSIS_SPEED_PID_RIGHT_KI, RobotMap.CHASSIS_SPEED_PID_RIGHT_KD,
		RobotMap.CHASSIS_SPEED_PID_RIGHT_KF);

	pidRight.setAbsoluteTolerance(RobotMap.CHASSIS_SPEED_PID_TOLERANCE);
	pidRight.setSetpoint(this.setpointRight);
	pidRight.setInputRange(-1.0, 1.0);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
	pidLeft.enable();
	pidLeftEnabled = true;

	pidRight.enable();
	pidRightEnabled = true;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
	if (pidLeftEnabled && pidLeft.onTarget()) {
	    pidLeft.disable();
	    pidLeftEnabled = false;
	}

	if (pidRightEnabled && pidRight.onTarget()) {
	    pidRight.disable();
	    pidRightEnabled = false;
	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
	pidLeft.disable();
	pidLeftEnabled = false;
	
	pidRight.disable();
	pidRightEnabled = false;

	Robot.chassis.setMotors(0.0, 0.0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
	end();
    }
}
