package org.usfirst.frc.team3316.robot.commands;

import org.usfirst.frc.team3316.robot.RobotMap;
import org.usfirst.frc.team3316.robot.util.falcon.FalconPathPlanner;
import org.usfirst.frc.team3316.robot.util.gen.Utils;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class PathFollowing extends Command {
    double lastTime;
    FalconPathPlanner path;
    int i = 1;
    Command cmd;

    public PathFollowing() {

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
//	double currentTime = System.currentTimeMillis();
//	if (currentTime - lastTime >= RobotMap.pf_step_time * 1000) {
//	    cmd.cancel();
//	    cmd = null;
//
//	    cmd = (new SetSpeedPID(Utils.convertFootToMeter(path.smoothLeftVelocity[i][1]),
//		    Utils.convertFootToMeter(path.smoothRightVelocity[i][1])));
//	    cmd.start();
//
//	    i++;
//	}
//	lastTime = currentTime;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }

    // Called just before this Command runs the first time
    protected void initialize() {
	double[][] waypoints = new double[][] { { 0.5, 0.3 }, { 1, 1 } };

	path = new FalconPathPlanner(waypoints);
	path.calculate(RobotMap.pf_total_time, RobotMap.pf_step_time, RobotMap.PF_ROBOT_TRACK_WIDTH);

	cmd = (new SetSpeedPID(Utils.convertFootToMeter(path.smoothLeftVelocity[7][1]),
		Utils.convertFootToMeter(path.smoothRightVelocity[7][1])));
	cmd.start();
	
	lastTime = System.currentTimeMillis();
    }

    @Override
    protected boolean isFinished() {
	// TODO Auto-generated method stub
	return i >= Math.round(RobotMap.pf_total_time / RobotMap.pf_step_time);
    }
}
