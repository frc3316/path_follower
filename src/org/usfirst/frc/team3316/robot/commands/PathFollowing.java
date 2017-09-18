package org.usfirst.frc.team3316.robot.commands;

import org.usfirst.frc.team3316.robot.RobotMap;
import org.usfirst.frc.team3316.robot.util.falcon.FalconPathPlanner;
import org.usfirst.frc.team3316.robot.util.gen.Utils;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class PathFollowing extends CommandGroup {

	public PathFollowing() {
		double[][] waypoints = new double[][] { { 0.5, 0.3 }, { 1, 1 } };

		FalconPathPlanner path = new FalconPathPlanner(waypoints);
		path.calculate(RobotMap.pf_total_time, RobotMap.pf_step_time, RobotMap.PF_ROBOT_TRACK_WIDTH);

		for (int i = 0; i < Math.round(RobotMap.pf_total_time / RobotMap.pf_step_time); i++) {
			addSequential(new SetSpeedPID(Utils.convertFootToMeter(path.smoothLeftVelocity[i][1]),
					Utils.convertFootToMeter(path.smoothRightVelocity[i][1])), RobotMap.pf_step_time);
		}
	}

}
