package org.usfirst.frc.team3316.robot.commands;

import org.usfirst.frc.team3316.robot.RobotMap;
import org.usfirst.frc.team3316.robot.util.falcon.FalconPathPlanner;
import org.usfirst.frc.team3316.robot.util.gen.Utils;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class PathFollowing extends CommandGroup {

    public PathFollowing() {
	double[][] waypoints = new double[][] { {1.0,1.0}, { 3.0, 3.5 }, {3.5, 3.75}, {1.0, 4.0}};

	FalconPathPlanner path = new FalconPathPlanner(waypoints);
	path.calculate(RobotMap.pf_total_time, RobotMap.pf_step_time, RobotMap.PF_ROBOT_TRACK_WIDTH);

	for (int j = 0; j < path.smoothLeftVelocity.length; j++)
	{
	    System.out.println(path.smoothLeftVelocity[j][1] + ",");
	}
	
	addSequential(new SetSpeedPID(Utils.convertFootToMeter(path.smoothLeftVelocity[0][1]),
		Utils.convertFootToMeter(path.smoothRightVelocity[0][1]), path));
    }

}
