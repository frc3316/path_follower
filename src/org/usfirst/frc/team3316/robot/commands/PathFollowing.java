package org.usfirst.frc.team3316.robot.commands;

import org.usfirst.frc.team3316.robot.RobotMap;
import org.usfirst.frc.team3316.robot.util.falcon.FalconPathPlanner;
import org.usfirst.frc.team3316.robot.util.falcon.PathPoints;
import org.usfirst.frc.team3316.robot.util.gen.Utils;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class PathFollowing extends CommandGroup {

    public PathFollowing() {
    	PathPoints waypoints = new PathPoints(); // Object using to store all the points
    	
    	/*
    	 * The place where you can change the path
    	 */
    	waypoints.addPathPoint(1.0, 1.0);
    	waypoints.addPathPoint(2.0, 2.0);

    	
	FalconPathPlanner path = new FalconPathPlanner(waypoints.getPathPoints());
	path.calculate(RobotMap.pf_total_time, RobotMap.pf_step_time, RobotMap.PF_ROBOT_TRACK_WIDTH);

	for (int j = 0; j < path.smoothLeftVelocity.length; j++)
	{
	    System.out.println(path.smoothLeftVelocity[j][1] + ",");
	}
	
	addSequential(new SetSpeedPID(Utils.convertFootToMeter(path.smoothLeftVelocity[0][1]),
		Utils.convertFootToMeter(path.smoothRightVelocity[0][1]), path));
    }

}
