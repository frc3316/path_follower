package org.usfirst.frc.team3316.robot.commands;

import java.awt.Color;
import java.awt.GraphicsEnvironment;

import org.usfirst.frc.team3316.robot.RobotMap;
import org.usfirst.frc.team3316.robot.util.falcon.FalconPathPlanner;
import org.usfirst.frc.team3316.robot.util.falcon.PathPoints;
import org.usfirst.frc.team3316.robot.util.gen.Utils;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc.team3316.robot.util.falcon.*;

/**
 *
 */
public class PathFollowing extends CommandGroup {

    public PathFollowing() {
	PathPoints waypoints = new PathPoints(); // Object using to store all
						 // the points

	/*
	 * The place where you can change the path (point values in meters)
	 */
	waypoints.addPathPoint(1.0, 1.0);
	waypoints.addPathPoint(1.0, 2.2);

	FalconPathPlanner path = new FalconPathPlanner(waypoints.getPathPoints());
	path.calculate(RobotMap.pf_total_time, RobotMap.pf_step_time, RobotMap.PF_ROBOT_TRACK_WIDTH);

	// printLinePlot("Path", "X (feet)", "Y (feet)", path.smoothPath);

	addSequential(new SetSpeedPID(Utils.convertFootToMeter(path.smoothLeftVelocity[0][1]),
		Utils.convertFootToMeter(path.smoothRightVelocity[0][1]), path.heading[0][1], path));
    }

    private void printLinePlot(String title, String xTitle, String yTitle, double[][] data) {
	if (!GraphicsEnvironment.isHeadless()) {
	    FalconLinePlot fig = new FalconLinePlot(data, null, Color.blue);
	    fig.yGridOn();
	    fig.xGridOn();
	    fig.setYLabel(yTitle);
	    fig.setXLabel(xTitle);
	    fig.setTitle(title);
	}
    }

}
