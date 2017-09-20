
package org.usfirst.frc.team3316.robot;

import java.util.Timer;

import org.usfirst.frc.team3316.robot.paths.Path1;
import org.usfirst.frc.team3316.robot.paths.PathsCommandGroup;
import org.usfirst.frc.team3316.robot.subsystems.Chassis;
import org.usfirst.frc.team3316.robot.subsystems.None;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

    public static Timer timer;

    public static Chassis chassis;
    public static None none;
    
    public static OI oi;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
	timer = new Timer();

	oi = new OI();

	chassis = new Chassis();
	none = new None();

	SmartDashboard.putData("Paths together", new PathsCommandGroup());
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     * You can use it to reset any subsystem information you want to clear when
     * the robot is disabled.
     */
    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {
	Scheduler.getInstance().run();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString code to get the auto name from the text box below the Gyro
     *
     * You can add additional auto modes by adding additional commands to the
     * chooser code above (like the commented example) or additional comparisons
     * to the switch structure below with additional strings & commands.
     */
    @Override
    public void autonomousInit() {

    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
	Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {

    }

    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
	Scheduler.getInstance().run();

	SmartDashboard.putNumber("LEFT SPEED", this.chassis.getLeftSpeed());
	SmartDashboard.putNumber("RIGHT SPEED", this.chassis.getRightSpeed());

	SmartDashboard.putNumber("YAW", Robot.chassis.getYawAngle());
    }

    /**
     * This function is called periodically during test mode
     */
    @Override
    public void testPeriodic() {
	LiveWindow.run();
    }
}
