package org.usfirst.frc.team3316.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

    /*
     * CHASSIS
     */

    // SPEED CONTROLLERS
    public static final int MOTOR_1_LEFT = 2;
    public static final int MOTOR_2_LEFT = 3;
    public static final int MOTOR_1_RIGHT = 0;
    public static final int MOTOR_2_RIGHT = 1;
    
    public static final boolean MOTOR_LEFT_INVERTED = false;
    public static final boolean MOTOR_RIGHT_INVERTED = true;
    
    // ENCODERS
    public static final int EN_RIGHT_CH_A = 0;
    public static final int EN_RIGHT_CH_B = 1;
    public static final int EN_LEFT_CH_A = 2;
    public static final int EN_LEFT_CH_B = 3;
    
    public static final boolean EN_RIGHT_REVERSE = true;
    public static final boolean EN_LEFT_REVERSE = false;
    
    public static final double EN_DIST_PER_PULSE = 0.00124224; // in meters
    
    
    /*
     * Controllers
     */
    
    // CHASSIS SPEED PID
    public static final double CHASSIS_SPEED_PID_LEFT_KP = 100.0 / 1000.0;
    public static final double CHASSIS_SPEED_PID_LEFT_KI = 0.0 / 1000.0;
    public static final double CHASSIS_SPEED_PID_LEFT_KD = 0.0 / 1000.0;
    public static final double CHASSIS_SPEED_PID_LEFT_KF = 50.0 / 1000.0;
    
    public static final double CHASSIS_SPEED_PID_RIGHT_KP = 120.0 / 1000.0;
    public static final double CHASSIS_SPEED_PID_RIGHT_KI = 0.0 / 1000.0;
    public static final double CHASSIS_SPEED_PID_RIGHT_KD = 0.0 / 1000.0;
    public static final double CHASSIS_SPEED_PID_RIGHT_KF = 50.0 / 1000.0;
    
    public static final double CHASSIS_SPEED_PID_TOLERANCE = 0.05;
    
    
    /*
     * PATH FOLLOWER
     */

    // CONSTANTS
    public static final double PF_ROBOT_TRACK_WIDTH = 1.5; // in ft.

    // VARIABLES
    public static double pf_total_time = 10; // in seconds
    public static double pf_step_time = 1; // in seconds

}
