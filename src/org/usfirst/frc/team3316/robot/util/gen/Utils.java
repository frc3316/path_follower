package org.usfirst.frc.team3316.robot.util.gen;

public class Utils {
    public static double convertFootToMeter (double ft) {
	return ft * 0.3048000; 
    }
    
    public static double convertMeterToFoot (double m) {
	return m / 0.3048000; 
    }
}
