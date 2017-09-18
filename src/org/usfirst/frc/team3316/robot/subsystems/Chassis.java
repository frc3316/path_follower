package org.usfirst.frc.team3316.robot.subsystems;

import java.util.concurrent.Semaphore;

import org.usfirst.frc.team3316.robot.Robot;
import org.usfirst.frc.team3316.robot.RobotMap;
import org.usfirst.frc.team3316.robot.commands.EmptyCmd;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Chassis extends Subsystem {

    // Actuators
    private CANTalon motor1_left, motor2_left, motor1_right, motor2_right;

    // Sensors
    private Encoder en_left, en_right;

    public Chassis() {
	initActuators();
	initSensors();
    }

    public void initDefaultCommand() {
	setDefaultCommand(new EmptyCmd());
    }

    private void initActuators() {
	// Initializing speed controllers
	motor1_left = new CANTalon(RobotMap.MOTOR_1_LEFT);
	motor2_left = new CANTalon(RobotMap.MOTOR_2_LEFT);
	motor1_right = new CANTalon(RobotMap.MOTOR_1_RIGHT);
	motor2_right = new CANTalon(RobotMap.MOTOR_2_RIGHT);

	motor1_left.setInverted(RobotMap.MOTOR_LEFT_INVERTED);
	motor2_left.setInverted(RobotMap.MOTOR_LEFT_INVERTED);
	motor1_right.setInverted(RobotMap.MOTOR_RIGHT_INVERTED);
	motor2_right.setInverted(RobotMap.MOTOR_RIGHT_INVERTED);
    }

    private void initSensors() {
	// Initializing encoders
	en_left = new Encoder(RobotMap.EN_LEFT_CH_A, RobotMap.EN_LEFT_CH_B, RobotMap.EN_LEFT_REVERSE, EncodingType.k4X);
	en_right = new Encoder(RobotMap.EN_RIGHT_CH_A, RobotMap.EN_RIGHT_CH_B, RobotMap.EN_RIGHT_REVERSE,
		EncodingType.k4X);
	
	en_left.setDistancePerPulse(RobotMap.EN_DIST_PER_PULSE);
	en_right.setDistancePerPulse(RobotMap.EN_DIST_PER_PULSE);
    }

    public void setMotors(double leftV, double rightV) {
	setMotorLeft(leftV);
	setMotorRight(rightV);
    }

    public void setMotorLeft(double v) {
	motor1_left.set(v);
	motor2_left.set(v);
    }

    public void setMotorRight(double v) {
	motor1_right.set(v);
	motor2_right.set(v);
    }

    public double getLeftSpeed() {
	return en_left.getRate();
    }

    public double getRightSpeed() {
	return en_right.getRate();
    }

    public PIDController setSpeedPID(boolean leftSide, double Kp, double Ki, double Kd, double Kf) {
	PIDController pid = new PIDController(Kp, Ki, Kd, Kf, new PIDSource() {
	    @Override
	    public void setPIDSourceType(PIDSourceType pidSource) {
		return;
	    }

	    @Override
	    public PIDSourceType getPIDSourceType() {
		return PIDSourceType.kRate;
	    }

	    @Override
	    public double pidGet() {
		if (leftSide) {
		    return Robot.chassis.getLeftSpeed();
		}

		return Robot.chassis.getRightSpeed();
	    }
	}, new PIDOutput() {
	    @Override
	    public void pidWrite(double output) {
		if (leftSide) {
		    Robot.chassis.setMotorLeft(output);
		}
		else {
		    Robot.chassis.setMotorRight(output);
		}
	    }
	}, 0.02);

	return pid;
    }
}
