// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class Constants 
{
	public static final double ROBOT_MASS = 50;
	public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
	public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
	public static final double MAX_SPEED = 4.5;
	// Maximum speed of the robot in meters per second, used to limit acceleration.

	public static final int SECONDS_IN_MINUTE = 60;

	public static final class DrivebaseConstants
	{
		// Hold time on motor brakes when disabled
		public static final double WHEEL_LOCK_TIME = 10; // seconds
	}

	public static class OperatorConstants 
	{
		// Joystick Deadband
		public static final double DEADBAND = 0.1;
		public static final double LEFT_Y_DEADBAND = 0.1;
		public static final double RIGHT_X_DEADBAND = 0.1;
		public static final double TURN_CONSTANT = 6;
		public static final int kDriverControllerPort = 0;
	}

	// Intake roller subsystem
	public static final int INTAKE_MOTOR_ID = 0;
	public static final int INTAKE_P = 0;
	public static final int INTAKE_I = 0;
	public static final int INTAKE_D = 0;
	public static final int INTAKE_MOTOR_CURRENT_LIMIT = 50;
	public static final double INTAKE_TOLERANCE = 0.0;
	public static final double INTAKE_POWER = 0;
	public static final double INTAKE_ROLLER_VELOCITY = 0;

	// Fuel container subsystem
	public static final double FUEL_CONTAINER_P = 0;
	public static final double FUEL_CONTAINER_I = 0;
	public static final double FUEL_CONTAINER_D = 0;
	public static final int FuelContainer_MOTOR_CURRENT_LIMIT = 5;
	public static final int FuelContainer_MOTOR_ID = 0;
	public static final double FUEL_CONTAINER_TOLERANCE = 0.0;
	public static final double FUEL_CONTAINER_POWER = 0;
	public static final double FUEL_CONTAINER_OPENED_POSE = 0;
	public static final double FUEL_CONTAINER_CLOSED_POSE = 0;

	// Shooter subsystem
	public static final int SHOOTER_MOTOR_ID = 0;
	public static final int SHOOTER_CURRENT_LIMIT = 100;
	public static final double SHOOTER_P = 0; 
	public static final double SHOOTER_I = 0;
	public static final double SHOOTER_D = 0;
	public static final double SHOOTER_WHEEL_RADIUS = 1;
	public static final double SHOOTER_WHEEL_PERIMETER = 2 * Math.PI * SHOOTER_WHEEL_RADIUS;
	public static final double SHOOTER_DEFAULT_VELOCITY = 0.0; // In RPM
	public static final double SHOOTER_VELOCITY_TOLERANCE = 50; // RPM, tolerance for considering the shooter "at speed"
	public static final double SHOOTER_POWER = 0;

	// Revolver Susbsystem
	public static final int REVOLVER_MOTOR_ID = 0;
	public static final int REVOLVER_MOTOR_CURRENT_LIMIT = 50;
	public static final double REVOLVER_P = 0;
	public static final double REVOLVER_I = 0;
	public static final double REVOLVER_D = 0;
	public static final double REVOLVER_POWER = 0;
	public static final double REVOLVER_VELOCITY = 0;
	public static final double REVOLVER_VELOCITY_TOLERANCE = 50; // RPM, tolerance for considering the revolver "at speed"

	// Climber subsystem
	public static final int CLIMBER_MOTOR_ID = 0;
	public static final double CLIMBER_P = 0;
	public static final double CLIMBER_I = 0;
	public static final double CLIMBER_D = 0;
	public static final int CLIMBER_CURRENT_LIMIT = 50;
	public static final double CLIMBER_POSITION_TOLERANCE = 0;
    public static final double CLIMBER_OPENED_HEIGHT = 0;
    public static final double CLIMBER_CLOSED_HEIGHT = 0;
	public static final int CLIMBER_POWER = 0;
}
