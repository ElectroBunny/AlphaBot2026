// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Climber extends SubsystemBase
{
  private static Climber instance = null;

  private SparkFlex motor;
  private SparkFlexConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;

  private static double defaultPose = 0;
  
  /** Creates a new Climber. */
  private Climber() 
  {
    motor = new SparkFlex(Constants.CLIMBER_MOTOR_ID, MotorType.kBrushless);

    closedLoopController = motor.getClosedLoopController();
    
    motorConfig = new SparkFlexConfig();

    motorConfig.idleMode(IdleMode.kCoast);
	motorConfig.smartCurrentLimit(Constants.CLIMBER_CURRENT_LIMIT);

		// motorConfig.encoder
				// .positionConversionFactor(
						// 2 * Math.PI * Constants.ELEVATOR_ROLLER_RAIDUS / Constants.ELEVATOR_CONVERSION_FACTOR)
				// .velocityConversionFactor(
						// 2 * Math.PI * Constants.ELEVATOR_ROLLER_RAIDUS / Constants.ELEVATOR_CONVERSION_FACTOR);

		motorConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(Constants.CLIMBER_P)
				.i(Constants.CLIMBER_I)
				.d(Constants.CLIMBER_D);

		motorConfig.closedLoop.maxMotion
				.allowedClosedLoopError(Constants.CLIMBER_POSITION_TOLERANCE);

		motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		encoder = motor.getEncoder();
  }

  public void resetPosition() 
  {
		encoder.setPosition(0);
		defaultPose = 0;
	}

  public void moveClimberToPose(double setpoint) 
  {
		closedLoopController.setReference(setpoint, ControlType.kMAXMotionPositionControl,
				ClosedLoopSlot.kSlot0);
  }

  public void stop()
  {
	motor.stopMotor();
  }

	public void setPower(double power) 
  {
		motor.set(power);
	}

  public boolean isInPoint(double point) 
  {
	return (Math.abs(encoder.getPosition() - point) <= Constants.CLIMBER_POSITION_TOLERANCE);
  }

	public double getPose()
  {
		return encoder.getPosition();
	}

	public void setDefaultPose(double pose)
  {
		defaultPose = pose;
	}
	
	public static double getDefaultPose()
  {
		return defaultPose;
	}

	public static Climber getInstance() 
  {
		if (instance == null) 
    {
			instance = new Climber();
		}
		
    return instance;
	}

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("climberPose", encoder.getPosition());
		SmartDashboard.putNumber("climberSpeed", encoder.getVelocity());
		SmartDashboard.putNumber("climberCurrent", motor.getOutputCurrent());
  }
}
