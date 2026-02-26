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
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FuelContainer extends SubsystemBase {
  private SparkFlex motor;
  private SparkFlexConfig motorConfig;
  private RelativeEncoder motorEncoder;
  private static FuelContainer instance = null;
  private SparkClosedLoopController closedLoopController;

  /** Creates a new FuelContainer. */
  public FuelContainer() {
    this.motor = new SparkFlex(Constants.FuelContainer_MOTOR_ID, MotorType.kBrushless);
    this.motorConfig = new SparkFlexConfig();
    this.motorEncoder = motor.getEncoder();

    closedLoopController = motor.getClosedLoopController();

    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.smartCurrentLimit(Constants.INTAKE_MOTOR_CURRENT_LIMIT);

    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(Constants.FUEL_CONTAINER_P)
        .i(Constants.FUEL_CONTAINER_I)
        .d(Constants.FUEL_CONTAINER_D)
        .outputRange(-1, 1);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setPower(double power)
  {
    motor.setVoltage(power);
  }

  public void stop()
  {
    motor.stopMotor();
  }

  public void setPosition(double position) 
  {
    closedLoopController.setSetpoint(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public boolean isInPoint(double point) 
  {
	return (Math.abs(motorEncoder.getPosition() - point) <= Constants.FUEL_CONTAINER_TOLERANCE);
  }

  public static FuelContainer getInstance()
  {
    if (instance == null)
    {
      instance = new FuelContainer();
    }
    return instance;
  }

  @Override
  public void periodic() 
  {
    SmartDashboard.putNumber("FuelContainerCurrent", motor.getOutputCurrent());
    SmartDashboard.putNumber("FuelContainerPosition", motorEncoder.getVelocity());
  }
}
