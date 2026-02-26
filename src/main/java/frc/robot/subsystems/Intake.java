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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private SparkFlex motor;
  private SparkFlexConfig motorConfig;
  private static Intake instance = null;
  private RelativeEncoder motorEncoder;
  private SparkClosedLoopController closedLoopController;

  /** Creates a new Intake. */
  public Intake() 
  {
    motor = new SparkFlex(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    motorConfig = new SparkFlexConfig();
    motorEncoder = motor.getEncoder();

    closedLoopController = motor.getClosedLoopController();

    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.smartCurrentLimit(Constants.INTAKE_MOTOR_CURRENT_LIMIT);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(Constants.INTAKE_P)
        .i(Constants.INTAKE_I)
        .d(Constants.INTAKE_D)
        .outputRange(-1, 1);

  }

  public void setPower(double voltage)
  {
    motor.set(voltage);
  }

  public void stop() 
  {
    motor.stopMotor();
  }

  public static Intake getInstance() 
  {
    if (instance == null) {
      instance = new Intake();
    }
    return instance;

  }

  public double getvelocity() 
  {
    return this.motorEncoder.getVelocity();
  }

  public void setVelocity(double velocity) 
  {
    closedLoopController.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public boolean isInVelocity(double velocity) 
  {
    return (Math.abs(motorEncoder.getVelocity() - velocity) <= Constants.INTAKE_TOLERANCE);
  }

  @Override
  public void periodic() 
  {
    SmartDashboard.putNumber("intakeCurrent", motor.getOutputCurrent());
    SmartDashboard.putNumber("intakeVelocity", motorEncoder.getVelocity());
  }
}
