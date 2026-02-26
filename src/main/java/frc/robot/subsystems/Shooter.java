// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Shooter subsystem.
 *
 * <p>Controls a single flywheel motor using a {@code SparkFlex} motor controller
 * and its closed-loop velocity controller. This subsystem exposes methods to
 * set open-loop motor power, configure a closed-loop velocity setpoint, and
 * read the current velocity in both RPM and meters-per-second.
 *
 * <p>Implementation details:
 * - Uses a singleton pattern via {@link #getInstance()}.
 * - The internal closed-loop controller expects velocity in encoder units
 *   (RPM) when calling {@link #setVelocity(double)}.
 * - Linear velocity returned by {@link #getVelocityMS()} is computed from the
 *   encoder RPM using {@code Constants.SHOOTER_WHEEL_PERIMETER} and
 *   {@code Constants.SECONDS_IN_MINUTE}.
 *
 * <p>Important constants referenced from {@code frc.robot.Constants}:
 * {@code SHOOTER_MOTOR_ID}, {@code SHOOTER_CURRENT_LIMIT},
 * {@code SHOOTER_P}, {@code SHOOTER_I}, {@code SHOOTER_D},
 * {@code SHOOTER_WHEEL_PERIMETER}, and {@code SECONDS_IN_MINUTE}.
 *
 * @since 1.0
 */
public class Shooter extends SubsystemBase
 {
  private SparkFlex motor;
	private SparkFlexConfig motorConfig;
	private static Shooter instance = null;

  private RelativeEncoder encoder;
	private SparkClosedLoopController closedLoopController;

  /**
   * Creates a new Shooter subsystem.
   *
   * <p>Initializes the {@code SparkFlex} motor controller and its closed-loop
   * controller. Applies configuration values (idle mode, smart current limit,
   * and PID gains) from {@link frc.robot.Constants} and obtains the encoder
   * used for velocity feedback.
   */
  public Shooter() 
  {
    motor = new SparkFlex(Constants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
		motorConfig = new SparkFlexConfig();
    closedLoopController = motor.getClosedLoopController();

		motorConfig.idleMode(IdleMode.kBrake);
		motorConfig.smartCurrentLimit(Constants.SHOOTER_CURRENT_LIMIT);

    motorConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(Constants.SHOOTER_P)
				.i(Constants.SHOOTER_I)
				.d(Constants.SHOOTER_D)
				.outputRange(-1, 1);


    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    encoder = motor.getEncoder();
  }

  /**
   * Sets open-loop power to the shooter motor.
   *
   * @param power motor output in the range [-1.0, 1.0]. Positive values spin
   *              the wheel in the forward shooting direction.
   */
  public void setPower(double power)
  {
    this.motor.set(power);
  }

  public void stop()
  {
    this.motor.stopMotor();
  }

  /**
   * Returns the linear tangential velocity of the shooter wheel.
   *
   * <p>This converts the encoder-reported rotational speed (RPM) to meters per
   * second using {@code Constants.SHOOTER_WHEEL_PERIMETER} and
   * {@code Constants.SECONDS_IN_MINUTE}.
   *
   * @return velocity in meters per second (m/s)
   */
  public double getVelocityMS()
  {
    return (this.encoder.getVelocity() * Constants.SHOOTER_WHEEL_PERIMETER) / Constants.SECONDS_IN_MINUTE;
  }

  /**
   * Returns the shooter wheel rotational velocity as reported by the encoder.
   *
   * @return rotational velocity in RPM (encoder units)
   */
  public double getVelocityRPM()
  {
    return this.encoder.getVelocity();
  }

  /**
   * Sets a closed-loop velocity setpoint for the shooter.
   *
   * <p>The {@code vel} parameter is expressed in encoder units (RPM). The
   * underlying controller uses {@link com.revrobotics.spark.SparkBase.ControlType#kVelocity}
   * to track the requested speed.
   *
   * @param vel target velocity in RPM
   */
  public void setVelocity(double vel)
  {
    closedLoopController.setSetpoint(vel, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public double getVelocityByDistance(double distance)
  {
    // Placeholder for a method that calculates the required velocity based on distance.
    // This would likely involve some physics calculations or a lookup table.
    return Constants.SHOOTER_DEFAULT_VELOCITY; // Replace with actual implementation.
  }

  /**
   * Returns the singleton instance of the Shooter subsystem, creating it if
   * necessary.
   *
   * @return global {@code Shooter} instance
   */
  public static Shooter getInstance()
  {
    if (instance == null) 
    {
      	instance = new Shooter();
    }

    return instance;
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run. Use for telemetry
    // updates or periodic maintenance if needed.
  }
}
