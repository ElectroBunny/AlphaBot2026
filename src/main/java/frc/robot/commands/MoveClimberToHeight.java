// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveClimberToHeight extends Command {
  private static Climber climber;
  private double targetPose;

  private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
			//Constants.CLIMBER_MAX_VELO,
			//Constants.CLIMBER_MAX_ACCELLERATION);
  
  private final ProfiledPIDController pidController = new ProfiledPIDController(Constants.CLIMBER_P,
			Constants.CLIMBER_I, Constants.CLIMBER_D, m_constraints);

	ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0.0086531, 0.029608, 0.000215);


  public MoveClimberToHeight(double targetPose, double tolerance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.targetPose=targetPose;
    climber = Climber.getInstance();
    addRequirements(climber);
    pidController.setTolerance(tolerance);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setDefaultPose(targetPose);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = pidController.calculate(climber.getPose(), targetPose)
				+ elevatorFeedforward.calculate(pidController.getSetpoint().velocity);

		if (pidController.getPositionError() < 0 && power < -0.3) {
			power = -0.3;
		}

		climber.setPower(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setPower(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
