// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.MoveClimberManually;
import frc.robot.commands.MoveClimberToHeight;
import frc.robot.commands.MoveFuelContainerManually;
import frc.robot.commands.MoveFuelContainerToPos;
import frc.robot.commands.MoveIntakeManually;
import frc.robot.commands.SetRevolverVelocity;
import frc.robot.commands.SetShooterVelocity;
import frc.robot.commands.MoveRevolverManually;
import frc.robot.commands.MoveShooterByDistance;
import frc.robot.commands.MoveShooterManually;
import frc.robot.commands.SetIntakeVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.function.DoubleSupplier;

import swervelib.SwerveInputStream;

public class RobotContainer {
	final CommandPS5Controller driverController = new CommandPS5Controller(0);
	final CommandPS5Controller operatorController = new CommandPS5Controller(1);

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  
	public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
			"swerve"));

	DoubleSupplier swerveSpeedScaleTranslation = () -> 1;
	DoubleSupplier swerveSpeedScaleRotation = () -> 1;

	SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
			() -> driverController.getLeftY() * -1 * swerveSpeedScaleTranslation.getAsDouble(),
			() -> driverController.getLeftX() * -1 * swerveSpeedScaleTranslation.getAsDouble())
			.withControllerRotationAxis(
					() -> driverController.getRightX() * -1 * swerveSpeedScaleRotation.getAsDouble())
			.deadband(OperatorConstants.DEADBAND)
			.cubeRotationControllerAxis(true)
			.cubeRotationControllerAxis(true)
			.allianceRelativeControl(true);

	Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

	SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
			() -> -driverController.getLeftY(),
			() -> -driverController.getLeftX())
			.withControllerRotationAxis(() -> driverController.getRawAxis(2))
			.deadband(OperatorConstants.DEADBAND)
			.scaleTranslation(0.8)
			.allianceRelativeControl(true);

	SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
			.withControllerHeadingAxis(() -> Math.sin(
					driverController.getRawAxis(
							2) * Math.PI)
					* (Math.PI * 2),
					() -> Math.cos(
							driverController.getRawAxis(
									2) * Math.PI)
							*
							(Math.PI * 2))
			.headingWhile(true);

	Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

	Command driveFieldOrientedAnglularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);

	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();
		DriverStation.silenceJoystickConnectionWarning(true);
	}

	public void setMotorBrake(boolean brake) {
		drivebase.setMotorBrake(brake);
	}

  private void configureBindings() 
  {
	// Swerve
	drivebase.setDefaultCommand(
				!RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedAnglularVelocitySim);


    operatorController.R2().whileTrue(new MoveRevolverManually(Constants.REVOLVER_POWER));
    operatorController.R1().toggleOnTrue(new SetRevolverVelocity(Constants.REVOLVER_VELOCITY));

	operatorController.cross().whileTrue(new MoveShooterManually(Constants.SHOOTER_POWER));
    operatorController.square().toggleOnTrue(new SetShooterVelocity(Constants.SHOOTER_DEFAULT_VELOCITY));
	operatorController.triangle().toggleOnTrue(new MoveShooterByDistance(0)); // Placeholder distance, replace with actual value or input method.

	operatorController.povUp().onTrue(new MoveFuelContainerToPos(Constants.FUEL_CONTAINER_OPENED_POSE));
	operatorController.povDown().onTrue(new MoveFuelContainerToPos(Constants.FUEL_CONTAINER_CLOSED_POSE));
	operatorController.povRight().whileTrue(new MoveFuelContainerManually(Constants.FUEL_CONTAINER_POWER));
	operatorController.povLeft().whileTrue(new MoveFuelContainerManually(-Constants.FUEL_CONTAINER_POWER));

	driverController.L1().whileTrue(new MoveIntakeManually(Constants.INTAKE_POWER));
	driverController.L2().whileTrue(new MoveIntakeManually(-Constants.INTAKE_POWER));
	driverController.cross().toggleOnTrue(new SetIntakeVelocity(Constants.INTAKE_ROLLER_VELOCITY));

	driverController.povUp().onTrue(new MoveClimberToHeight(Constants.CLIMBER_OPENED_HEIGHT));
	driverController.povDown().onTrue(new MoveClimberToHeight(Constants.CLIMBER_CLOSED_HEIGHT));
	driverController.povRight().whileTrue(new MoveClimberManually(Constants.CLIMBER_POWER));
	driverController.povLeft().whileTrue(new MoveClimberManually(-Constants.CLIMBER_POWER));
  }

	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return m_chooser.getSelected();
	}

}
