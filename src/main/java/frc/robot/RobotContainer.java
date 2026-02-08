// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;

import swervelib.SwerveInputStream;

public class RobotContainer {
	final CommandPS5Controller driverController = new CommandPS5Controller(0);

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
		NamedCommands.registerCommand("test", Commands.print("I EXIST"));
	}

	private void configureBindings() {

		drivebase.setDefaultCommand(
				!RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedAnglularVelocitySim);
	}

	public void setMotorBrake(boolean brake) {
		drivebase.setMotorBrake(brake);
	}

  /**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return m_chooser.getSelected();
	}

}