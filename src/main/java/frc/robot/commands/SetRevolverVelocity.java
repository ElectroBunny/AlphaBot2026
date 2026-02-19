// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Revolver;

public class SetRevolverVelocity extends Command 
{
  private Revolver revolver;
  private double velocity;

  public SetRevolverVelocity(double vel) {
    this.revolver = Revolver.getInstance();
    this.velocity = vel;

    addRequirements(revolver);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.revolver.setVelocity(this.velocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.revolver.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
