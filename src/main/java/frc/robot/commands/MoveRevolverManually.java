// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Revolver;

public class MoveRevolverManually extends Command 
{
  private Revolver revolver;  // The instance of the Revolver subsystem
  private double power; //power
  
  public MoveRevolverManually(double power) {
    this.revolver = Revolver.getInstance();
    this.power = power;
    addRequirements(revolver);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.revolver.setPower(this.power);
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
