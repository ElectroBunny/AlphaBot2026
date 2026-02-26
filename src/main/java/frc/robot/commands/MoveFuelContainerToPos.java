// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FuelContainer;

public class MoveFuelContainerToPos extends Command 
{
  private FuelContainer fuelContainer;
  private double pose;

  public MoveFuelContainerToPos(double pose) 
  {
    this.pose = pose;
    addRequirements(fuelContainer);
  }

  @Override
  public void initialize() 
  {
    fuelContainer.setPosition(this.pose);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) 
  {
    fuelContainer.stop();
  }

  @Override
  public boolean isFinished() {
    return fuelContainer.isInPoint(this.pose);
  }
}
