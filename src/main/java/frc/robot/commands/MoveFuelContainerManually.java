// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FuelContainer;

public class MoveFuelContainerManually extends Command 
{
  private FuelContainer fuelContainer;
  private double power;
  
  public MoveFuelContainerManually(double power)
  {
    this.fuelContainer = FuelContainer.getInstance();
    this.power = power;
    addRequirements(this.fuelContainer);
  }

  @Override
  public void initialize() 
  {
    this.fuelContainer.setPower(this.power);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) 
  {
    this.fuelContainer.stop();
  }

  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
