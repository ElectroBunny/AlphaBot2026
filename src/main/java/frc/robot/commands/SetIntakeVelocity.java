
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class SetIntakeVelocity extends Command 
{
  private Intake intake;
  private double velocity;

  public SetIntakeVelocity(double velocity) 
  {
    this.velocity = velocity;
    this.intake = Intake.getInstance();
    addRequirements(intake);
  }

  @Override
  public void initialize() 
  {
    this.intake.setVelocity(this.velocity);
  }
  
  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) 
  {
    this.intake.stop();
  }

  @Override
  public boolean isFinished() 
  {
    return intake.isInVelocity(this.velocity);
  }
}
