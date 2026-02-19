

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveIntakeManually extends Command 
{
  private Intake intake;  // The instance of the Revolver subsystem
  private double power; //power

  public MoveIntakeManually(double power) {
      this. intake = Intake.getInstance();
    this.power = power;
    addRequirements(intake);

  }

  @Override
  public void initialize() {
      this.intake.setPower(this.power);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    this.intake.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
