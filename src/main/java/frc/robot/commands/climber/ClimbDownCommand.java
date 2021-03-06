package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbDownCommand extends CommandBase {

  private final ClimberSubsystem climberSubsystem;

  public ClimbDownCommand(ClimberSubsystem climberSubsystem) {
    this.climberSubsystem = climberSubsystem;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    climberSubsystem.climbDown();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    climberSubsystem.stopClimb();
  }
}
