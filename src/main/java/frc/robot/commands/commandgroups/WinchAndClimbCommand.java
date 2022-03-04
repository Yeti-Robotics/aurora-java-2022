// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.MovingBrakeStatus;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WinchAndClimbCommand extends CommandBase {
  private ClimberSubsystem climberSubsystem;
  /** Creates a new WinchAndClimbCommand. */
  public WinchAndClimbCommand(ClimberSubsystem climberSubsystem) {
    this.climberSubsystem = climberSubsystem;
    addRequirements(climberSubsystem);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climberSubsystem.getBrakeStatus() == MovingBrakeStatus.OFF){
      climberSubsystem.climbDown();
      climberSubsystem.moveWinch(-ClimberConstants.CLIMBER_WINCH_SPEED); 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberSubsystem.stopClimb();
    climberSubsystem.stopWinch();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
