// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShiftingSubsystem;
import frc.robot.subsystems.ShiftingSubsystem.ShiftStatus;

public class ClimbAndWinchInCommand extends CommandBase {
  private final ClimberSubsystem climberSubsystem;
  private ShiftingSubsystem shiftingSubsystem;

  public ClimbAndWinchInCommand(ClimberSubsystem climberSubsystem, ShiftingSubsystem shiftingSubsystem) {
    this.climberSubsystem = climberSubsystem;
    addRequirements(climberSubsystem);
  }

  @Override
  public void initialize() {
    if(ShiftingSubsystem.shiftStatus == ShiftStatus.LOW){
      shiftingSubsystem.shiftUp();
    }
  }

  @Override
  public void execute() {
    if (climberSubsystem.getLimitSwitch()) {
      climberSubsystem.stopWinch();
    } else {
      climberSubsystem.moveWinch(ClimberConstants.CLIMBER_WINCH_SPEED);
    }

    if (climberSubsystem.getAverageEncoder() <= ClimberConstants.CLIMBER_TOLERANCE || ShiftingSubsystem.shiftStatus == ShiftStatus.LOW) {
      climberSubsystem.stopClimb();
    } else {
      climberSubsystem.climbDown();
    }
  }

  @Override
  public void end(boolean interrupted) {
    climberSubsystem.stopWinch();
    climberSubsystem.stopClimb();
  }

  @Override
  public boolean isFinished() {
    return climberSubsystem.getAverageEncoder() <= ClimberConstants.CLIMBER_TOLERANCE && climberSubsystem.getLimitSwitch();
  }
}
