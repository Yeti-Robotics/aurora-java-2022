// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class WinchInCommand extends CommandBase {
  
  private ClimberSubsystem climberSubsystem;

  public WinchInCommand(ClimberSubsystem climberSubsystem) {
    this.climberSubsystem = climberSubsystem;
    
  }

  
  @Override
  public void initialize() {}

  
  @Override
  public void execute() {
    if(!climberSubsystem.getLimitSwitch()){
      climberSubsystem.moveWinch(ClimberConstants.CLIMBER_WINCH_SPEED);
    }
  }

  
  @Override
  public void end(boolean interrupted) {
    climberSubsystem.stopWinch();
  }

  
  @Override
  public boolean isFinished() {
    return climberSubsystem.getLimitSwitch();
  }
}
