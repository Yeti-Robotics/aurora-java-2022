// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NeckSubsystem;

public class AllOutCommand extends CommandBase {

  private IntakeSubsystem intakeSubsystem;
  private NeckSubsystem neckSubsystem;

  public AllOutCommand(IntakeSubsystem intakeSubsystem, NeckSubsystem neckSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.neckSubsystem = neckSubsystem;
    addRequirements(neckSubsystem);
  }

  
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    neckSubsystem.moveDown();
    intakeSubsystem.rollOut();
  }

  @Override
  public void end(boolean interrupted) {
    neckSubsystem.stopNeck();
    intakeSubsystem.stopRoll();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
