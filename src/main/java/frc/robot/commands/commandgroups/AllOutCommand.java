// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NeckSubsystem;

public class AllOutCommand extends CommandBase {

  private final IntakeSubsystem intakeSubsystem;
  private final NeckSubsystem neckSubsystem;

  /** Creates a new AllOutCommand. */
  public AllOutCommand(IntakeSubsystem intakeSubsystem, NeckSubsystem neckSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.neckSubsystem = neckSubsystem;
    addRequirements(neckSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    neckSubsystem.moveDown();
    intakeSubsystem.rollOut();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    neckSubsystem.stopNeck();
    intakeSubsystem.stopRoll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
