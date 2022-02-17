// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NeckSubsystem;

public class AllInCommand extends CommandBase {

  private NeckSubsystem neckSubsystem;
  private IntakeSubsystem intakeSubsystem;
  /** Creates a new AllInCommand. */
  public AllInCommand(NeckSubsystem neckSubsystem, IntakeSubsystem intakeSubsystem) {
    addRequirements(neckSubsystem, intakeSubsystem);
    this.neckSubsystem = neckSubsystem;
    this.intakeSubsystem = intakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.rollIn();
    if (!neckSubsystem.getUpperBeamBreak()) {
      neckSubsystem.moveUp();
    }
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
