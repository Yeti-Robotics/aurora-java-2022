// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.neck;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NeckSubsystem;

public class NeckInCommand extends CommandBase {

  private final NeckSubsystem neckSubsystem;

  public NeckInCommand(NeckSubsystem neckSubsystem) {
    this.neckSubsystem = neckSubsystem;
    addRequirements(neckSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    neckSubsystem.moveUp();
  }

  @Override
  public void end(boolean interrupted) {
    neckSubsystem.stopNeck();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
