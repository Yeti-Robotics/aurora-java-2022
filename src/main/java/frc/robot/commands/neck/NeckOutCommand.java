// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.neck;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NeckSubsystem;

public class NeckOutCommand extends CommandBase {
  /** Creates a new NeckInCommand. */
  private final NeckSubsystem neckSubsystem;

  public NeckOutCommand(NeckSubsystem neckSubsystem) {
    this.neckSubsystem = neckSubsystem;
    addRequirements(neckSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    neckSubsystem.moveDown();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    neckSubsystem.stopNeck();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
