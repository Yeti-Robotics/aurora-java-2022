// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionSubsystem;

public class TurnToTargetDriveCommand extends CommandBase {

  private final DrivetrainSubsystem drivetrainSubsystem;

  public TurnToTargetDriveCommand(DrivetrainSubsystem drivetrainSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    drivetrainSubsystem.cheezyDrive(0.0, 0.3); // Change to -0.3 to switch direction
  }

  @Override
  public void execute() {
    drivetrainSubsystem.cheezyDrive(0.0, 0.3); // Change to -0.3 to switch direction
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stopDrive();
  }

  @Override
  public boolean isFinished() {
    return VisionSubsystem.hasTargets() && Math.abs(VisionSubsystem.getX()) < 3.0;
  }
}
