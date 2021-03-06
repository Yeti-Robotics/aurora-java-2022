// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TurnForAngleCommand extends CommandBase {

  private final DrivetrainSubsystem drivetrainSubsystem;
  private final double angle; // in degrees

  public TurnForAngleCommand(DrivetrainSubsystem drivetrainSubsystem, double angle) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.angle = angle;

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    drivetrainSubsystem.resetGyro();
  }

  @Override
  public void execute() {
    drivetrainSubsystem.cheezyDrive(0.0, 0.4);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stopDrive();
  }

  @Override
  public boolean isFinished() {
    return drivetrainSubsystem.getHeading() >= angle;
  }
}
