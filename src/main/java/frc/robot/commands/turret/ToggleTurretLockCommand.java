// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.TurretSubsystem.TurretLockStatus;

public class ToggleTurretLockCommand extends CommandBase {
  private final TurretSubsystem turretSubsystem;

  public ToggleTurretLockCommand(TurretSubsystem turretSubsystem) {
    this.turretSubsystem = turretSubsystem;
    addRequirements(turretSubsystem);
  }

  @Override
  public void initialize() {
    if (turretSubsystem.lockStatus == TurretLockStatus.LOCKED) {
      turretSubsystem.lockStatus = TurretLockStatus.UNLOCKED;
    } else {
      turretSubsystem.lockStatus = TurretLockStatus.LOCKED;
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
