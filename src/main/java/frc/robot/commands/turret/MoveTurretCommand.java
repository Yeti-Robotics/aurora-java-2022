// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.Limelight;

public class MoveTurretCommand extends CommandBase {
  private TurretSubsystem turretSubsystem;
  private boolean atLimit = false;
  private double power;

  public MoveTurretCommand(TurretSubsystem turretSubsystem, double power) {
    this.turretSubsystem = turretSubsystem;
    this.power = power;
    addRequirements(turretSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    atLimit = (power > 0)
        ? turretSubsystem.getEncoder() >= TurretConstants.TURRET_MAX_RIGHT - TurretConstants.TURRET_TOLERANCE
        : turretSubsystem.getEncoder() <= TurretConstants.TURRET_MAX_LEFT + TurretConstants.TURRET_TOLERANCE;
    if (!atLimit) {
      turretSubsystem.moveTurret((Limelight.getTx() == 0.0) ? 0 : power);
    }
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stopTurret();
  }

  @Override
  public boolean isFinished() {
    return atLimit;
  }
}
