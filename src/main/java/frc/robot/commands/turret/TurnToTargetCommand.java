// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionSubsystem;

public class TurnToTargetCommand extends CommandBase {

  private final TurretSubsystem turretSubsystem;
  private double power;
  private boolean atLimit = false;
  private boolean isAligned = false;

  private VisionSubsystem visionSubsystem;

  // power should be positive
  public TurnToTargetCommand(
      TurretSubsystem turretSubsystem, VisionSubsystem visionSubsystem, double power) {
    this.turretSubsystem = turretSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.power = power;

    addRequirements(turretSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    power = Math.abs(power) * ((visionSubsystem.getX() < 0) ? -1 : 1);
    isAligned = Math.abs(visionSubsystem.getX()) <= TurretConstants.LIMELIGHT_TOLERANCE;

    atLimit =
        (power > 0)
            ? turretSubsystem.getEncoder()
                >= TurretConstants.TURRET_MAX_RIGHT - TurretConstants.TURRET_TOLERANCE
            : turretSubsystem.getEncoder()
                <= TurretConstants.TURRET_MAX_LEFT + TurretConstants.TURRET_TOLERANCE;
    if (!isAligned || !atLimit) {
      turretSubsystem.moveTurret(power);
    }
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stopTurret();
  }

  @Override
  public boolean isFinished() {
    return atLimit; // || isAligned
  }
}
