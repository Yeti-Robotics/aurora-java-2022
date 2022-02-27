// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.Limelight;

public class TurnToTargetCommand extends CommandBase {
  private TurretSubsystem turretSubsystem;
  private double power;
  private boolean atLimit = false;
  private boolean isAligned = false;

  // power should be positive
  public TurnToTargetCommand(TurretSubsystem turretSubsystem, double power) {
    this.turretSubsystem = turretSubsystem;
    this.power = power;

    addRequirements(turretSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    power = Math.abs(power) * ((Limelight.getTx() < 0) ? -1 : 1);
    isAligned = Math.abs(Limelight.getTx()) <= TurretConstants.LIMELIGHT_TOLERANCE;

    System.out.println("GETTX: " + Limelight.getTx());
    System.out.println("POWER: " + power);

    atLimit = (power > 0)
        ? turretSubsystem.getEncoder() >= TurretConstants.TURRET_MAX_RIGHT - TurretConstants.TURRET_TOLERANCE
        : turretSubsystem.getEncoder() <= TurretConstants.TURRET_MAX_LEFT + TurretConstants.TURRET_TOLERANCE;
    if(!isAligned || !atLimit){
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
