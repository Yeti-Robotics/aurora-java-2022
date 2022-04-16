// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.TurretSubsystem.TurretLockStatus;
import frc.robot.utils.Limelight;

public class TurretLockCommand extends PIDCommand {
  private final TurretSubsystem turretSubsystem;

  public TurretLockCommand(TurretSubsystem turretSubsystem) {
    super(
        // Tune values later
        new PIDController(
            TurretConstants.TURRET_P, TurretConstants.TURRET_I, TurretConstants.TURRET_D),
        // This should return the measurement
        // PhotonVision::getX,
        Limelight::getTx,
        // This should return the setpoint (can also be a constant)
        turretSubsystem::getTurretOffset,
        // This uses the output
        output -> {
          turretSubsystem.moveTurret(TurretConstants.TURRET_F + -output);
        });

    this.turretSubsystem = turretSubsystem;
    getController().setTolerance(TurretConstants.LIMELIGHT_TOLERANCE);

    addRequirements(turretSubsystem);
  }

  @Override
  public void execute() {
    if (turretSubsystem.lockStatus == TurretLockStatus.UNLOCKED
        || Limelight.getDistance() < 36.0
        || Limelight.getDistance() > 250.0) return;
    super.execute();
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stopTurret();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
