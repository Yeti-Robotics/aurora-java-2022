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
        new PIDController(TurretConstants.kPTurretVel, TurretConstants.kITurretVel, TurretConstants.kDTurretVel),
        // This should return the measurement
        Limelight::getTx,
        // This should return the setpoint (can also be a constant)
        0.0,
        // This uses the output
        output -> {
          turretSubsystem.moveTurret(-output);
        }
    );
    this.turretSubsystem = turretSubsystem;
    addRequirements(turretSubsystem);
  }

  @Override
  public void execute() {
    if (turretSubsystem.lockStatus == TurretLockStatus.UNLOCKED) return;
    super.execute();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
