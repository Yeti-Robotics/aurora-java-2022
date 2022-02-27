// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.TurretSubsystem.TurretLockStatus;

// run this command before encoders are reset
public class HomeTurretCommand extends CommandBase {
  private TurretSubsystem turretSubsystem;
  private double power;

  public HomeTurretCommand(TurretSubsystem turretSubsystem) {
    this.turretSubsystem = turretSubsystem;
    addRequirements(turretSubsystem);
  }

  @Override
  public void initialize() {
    power = -Math.signum(turretSubsystem.getEncoder()) * TurretConstants.TURRET_SPEED;
  }

  @Override
  public void execute() {
    if(!(turretSubsystem.getMagSwitch()) && turretSubsystem.lockStatus != TurretLockStatus.LOCKED){
      turretSubsystem.moveTurret(power);
    }
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stopTurret();
  }

  @Override
  public boolean isFinished() {
    return turretSubsystem.getMagSwitch() || turretSubsystem.lockStatus == TurretLockStatus.LOCKED;
  }
}
