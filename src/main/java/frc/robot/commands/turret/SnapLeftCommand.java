// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;

public class SnapLeftCommand extends CommandBase {
  private TurretSubsystem turretSubsystem;
  private double turretPostition;
  private boolean finished = false;

  public SnapLeftCommand(TurretSubsystem turretSubsystem) {
    this.turretSubsystem = turretSubsystem;
    addRequirements(turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turretPostition = turretSubsystem.getEncoder();
    if (turretPostition > TurretConstants.TURRET_MAX_LEFT) {
      turretSubsystem.moveTurret(-30);
    } else {
      turretSubsystem.moveTurret(30);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turretPostition = turretSubsystem.getEncoder();
    if (!turretSubsystem.isWithinRotationLimit()) {
      turretSubsystem.stopTurret();
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
