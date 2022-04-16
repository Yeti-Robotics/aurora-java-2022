// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;

public class SnapTurretLeftCommand extends CommandBase {

    private final TurretSubsystem turretSubsystem;
    private double limit;

    public SnapTurretLeftCommand(TurretSubsystem turretSubsystem) {
        this.turretSubsystem = turretSubsystem;
        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        double currEncoder = turretSubsystem.getEncoder() - TurretConstants.TURRET_TOLERANCE;
        if (currEncoder <= TurretConstants.TURRET_MAX_LEFT) {
            limit = 0.0; // stop
        } else if (currEncoder <= TurretConstants.TURRET_45_LEFT) {
            limit = TurretConstants.TURRET_MAX_LEFT;
        } else {
            limit = TurretConstants.TURRET_45_LEFT;
        }
    }

    @Override
    public void execute() {
        if (!(turretSubsystem.getEncoder() - TurretConstants.TURRET_TOLERANCE <= limit)) {
            turretSubsystem.moveTurret(-TurretConstants.TURRET_SNAP_SPEED);
        }
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.stopTurret();
    }

    @Override
    public boolean isFinished() {
        return turretSubsystem.getEncoder() - TurretConstants.TURRET_TOLERANCE <= limit;
    }
}
