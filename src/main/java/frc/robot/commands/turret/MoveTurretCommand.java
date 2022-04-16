// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.TurretSubsystem;

public class MoveTurretCommand extends CommandBase {

    private final TurretSubsystem turretSubsystem;
    private final double power;

    public MoveTurretCommand(TurretSubsystem turretSubsystem, double power) {
        this.turretSubsystem = turretSubsystem;
        this.power = power;
        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        turretSubsystem.moveTurret(power);
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
