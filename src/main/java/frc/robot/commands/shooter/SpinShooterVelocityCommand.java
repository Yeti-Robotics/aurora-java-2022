// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinShooterVelocityCommand extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final double frontVel;
  private final double backVel;

  public SpinShooterVelocityCommand(ShooterSubsystem shooterSubsystem, double frontVel, double backVel) {
    this.shooterSubsystem = shooterSubsystem;
    this.frontVel = frontVel;
    this.backVel = backVel;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setFlywheelVelocities(frontVel, backVel);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopFlywheels();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
