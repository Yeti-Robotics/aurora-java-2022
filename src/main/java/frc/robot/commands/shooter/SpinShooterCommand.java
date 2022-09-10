// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinShooterCommand extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final double frontPower;
  private final double backPower;

  public SpinShooterCommand(ShooterSubsystem shooterSubsystem, double frontPower, double backPower) {
    this.shooterSubsystem = shooterSubsystem;
    this.frontPower = frontPower;
    this.backPower = backPower;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.shootFlywheels(frontPower, backPower);
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
