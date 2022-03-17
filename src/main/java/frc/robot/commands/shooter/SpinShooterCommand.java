// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinShooterCommand extends CommandBase {
  private ShooterSubsystem shooterSubsystem;
  private double power;
  private double maxVel = 0.0;

  public SpinShooterCommand(ShooterSubsystem shooterSubsystem, double power) {
    this.shooterSubsystem = shooterSubsystem;
    this.power = power;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooterSubsystem.shootFlywheel(power);

    double currVel = shooterSubsystem.getAverageEncoder(); 
    if(currVel > maxVel){
      maxVel = currVel;
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopFlywheel();
    System.out.println("MAX VEL AT " + (power * 100) + "%: " + maxVel);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
