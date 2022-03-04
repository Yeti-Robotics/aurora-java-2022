// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class FlywheelPIDCommand extends CommandBase {
  private PIDController highPIDController; // high RPM shooting
  private PIDController lowPIDController; // low RPM shooting
  
  private ShooterSubsystem shooterSubsystem;

  public FlywheelPIDCommand(ShooterSubsystem shooterSubsystem) {
    highPIDController = new PIDController(ShooterConstants.HIGH_P, ShooterConstants.HIGH_I, ShooterConstants.HIGH_D);
    lowPIDController = new PIDController(ShooterConstants.LOW_P, ShooterConstants.LOW_I, ShooterConstants.LOW_D);
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double setPoint = ShooterSubsystem.setPoint;
    double RPM = shooterSubsystem.getFlywheelRPM();
    // 4000 RPM = set point at which we switch PIDs (value found through testing)
    shooterSubsystem.shootFlywheel((setPoint < 4000.0) ? ShooterConstants.LOW_F + lowPIDController.calculate(RPM, setPoint) : ShooterConstants.HIGH_F + highPIDController.calculate(RPM, setPoint));
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopFlywheel();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
