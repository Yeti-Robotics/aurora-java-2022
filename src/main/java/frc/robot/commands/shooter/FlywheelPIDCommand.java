// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class FlywheelPIDCommand extends PIDCommand {
  public FlywheelPIDCommand(ShooterSubsystem shooterSubsystem) {
    super(
        // The controller that the command will use
        new PIDController(ShooterConstants.SHOOTER_P, ShooterConstants.SHOOTER_I, ShooterConstants.SHOOTER_D),
        // This should return the measurement
        shooterSubsystem::getFlywheelRPM,
        // This should return the setpoint (can also be a constant)
        () -> ShooterSubsystem.setPoint,
        // This uses the output
        output -> {
          shooterSubsystem.shootFlywheel(ShooterConstants.SHOOTER_F + output);
        });
    addRequirements(shooterSubsystem);
    getController().setTolerance(ShooterConstants.RPM_TOLERANCE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
