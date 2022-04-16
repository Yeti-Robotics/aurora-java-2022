// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnForAnglePIDCommand extends PIDCommand {

  private final DrivetrainSubsystem drivetrainSubsystem;
  private final double angle;

  public TurnForAnglePIDCommand(DrivetrainSubsystem drivetrainSubsystem, double angle) {
    super(
        // The controller that the command will use
        new PIDController(DriveConstants.DRIVE_P, DriveConstants.DRIVE_I, DriveConstants.DRIVE_D),
        // This should return the measurement
        drivetrainSubsystem::getHeading,
        // This should return the setpoint (can also be a constant)
        () -> angle,
        // This uses the output
        output -> {
          double realOutput = output;
          if (Math.abs(output) >= 0.5) {
            realOutput = Math.signum(output) * 0.5;
          } else if (Math.abs(output) <= 0.175) {
            realOutput = Math.signum(output) * 0.175;
          }
          drivetrainSubsystem.cheezyDrive(0.0, realOutput);
        });

    this.drivetrainSubsystem = drivetrainSubsystem;
    this.angle = angle;

    getController().setTolerance(0.1);
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    drivetrainSubsystem.resetGyro();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    drivetrainSubsystem.stopDrive();
    System.out.println("ANGLE: " + drivetrainSubsystem.getHeading());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
