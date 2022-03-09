// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoRoutines;

import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class PathFollowingCommand extends CommandBase {
  /** Creates a new PathFollowingCommand. */
  private final DrivetrainSubsystem drivetrainSubsystem;
  private RamseteCommand ramseteCommand;
  private Trajectory trajectory = Robot.trajectory;

  public PathFollowingCommand(DrivetrainSubsystem drivetrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ramseteCommand = new RamseteCommand(
        trajectory,
        drivetrainSubsystem::getPose,
        new RamseteController(AutoConstants.RAMSETE_B, AutoConstants.RAMSETE_ZETA),
        new SimpleMotorFeedforward(
            AutoConstants.AUTO_KS,
            AutoConstants.AUTO_KV,
            AutoConstants.AUTO_KA),
        AutoConstants.KINEMATICS,
        drivetrainSubsystem::getWheelSpeeds,
        new PIDController(AutoConstants.AUTO_P, 0, 0),
        new PIDController(AutoConstants.AUTO_P, 0, 0),
        drivetrainSubsystem::tankDriveVolts,
        drivetrainSubsystem);

    drivetrainSubsystem.resetOdometry(trajectory.getInitialPose());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
