// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoRoutines;

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
  String trajectoryJSON;
  private Trajectory trajectory;
  
  
  
  
  
  public PathFollowingCommand(DrivetrainSubsystem drivetrainSubsystem, String trajectoryJSON) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);
    

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    ramseteCommand = new RamseteCommand(
      trajectory,
      drivetrainSubsystem::getPose,
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
      new SimpleMotorFeedforward(
          AutoConstants.ksVolts,
          AutoConstants.kvVoltSecondsPerMeters,
          AutoConstants.kaVoltSecondsSquaredPerMeter),
      AutoConstants.kinematics,
      drivetrainSubsystem::getWheelSpeeds,
      new PIDController(AutoConstants.kPDriveVel, 0, 0),
      new PIDController(AutoConstants.kPDriveVel, 0, 0),
      drivetrainSubsystem::tankDriveVolts,
      drivetrainSubsystem);

  drivetrainSubsystem.resetOdometry(trajectory.getInitialPose());

    ramseteCommand.schedule();

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
    return false;
  }
}
