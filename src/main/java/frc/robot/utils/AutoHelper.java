// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.io.IOException;
import java.nio.file.Path;

public class AutoHelper {
  private static Trajectory loadTrajectoryJSON(String trajectoryJSON) {
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
      return null;
    }
  }

  private static Trajectory loadTrajectoryPath(String trajectoryPath) {
    return PathPlanner.loadPath(
        trajectoryPath, AutoConstants.MAX_SPEED, AutoConstants.MAX_ACCELERATION);
  }

  // for PathWeaver
  public static Command runTrajectoryJSON(DrivetrainSubsystem drivetrainSubsystem, String trajectoryJSON) {
    Trajectory trajectory = loadTrajectoryJSON(trajectoryJSON);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            trajectory,
            drivetrainSubsystem::getPose,
            new RamseteController(AutoConstants.RAMSETE_B, AutoConstants.RAMSETE_ZETA),
            new SimpleMotorFeedforward(
                AutoConstants.AUTO_KS, AutoConstants.AUTO_KV, AutoConstants.AUTO_KA),
            AutoConstants.KINEMATICS,
            drivetrainSubsystem::getWheelSpeeds,
            new PIDController(AutoConstants.AUTO_P, 0, 0),
            new PIDController(AutoConstants.AUTO_P, 0, 0),
            drivetrainSubsystem::tankDriveVolts,
            drivetrainSubsystem);

    return ramseteCommand.beforeStarting(
        () -> drivetrainSubsystem.resetOdometry(trajectory.getInitialPose()));
  }

  // for PathPlanner
  public static Command runTrajectoryPath(DrivetrainSubsystem drivetrainSubsystem, String trajectoryPath) {
    Trajectory trajectory = loadTrajectoryPath(trajectoryPath);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            trajectory,
            drivetrainSubsystem::getPose,
            new RamseteController(AutoConstants.RAMSETE_B, AutoConstants.RAMSETE_ZETA),
            new SimpleMotorFeedforward(
                AutoConstants.AUTO_KS, AutoConstants.AUTO_KV, AutoConstants.AUTO_KA),
            AutoConstants.KINEMATICS,
            drivetrainSubsystem::getWheelSpeeds,
            new PIDController(AutoConstants.AUTO_P, 0, 0),
            new PIDController(AutoConstants.AUTO_P, 0, 0),
            drivetrainSubsystem::tankDriveVolts,
            drivetrainSubsystem);

    return ramseteCommand.beforeStarting(
        () -> drivetrainSubsystem.resetOdometry(trajectory.getInitialPose()));
  }

  public static Command buildAutonomousCommand(Command[] autoCommands, Command[] pathCommands) {
    ParallelCommandGroup command = new ParallelCommandGroup();
    SequentialCommandGroup subsystemCommandGroup = new SequentialCommandGroup();
    SequentialCommandGroup pathCommandGroup = new SequentialCommandGroup();

    subsystemCommandGroup.addCommands(autoCommands);
    pathCommandGroup.addCommands(pathCommands);

    command.alongWith(pathCommandGroup, subsystemCommandGroup);
    return command;
  }
}
