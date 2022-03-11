// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.io.IOException;
import java.nio.file.Path;

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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Robot.AutoModes;
import frc.robot.commands.commandgroups.AllInCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.ToggleIntakeCommand;

/** Add your docs here. */
public class AutoBuilder {
    private RobotContainer robotContainer;
    private AutoModes autoMode;

    private ParallelCommandGroup command;
    private SequentialCommandGroup subsystemCommandGroup;
    private SequentialCommandGroup pathCommandGroup;

    // Autonomous sequences written here
    private void twoBallAuto() {
        subsystemCommandGroup.addCommands(
                new ToggleIntakeCommand(robotContainer.intakeSubsystem),
                new WaitCommand(.25),
                new AllInCommand(robotContainer.neckSubsystem, robotContainer.intakeSubsystem).withTimeout(5.0),
                new WaitCommand(1));

        pathCommandGroup.addCommands(
                runPath(AutoConstants.twoBallPrimary)
            );

        command.alongWith(pathCommandGroup, subsystemCommandGroup);
    }

    private void testAuto() {
        subsystemCommandGroup.addCommands(
            new ToggleIntakeCommand(robotContainer.intakeSubsystem),
            new IntakeInCommand(robotContainer.intakeSubsystem).withTimeout(5)
        );

        pathCommandGroup.addCommands(
            runPath("paths/scurve.wpilib.json"),
            
            runPath("paths/leftTurn.wpilib.json")
        );

        command.alongWith(pathCommandGroup, subsystemCommandGroup);
    }

    // AutoBuilder build tools here
    public void setRobotContainer(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    public void setAutoMode(AutoModes autoMode) {
        this.autoMode = autoMode;
    }

    public Command build() {
        command = new ParallelCommandGroup();
        subsystemCommandGroup = new SequentialCommandGroup();
        pathCommandGroup = new SequentialCommandGroup();

        switch ((Robot.AutoModes) autoMode) {
            case ONE_BALL:
                testAuto();
                break;
            case TWO_BALL:
                twoBallAuto();
                break;
            default: 
                testAuto();
                break;
        }

        return command;
    }

    private Trajectory loadTrajectory(String trajectoryJSON) {
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            return trajectory;
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            return null;
        }
    }

    private RamseteCommand runPath(String trajectoryJSON) {
        Trajectory trajectory = loadTrajectory(trajectoryJSON);

        RamseteCommand ramseteCommand = new RamseteCommand(
                trajectory,
                robotContainer.drivetrainSubsystem::getPose,
                new RamseteController(AutoConstants.RAMSETE_B, AutoConstants.RAMSETE_ZETA),
                new SimpleMotorFeedforward(
                    AutoConstants.AUTO_KS,
                    AutoConstants.AUTO_KV,
                    AutoConstants.AUTO_KA),
                AutoConstants.KINEMATICS,
                robotContainer.drivetrainSubsystem::getWheelSpeeds,
                new PIDController(AutoConstants.AUTO_P, 0, 0),
                new PIDController(AutoConstants.AUTO_P, 0, 0),
                robotContainer.drivetrainSubsystem::tankDriveVolts,
                robotContainer.drivetrainSubsystem);

        robotContainer.drivetrainSubsystem.resetOdometry(trajectory.getInitialPose());

        return ramseteCommand;
    }
}
