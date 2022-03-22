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
import frc.robot.commands.LED.ShooterLEDCommand;
import frc.robot.commands.commandgroups.AllInCommand;
import frc.robot.commands.drivetrain.DriveForDistanceCommand;
import frc.robot.commands.drivetrain.TurnToTargetDriveCommand;
import frc.robot.commands.intake.ToggleIntakeCommand;
import frc.robot.commands.shooter.ToggleFlywheelHighCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoBuilder {
    private RobotContainer robotContainer;
    private ShooterLEDCommand shooterLEDCommand;
    private AutoModes autoMode;

    private ParallelCommandGroup command;
    private SequentialCommandGroup subsystemCommandGroup;
    private SequentialCommandGroup pathCommandGroup;

    // Autonomous sequences written here
    private void twoBallAuto() {
        subsystemCommandGroup.addCommands(
            new ToggleIntakeCommand(robotContainer.intakeSubsystem),
            new AllInCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(3.0),
            new ToggleIntakeCommand(robotContainer.intakeSubsystem),
            new ToggleFlywheelHighCommand(shooterLEDCommand),
            new WaitCommand(3.0),
            new AllInCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(2.0),
            new ToggleFlywheelHighCommand(shooterLEDCommand));

        pathCommandGroup.addCommands(
            runPathCommand(AutoConstants.twoBallPrimary),
            new TurnToTargetDriveCommand(robotContainer.drivetrainSubsystem).withTimeout(3.0));

        ShooterSubsystem.setPoint = 4250.0;
        command.alongWith(pathCommandGroup, subsystemCommandGroup);
    }

    private void twoBallAlternative() {
        subsystemCommandGroup.addCommands(
            new ToggleIntakeCommand(robotContainer.intakeSubsystem),
            new AllInCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(3.0),
            new ToggleIntakeCommand(robotContainer.intakeSubsystem),
            new ToggleFlywheelHighCommand(shooterLEDCommand),
            new WaitCommand(3.0), 
            new AllInCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(2.0),
            new ToggleFlywheelHighCommand(shooterLEDCommand));

        pathCommandGroup.addCommands(
            runPathCommand(AutoConstants.twoBallAlternative),
            new TurnToTargetDriveCommand(robotContainer.drivetrainSubsystem).withTimeout(3.0));

        ShooterSubsystem.setPoint = 4000.0;
        command.alongWith(pathCommandGroup, subsystemCommandGroup);
    }

    private void oneBallAuto() {
        subsystemCommandGroup.addCommands(
            new DriveForDistanceCommand(robotContainer.drivetrainSubsystem, 48.0, -0.2, -0.2), 
            new ToggleFlywheelHighCommand(shooterLEDCommand), 
            new WaitCommand(1.0), 
            new AllInCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(3.0), 
            new ToggleFlywheelHighCommand(shooterLEDCommand)
        );

        pathCommandGroup.addCommands();

        ShooterSubsystem.setPoint = 3800;
        command.alongWith(pathCommandGroup, subsystemCommandGroup);
    }

    private void testAuto() {}

    // AutoBuilder build tools here
    public void setRobotContainer(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        this.shooterLEDCommand = new ShooterLEDCommand(robotContainer.ledSubsystem);
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
                oneBallAuto();
                break;
            case TWO_BALL:
                twoBallAuto();
                break;
            case TWO_BALL_ALTERNATIVE:
                twoBallAlternative();
                break;
            case TEST_AUTO: 
                // testAuto();
                return new TurnToTargetDriveCommand(robotContainer.drivetrainSubsystem).withTimeout(3.0);
            default:
                oneBallAuto();
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

    private Command runPathCommand(String trajectoryJSON) {
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

        return ramseteCommand
                .beforeStarting(() -> robotContainer.drivetrainSubsystem.resetOdometry(trajectory.getInitialPose()));
    }
}
