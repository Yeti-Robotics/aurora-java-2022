// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.io.IOException;
import java.nio.file.Path;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Robot.AutoModes;
import frc.robot.commands.LED.ShooterLEDCommand;
import frc.robot.commands.commandgroups.AllInCommand;
import frc.robot.commands.commandgroups.AllInCommandGroup;
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

    private void twoBallAuto() {
        subsystemCommandGroup.addCommands(
            new ToggleIntakeCommand(robotContainer.intakeSubsystem),
            new AllInCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(3.0),
            new WaitCommand(3.0), 
            new ToggleIntakeCommand(robotContainer.intakeSubsystem),
            new WaitCommand(1.0), 
            new AllInCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(1.0),
            new InstantCommand(() -> ShooterSubsystem.setPoint = 3600.0),
            new ToggleFlywheelHighCommand(shooterLEDCommand),
            new WaitCommand(1.5),
            new AllInCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(0.3),
            new WaitCommand(0.5), 
            new AllInCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(0.7),                
            new ToggleFlywheelHighCommand(shooterLEDCommand));

        pathCommandGroup.addCommands(
            runTrajectoryPath(AutoConstants.twoBallPrimary1),
            runTrajectoryPath(AutoConstants.twoBallPrimary2)
        );

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
            runTrajectoryJSON(AutoConstants.twoBallAlternative),
            new TurnToTargetDriveCommand(robotContainer.drivetrainSubsystem).withTimeout(3.0));

        ShooterSubsystem.setPoint = 4000.0;
        command.alongWith(pathCommandGroup, subsystemCommandGroup);
    }

    private void oneBallAuto() {
        subsystemCommandGroup.addCommands(
            new DriveForDistanceCommand(robotContainer.drivetrainSubsystem, 48.0, -0.2), 
            new ToggleFlywheelHighCommand(shooterLEDCommand), 
            new WaitCommand(1.0), 
            new AllInCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(3.0), 
            new ToggleFlywheelHighCommand(shooterLEDCommand)
        );

        pathCommandGroup.addCommands();

        ShooterSubsystem.setPoint = 3800;
        command.alongWith(pathCommandGroup, subsystemCommandGroup);
    }

    private void fourBall() {
        subsystemCommandGroup.addCommands(
            new ToggleIntakeCommand(robotContainer.intakeSubsystem), 
            new AllInCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(1.5), 
            new InstantCommand(() -> ShooterSubsystem.setPoint = 4300.0),
            new ToggleFlywheelHighCommand(shooterLEDCommand), 
            new WaitCommand(0.75), 
            new ToggleIntakeCommand(robotContainer.intakeSubsystem), 
            new WaitCommand(0.5), 
            new AllInCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(0.3),
            new WaitCommand(0.5), 
            new AllInCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(0.7),
            new ToggleFlywheelHighCommand(shooterLEDCommand),
            new ToggleIntakeCommand(robotContainer.intakeSubsystem), 
            new AllInCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(4.25),
            new WaitCommand(2.5),
            new InstantCommand(() -> ShooterSubsystem.setPoint = 4100.0),
            new ToggleFlywheelHighCommand(shooterLEDCommand), 
            new WaitCommand(0.75), 
            new ToggleIntakeCommand(robotContainer.intakeSubsystem), 
            new WaitCommand(0.25), 
            new AllInCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(0.3),
            new WaitCommand(0.6), 
            new AllInCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(0.7),
            new ToggleFlywheelHighCommand(shooterLEDCommand)
        );

        pathCommandGroup.addCommands(
            runTrajectoryPath(AutoConstants.fourBall1), 
            new WaitCommand(2.25),
            runTrajectoryPath(AutoConstants.fourBall2)
        );

        command.alongWith(pathCommandGroup, subsystemCommandGroup);
    }

    private void threeBall(){
        subsystemCommandGroup.addCommands(
            new ToggleIntakeCommand(robotContainer.intakeSubsystem), 
            new AllInCommandGroup(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(2.0), 
            new ToggleIntakeCommand(robotContainer.intakeSubsystem), 
            new InstantCommand(() -> ShooterSubsystem.setPoint = 3800.0),
            new ToggleFlywheelHighCommand(shooterLEDCommand), 
            new WaitCommand(1.25), 
            new AllInCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(0.5),
            new WaitCommand(0.25), 
            new AllInCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(0.75), 
            new ToggleFlywheelHighCommand(shooterLEDCommand), 
            new WaitCommand(1.0), 
            new ToggleIntakeCommand(robotContainer.intakeSubsystem), 
            new AllInCommandGroup(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(3.0), 
            new InstantCommand(() -> ShooterSubsystem.setPoint = 3800.0),
            new ToggleFlywheelHighCommand(shooterLEDCommand), 
            new WaitCommand(1.25), 
            new AllInCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(0.5),
            new WaitCommand(0.25), 
            new AllInCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(0.75), 
            new ToggleFlywheelHighCommand(shooterLEDCommand)
        );

        pathCommandGroup.addCommands(
            runTrajectoryPath(AutoConstants.threeBall1), 
            new WaitCommand(2.0), 
            runTrajectoryPath(AutoConstants.threeBall2)
        );

        command.alongWith(pathCommandGroup, subsystemCommandGroup);
    }

    private void testAuto() {
        subsystemCommandGroup.addCommands(
            // new ToggleIntakeCommand(robotContainer.intakeSubsystem), 
            // new AllInCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(1.5), 
            // new InstantCommand(() -> ShooterSubsystem.setPoint = 4200.0),
            // new ToggleFlywheelHighCommand(shooterLEDCommand), 
            // new WaitCommand(0.75), 
            // new ToggleIntakeCommand(robotContainer.intakeSubsystem), 
            // new WaitCommand(1.25), 
            // new AllInCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(0.5),
            // new WaitCommand(0.25), 
            // new AllInCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(0.75),
            // new ToggleFlywheelHighCommand(shooterLEDCommand),
            // new ToggleIntakeCommand(robotContainer.intakeSubsystem), 
            // new AllInCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(4.5),
            // new ToggleIntakeCommand(robotContainer.intakeSubsystem), 
            // new WaitCommand(2.0),
            // new InstantCommand(() -> ShooterSubsystem.setPoint = 4200.0),
            // new ToggleFlywheelHighCommand(shooterLEDCommand), 
            // new WaitCommand(1.5), 
            // new AllInCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(0.5),
            // new WaitCommand(0.25), 
            // new AllInCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(0.5),
            // new ToggleFlywheelHighCommand(shooterLEDCommand)
        );

        pathCommandGroup.addCommands(
            runTrajectoryPath(AutoConstants.fourBall1), 
            new WaitCommand(2.25),
            runTrajectoryPath(AutoConstants.fourBall2)
        );

        command.alongWith(pathCommandGroup, subsystemCommandGroup);
    }

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
            case THREE_BALL: 
                threeBall();
                break;  
            case FOUR_BALL: 
                fourBall();
                break;
            case TEST_AUTO: 
                testAuto();
                break;
            default:
                fourBall();
                break;
        }

        return command;
    }

    private Trajectory loadTrajectoryJSON(String trajectoryJSON) {
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            return trajectory;
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            return null;
        }
    }

    private Trajectory loadTrajectoryPath(String trajectoryPath){
        Trajectory trajectory = PathPlanner.loadPath(trajectoryPath, AutoConstants.MAX_SPEED, AutoConstants.MAX_ACCELERATION);
        return trajectory;
    }

    // for PathWeaver
    private Command runTrajectoryJSON(String trajectoryJSON) {
        Trajectory trajectory = loadTrajectoryJSON(trajectoryJSON);

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

    // for PathPlanner
    private Command runTrajectoryPath(String trajectoryPath){
        Trajectory trajectory = loadTrajectoryPath(trajectoryPath);

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
