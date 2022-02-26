// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoRoutines;


import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class PathFollowingCommand extends CommandBase {
  /** Creates a new PathFollowingCommand. */
private final DrivetrainSubsystem drivetrainSubsystem;
Trajectory customTrajectory = Robot.trajectory;
private final RamseteCommand ramseteCommand;


  public PathFollowingCommand(DrivetrainSubsystem drivetrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);

    ramseteCommand = new RamseteCommand(
      customTrajectory, 
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

        drivetrainSubsystem.resetOdometry(customTrajectory.getInitialPose());

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ramseteCommand.schedule();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
