// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;
import java.io.IOException;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.CompressorConfigType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.LED.AuroraLEDCommand;
import frc.robot.commands.LED.BlinkLEDCommand;
import frc.robot.commands.LED.SetLEDYetiBlueCommand;
import frc.robot.commands.drivetrain.DriveForDistanceCommand;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command beforeBlinkCommand = null;
  private boolean blinkWarningRan = false;
  public CompressorConfigType compressorConfigType;

  private RobotContainer m_robotContainer;

  String trajectoryJSON = "insert json here"; //No path is loaded yet
  public static Trajectory trajectory = new Trajectory();
    

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    m_robotContainer.ledSubsystem.setDefaultCommand(new AuroraLEDCommand(m_robotContainer.ledSubsystem));
  }

  @Override
  public void disabledPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.ledSubsystem.setDefaultCommand(new AuroraLEDCommand(m_robotContainer.ledSubsystem));

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_robotContainer.ledSubsystem.getCurrentCommand().cancel();
    m_robotContainer.ledSubsystem.setDefaultCommand(new SetLEDYetiBlueCommand(m_robotContainer.ledSubsystem));

    CommandScheduler.getInstance().onCommandFinish(command -> {
      if (command.getName().equals(new BlinkLEDCommand().getName())) {
        if (beforeBlinkCommand != null) beforeBlinkCommand.schedule();
      }
    });
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    if (DriverStation.getMatchTime() < 30 && !blinkWarningRan) {
      beforeBlinkCommand = m_robotContainer.ledSubsystem.getCurrentCommand();
      new BlinkLEDCommand(m_robotContainer.ledSubsystem, 300, 255, 34, 0).schedule();
      blinkWarningRan = true;
    }
    
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}
