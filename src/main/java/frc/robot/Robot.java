// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CompressorConfigType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.LED.AuroraLEDCommand;
import frc.robot.commands.LED.BlinkLEDCommand;
import frc.robot.commands.LED.SetLEDToRGBCommand;
import frc.robot.commands.turret.HomeTurretCommand;
import frc.robot.commands.turret.TurretLockCommand;
import frc.robot.di.DaggerRobotComponent;
import frc.robot.di.RobotComponent;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem.TurretLockStatus;
import javax.inject.Inject;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command beforeBlinkCommand = null;
  private boolean blinkWarningRan = false;
  public CompressorConfigType compressorConfigType;
  private RobotComponent robotComponent;
  @Inject PowerDistribution revPDH;
  @Inject RobotContainer robotContainer;

  public static SendableChooser<AutoModes> autoChooser;
  private SetLEDToRGBCommand redLedCommand;
  private AuroraLEDCommand autoDisabledCommand;

  public static enum AutoModes {
    ONE_BALL,
    TWO_BALL,
    TWO_BALL_ALTERNATIVE,
    TWO_BALL_DUMP,
    THREE_BALL,
    FOUR_BALL,
    TEST_AUTO,
  }

  long timer;

  public Robot() {
    robotComponent = DaggerRobotComponent.builder().build();
    robotComponent.inject(this);
  }

  @Override
  public void robotInit() {
    // robotContainer = new RobotContainer();
    // revPDH = new PowerDistribution(1, ModuleType.kRev);
    redLedCommand = new SetLEDToRGBCommand(robotContainer.ledSubsystem, 255, 0, 0);
    autoDisabledCommand = new AuroraLEDCommand(robotContainer.ledSubsystem);
    robotContainer.turretSubsystem.lockStatus = TurretLockStatus.UNLOCKED;

    revPDH.setSwitchableChannel(false);
    timer = System.currentTimeMillis();

    UsbCamera driverCam = CameraServer.startAutomaticCapture();

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("ONE_BALL", AutoModes.ONE_BALL);
    autoChooser.addOption("ONE_BALL", AutoModes.ONE_BALL);
    autoChooser.addOption("TWO_BALL", AutoModes.TWO_BALL);
    autoChooser.addOption("TWO_BALL_ALTERNATIVE", AutoModes.TWO_BALL_ALTERNATIVE);
    autoChooser.addOption("THREE_BALL", AutoModes.THREE_BALL);
    autoChooser.addOption("FOUR_BALL", AutoModes.FOUR_BALL);
    autoChooser.addOption("TEST_AUTO", AutoModes.TEST_AUTO);
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (System.currentTimeMillis() - timer >= 500) {
      revPDH.setSwitchableChannel(true);
    }

    SmartDashboard.putNumber("Flywheel RPM: ", robotContainer.shooterSubsystem.getFlywheelRPM());
    SmartDashboard.putString(
        "Turret Lock Status: ",
        ((robotContainer.turretSubsystem.lockStatus == TurretLockStatus.UNLOCKED)
            ? "UNLOCKED"
            : "LOCKED"));
    SmartDashboard.putString(
        "Control Mode: ", (robotContainer.shooterMode) ? "SHOOTING" : "CLIMBING");

    // System.out.println("DIST: " + PhotonVision.getDistance() + "; setPoint: " +
    // ShooterSubsystem.setPoint);
    // System.out.println("getFlywheelRPM: " + robotContainer.shooterSubsystem.getFlywheelRPM());
    // System.out.println("GYRO: " + robotContainer.drivetrainSubsystem.getHeading());
  }

  @Override
  public void disabledInit() {
    // fixes flashing
    robotContainer.ledSubsystem.setDefaultCommand(
        new RunCommand(() -> {}, robotContainer.ledSubsystem));
  }

  @Override
  public void disabledPeriodic() {
    CommandScheduler.getInstance().run();
    if (robotContainer.turretSubsystem.getMagSwitch()) {
      autoDisabledCommand.schedule();
    } else {
      redLedCommand.schedule();
    }
  }

  @Override
  public void autonomousInit() {
    robotContainer.turretSubsystem.resetEncoder();
    Command currLEDCommand = robotContainer.ledSubsystem.getCurrentCommand();
    if (currLEDCommand != null) {
      currLEDCommand.cancel();
    }
    robotContainer.ledSubsystem.setDefaultCommand(autoDisabledCommand);
    m_autonomousCommand = robotContainer.getAutonomousCommand();

    SequentialCommandGroup turretAuto;
    switch ((Robot.AutoModes) autoChooser.getSelected()) {
      case TWO_BALL:
        turretAuto =
            new SequentialCommandGroup(
                new WaitCommand(8.0),
                new InstantCommand(
                    () -> robotContainer.turretSubsystem.lockStatus = TurretLockStatus.LOCKED));
        break;
      case FOUR_BALL:
        turretAuto =
            new SequentialCommandGroup(
                new WaitCommand(2.7),
                new InstantCommand(
                    () -> robotContainer.turretSubsystem.lockStatus = TurretLockStatus.LOCKED),
                new WaitCommand(5.0),
                new HomeTurretCommand(robotContainer.turretSubsystem, true),
                new WaitCommand(1.5),
                new InstantCommand(
                    () -> robotContainer.turretSubsystem.lockStatus = TurretLockStatus.LOCKED));
        break;
      default:
        turretAuto =
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> robotContainer.turretSubsystem.lockStatus = TurretLockStatus.LOCKED));
        break;
    }

    if (m_autonomousCommand != null) {
      new ParallelCommandGroup(
              m_autonomousCommand.alongWith(new TurretLockCommand(robotContainer.turretSubsystem)),
              turretAuto)
          .schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (robotContainer.turretSubsystem.getMagSwitch()) {
      robotContainer.turretSubsystem.resetEncoder();
    }
    robotContainer.climberSubsystem.resetEncoders();

    robotContainer.shooterMode = true;
    ShooterSubsystem.isShooting = false;
    TurretConstants.TURRET_OFFSET = 8.0;

    robotContainer.drivetrainSubsystem.resetEncoders();
    robotContainer.drivetrainSubsystem.resetGyro();

    autoDisabledCommand.cancel();
    robotContainer.ledSubsystem.setDefaultCommand(
        new AuroraLEDCommand(robotContainer.ledSubsystem));

    CommandScheduler.getInstance()
        .onCommandFinish(
            command -> {
              if (command.getName().equals(new BlinkLEDCommand().getName())) {
                if (beforeBlinkCommand != null) beforeBlinkCommand.schedule();
              }
            });

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    new HomeTurretCommand(robotContainer.turretSubsystem, true).schedule();
  }

  @Override
  public void teleopPeriodic() {
    if (DriverStation.getMatchTime() < 30 && !blinkWarningRan) {
      beforeBlinkCommand = robotContainer.ledSubsystem.getCurrentCommand();
      new BlinkLEDCommand(robotContainer.ledSubsystem, 300, 255, 34, 0).schedule();
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
