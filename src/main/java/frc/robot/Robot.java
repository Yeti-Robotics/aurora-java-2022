// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.CompressorConfigType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.LED.AuroraLEDCommand;
import frc.robot.commands.LED.BlinkLEDCommand;
import frc.robot.commands.LED.SetLEDToRGBCommand;
import frc.robot.commands.LED.TeleLEDDefaultCommand;
import frc.robot.commands.turret.HomeTurretCommand;
import frc.robot.commands.turret.SnapTurretRightCommand;
import frc.robot.commands.turret.TurretLockCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.TurretSubsystem.TurretLockStatus;
import frc.robot.utils.PhotonVision;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand;
	private Command beforeBlinkCommand = null;
	private boolean blinkWarningRan = false;
	public CompressorConfigType compressorConfigType;

	private RobotContainer robotContainer;

	public static SendableChooser<AutoModes> autoChooser;
	private SetLEDToRGBCommand redLedCommand;
	private AuroraLEDCommand auroraLedCommand;

	public static enum AutoModes {
		ONE_BALL, TWO_BALL, TWO_BALL_ALTERNATIVE, THREE_BALL, FOUR_BALL, TEST_AUTO
	}

	@Override
	public void robotInit() {
		robotContainer = new RobotContainer();
		redLedCommand = new SetLEDToRGBCommand(robotContainer.ledSubsystem, 255, 0, 0);
		auroraLedCommand = new AuroraLEDCommand(robotContainer.ledSubsystem);
		robotContainer.turretSubsystem.lockStatus = TurretLockStatus.UNLOCKED;

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
		SmartDashboard.putNumber("Current Pressure: ", robotContainer.pneumaticsSubsystem.getPressure());
		SmartDashboard.putNumber("Flywheel RPM: ", robotContainer.shooterSubsystem.getFlywheelRPM());
		SmartDashboard.putString("Turret Lock Status: ",
				((robotContainer.turretSubsystem.lockStatus == TurretLockStatus.UNLOCKED) ? "UNLOCKED" : "LOCKED"));
		SmartDashboard.putString("Control Mode: ", (robotContainer.shooterMode) ? "SHOOTING" : "CLIMBING");
		System.out.println("DIST: " + PhotonVision.getDistance() + "; setPoint: " + ShooterSubsystem.setPoint);
		System.out.println("getFlywheelRPM: " + robotContainer.shooterSubsystem.getFlywheelRPM());
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
		if (robotContainer.turretSubsystem.getMagSwitch()) {
			redLedCommand.cancel();
			auroraLedCommand.schedule();
		} else {
			auroraLedCommand.cancel();
			redLedCommand.schedule();
		}
	}

	@Override
	public void autonomousInit() {
		robotContainer.turretSubsystem.resetEncoder();
		robotContainer.ledSubsystem.setDefaultCommand(auroraLedCommand);
		m_autonomousCommand = robotContainer.getAutonomousCommand();

		SequentialCommandGroup turretAuto;
		switch((Robot.AutoModes) autoChooser.getSelected()){
			case FOUR_BALL:
				turretAuto = new SequentialCommandGroup(
					new WaitCommand(2.0), 
					new InstantCommand(() -> robotContainer.turretSubsystem.lockStatus = TurretLockStatus.LOCKED),
					new WaitCommand(5.0),
					new HomeTurretCommand(robotContainer.turretSubsystem, true), 
					new WaitCommand(2.5),
					new InstantCommand(() -> robotContainer.turretSubsystem.lockStatus = TurretLockStatus.LOCKED)
				);
				break;
			default: 
				turretAuto = new SequentialCommandGroup(new InstantCommand(() -> robotContainer.turretSubsystem.lockStatus = TurretLockStatus.LOCKED));
				break; 
		}

		

		if (m_autonomousCommand != null) {
			new ParallelCommandGroup(m_autonomousCommand.alongWith(new TurretLockCommand(robotContainer.turretSubsystem)), turretAuto).schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		if (robotContainer.turretSubsystem.getMagSwitch()) {
			robotContainer.turretSubsystem.resetEncoder();
		}
		robotContainer.climberSubsystem.resetEncoders();

		robotContainer.shooterMode = true;
		ShooterSubsystem.isShooting = false;

		robotContainer.drivetrainSubsystem.resetEncoders();
		robotContainer.drivetrainSubsystem.resetGyro();

		auroraLedCommand.cancel();
		robotContainer.ledSubsystem.setDefaultCommand(new TeleLEDDefaultCommand(robotContainer.ledSubsystem));

		CommandScheduler.getInstance().onCommandFinish(command -> {
			if (command.getName().equals(new BlinkLEDCommand().getName())) {
				if (beforeBlinkCommand != null)
					beforeBlinkCommand.schedule();
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
			new BlinkLEDCommand(robotContainer.ledSubsystem, 300, 255, 34,
					0).schedule();
			blinkWarningRan = true;
		}
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {
	}
}
