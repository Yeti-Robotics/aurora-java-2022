// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.CompressorConfigType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.LED.AuroraLEDCommand;
import frc.robot.commands.LED.BlinkLEDCommand;
import frc.robot.commands.LED.SetLEDToRGBCommand;
import frc.robot.commands.LED.SetLEDYetiBlueCommand;
import frc.robot.commands.turret.HomeTurretCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterStatus;
import frc.robot.subsystems.TurretSubsystem.TurretLockStatus;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand;
	private Command beforeBlinkCommand = null;
	private boolean blinkWarningRan = false;
	public CompressorConfigType compressorConfigType;

	private RobotContainer robotContainer;

	private String trajectoryJSON = "insert json here"; //No path is loaded yet
	public static Trajectory trajectory = new Trajectory();

	@Override
	public void robotInit() {
		robotContainer = new RobotContainer();
		robotContainer.turretSubsystem.lockStatus = TurretLockStatus.UNLOCKED;

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
		SmartDashboard.putNumber("Current Pressure: ", robotContainer.pneumaticsSubsystem.getPressure());
		SmartDashboard.putNumber("Flywheel RPM: ", robotContainer.shooterSubsystem.getFlywheelRPM());
		SmartDashboard.putString("Turret Lock Status: ", ((robotContainer.turretSubsystem.lockStatus == robotContainer.turretSubsystem.lockStatus.UNLOCKED) ? "UNLOCKED" : "LOCKED"));
		SmartDashboard.putBoolean("ShooterSubsystem.isShooting: ", ShooterSubsystem.isShooting);
		// System.out.println("LIMELIGHT TX: " + Limelight.getTx());
		// System.out.println("Mag switch: " + robotContainer.turretSubsystem.getMagSwitch());
	}

	@Override
	public void disabledInit() {
		robotContainer.ledSubsystem.setDefaultCommand(new AuroraLEDCommand(robotContainer.ledSubsystem));
	}

	@Override
	public void disabledPeriodic() {
		CommandScheduler.getInstance().run();
		SetLEDToRGBCommand redLedCommand = new SetLEDToRGBCommand(robotContainer.ledSubsystem, 255, 0, 0);
		if (robotContainer.turretSubsystem.getMagSwitch()) {
			redLedCommand.cancel();
		} else {
			redLedCommand.schedule();
		}

	}

	@Override
	public void autonomousInit() {
		robotContainer.ledSubsystem.setDefaultCommand(new AuroraLEDCommand(robotContainer.ledSubsystem));
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		robotContainer.turretSubsystem.resetEncoder();
		robotContainer.climberSubsystem.resetEncoders();
		robotContainer.ledSubsystem.getCurrentCommand().cancel();
		robotContainer.ledSubsystem.setDefaultCommand(new SetLEDYetiBlueCommand(robotContainer.ledSubsystem));

		CommandScheduler.getInstance().onCommandFinish(command -> {
			if (command.getName().equals(new BlinkLEDCommand().getName())) {
				if (beforeBlinkCommand != null)
					beforeBlinkCommand.schedule();
			}
		});
		robotContainer.climberSubsystem.resetEncoders();
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
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
