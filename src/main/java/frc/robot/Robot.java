// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.CompressorConfigType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.LED.AuroraLEDCommand;
import frc.robot.commands.LED.BlinkLEDCommand;
import frc.robot.commands.LED.SetLEDYetiBlueCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand;
	private Command beforeBlinkCommand = null;
	private boolean blinkWarningRan = false;
	public CompressorConfigType compressorConfigType;

	private RobotContainer robotContainer;
	private SimpleMotorFeedforward shooterFeedForward;

	public Robot() {
		shooterFeedForward = new SimpleMotorFeedforward(ShooterConstants.SHOOTER_KS, ShooterConstants.SHOOTER_KV, ShooterConstants.SHOOTER_KA);
		addPeriodic(() -> {
			if (ShooterSubsystem.isShooting) {
				// robotContainer.shooterSubsystem.shootFlywheel(shooterFeedForward.calculate(robotContainer.shooterSubsystem.getVelocityUnitsFromRPM(ShooterSubsystem.setPoint) / 10.0));
				// double error = Math.abs(robotContainer.shooterSubsystem.setPoint - robotContainer.shooterSubsystem.getFlywheelRPM()); // perhaps try scaling based on error?
				double RPM = robotContainer.shooterSubsystem.getFlywheelRPM();
				double setPoint = ShooterSubsystem.setPoint;
				if(RPM > setPoint){
					robotContainer.shooterSubsystem.shootFlywheel(0.0);
				} else {
					robotContainer.shooterSubsystem.shootFlywheel(ShooterConstants.SHOOTER_SPEED);
				}
			} else {
				robotContainer.shooterSubsystem.stopFlywheel();
			}
		}, 0.01, 0.005); // every 10ms with a 5ms offset so timing doesn't conflict with robotPeriodic // (every 20ms)
	}

	@Override
	public void robotInit() {
		robotContainer = new RobotContainer();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		SmartDashboard.putNumber("Current Pressure: ", robotContainer.pneumaticsSubsystem.getPressure());
		SmartDashboard.putNumber("Flywheel RPM: ", robotContainer.shooterSubsystem.getFlywheelRPM());
		// System.out.println("LIMELIGHT TX: " + Limelight.getTx());
	}

	@Override
	public void disabledInit() {
		robotContainer.ledSubsystem.setDefaultCommand(new AuroraLEDCommand(robotContainer.ledSubsystem));
	}

	@Override
	public void disabledPeriodic() {
		CommandScheduler.getInstance().run();
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
