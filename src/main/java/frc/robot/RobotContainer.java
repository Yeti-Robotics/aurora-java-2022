// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.climber.ClimbDownCommand;
import frc.robot.commands.climber.ClimbUpCommand;
import frc.robot.commands.climber.ToggleMovingHookCommand;
import frc.robot.commands.climber.ToggleStaticHooksCommand;
import frc.robot.commands.commandgroups.AllInCommand;
import frc.robot.commands.commandgroups.AllOutCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.ToggleIntakeCommand;
import frc.robot.commands.neck.NeckInCommand;
import frc.robot.commands.shifting.ToggleShiftCommand;
import frc.robot.commands.shooter.FlywheelPIDCommand;
import frc.robot.commands.shooter.SpinShooterCommand;
import frc.robot.commands.shooter.SpinShooterVelocityCommand;
import frc.robot.commands.shooter.ToggleBangBangCommand;
import frc.robot.commands.turret.HomeTurretCommand;
import frc.robot.commands.turret.MoveTurretCommand;
import frc.robot.commands.turret.ToggleTurretLockCommand;
import frc.robot.commands.turret.TurnToTargetCommand;
import frc.robot.commands.turret.TurretLockCommand;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public Joystick driverStationJoystick;
  public DrivetrainSubsystem drivetrainSubsystem;
  public ShiftingSubsystem shiftingSubsystem;
  public IntakeSubsystem intakeSubsystem;
  public NeckSubsystem neckSubsystem;
  public TurretSubsystem turretSubsystem;
  public ShooterSubsystem shooterSubsystem;
  public ClimberSubsystem climberSubsystem;
  public PneumaticSubsystem pneumaticsSubsystem;
  public LEDSubsystem ledSubsystem;

  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    ledSubsystem = new LEDSubsystem();
    driverStationJoystick = new Joystick(OIConstants.DRIVER_STATION_JOY);
    shiftingSubsystem = new ShiftingSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    neckSubsystem = new NeckSubsystem();
    turretSubsystem = new TurretSubsystem();
    shooterSubsystem = new ShooterSubsystem();
    climberSubsystem = new ClimberSubsystem();
    drivetrainSubsystem = new DrivetrainSubsystem();
    pneumaticsSubsystem = new PneumaticSubsystem();
    
    turretSubsystem.setDefaultCommand(new TurretLockCommand(turretSubsystem));

    switch (drivetrainSubsystem.getDriveMode()) {
      case TANK:
        drivetrainSubsystem.setDefaultCommand(new RunCommand(() -> drivetrainSubsystem.tankDrive(getLeftY(), getRightY()), drivetrainSubsystem));
        break;
      case CHEEZY:
        drivetrainSubsystem.setDefaultCommand(new RunCommand(() -> drivetrainSubsystem.cheezyDrive(getLeftY(), getRightX()), drivetrainSubsystem));
        break;
      case ARCADE:
        drivetrainSubsystem.setDefaultCommand(new RunCommand(() -> drivetrainSubsystem.arcadeDrive(getLeftY(), getRightX()), drivetrainSubsystem));
        break;
    }
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    setJoystickButtonWhileHeld(driverStationJoystick, 1, new AllInCommand(neckSubsystem, intakeSubsystem));
    setJoystickButtonWhileHeld(driverStationJoystick, 2, new SpinShooterCommand(shooterSubsystem, ShooterConstants.SHOOTER_SPEED));
    setJoystickButtonWhileHeld(driverStationJoystick, 3, new IntakeInCommand(intakeSubsystem));

    // setJoystickButtonWhenPressed(driverStationJoystick, 4, new ToggleTurretLockCommand(turretSubsystem));
    // setJoystickButtonWhileHeld(driverStationJoystick, 4, new TurnToTargetCommand(turretSubsystem, 0.2));
    setJoystickButtonWhileHeld(driverStationJoystick, 4, new MoveTurretCommand(turretSubsystem, -TurretConstants.TURRET_SPEED));
    // setJoystickButtonWhileHeld(driverStationJoystick, 4, new ClimbDownCommand(climberSubsystem));
    
    setJoystickButtonWhenPressed(driverStationJoystick, 5, new ToggleStaticHooksCommand(climberSubsystem));
    setJoystickButtonWhileHeld(driverStationJoystick, 6, new AllOutCommand(intakeSubsystem, neckSubsystem));
    // setJoystickButtonWhileHeld(driverStationJoystick, 7, new FlywheelPIDCommand(shooterSubsystem));
    // setJoystickButtonWhenPressed(driverStationJoystick, 7, new ToggleBangBangCommand(shooterSubsystem));
    setJoystickButtonWhileHeld(driverStationJoystick, 8, new SpinShooterVelocityCommand(shooterSubsystem, shooterSubsystem.getVelocityUnitsFromRPM(ShooterSubsystem.setPoint)));

    // setJoystickButtonWhenPressed(driverStationJoystick, 9, new HomeTurretCommand(turretSubsystem));
    setJoystickButtonWhileHeld(driverStationJoystick, 9, new MoveTurretCommand(turretSubsystem, TurretConstants.TURRET_SPEED));
    // setJoystickButtonWhileHeld(driverStationJoystick, 9, new ClimbUpCommand(climberSubsystem));
    
    setJoystickButtonWhenPressed(driverStationJoystick, 10, new ToggleMovingHookCommand(climberSubsystem));
    setJoystickButtonWhenPressed(driverStationJoystick, 11, new ToggleShiftCommand(shiftingSubsystem));
    setJoystickButtonWhenPressed(driverStationJoystick, 12, new ToggleIntakeCommand(intakeSubsystem));
  }

  private double getLeftY() {
    // prevents tipping when stopping backward movement abruptly
    double driveVel = drivetrainSubsystem.getVelocity();
    double joyInput = -driverStationJoystick.getRawAxis(0);

    if(drivetrainSubsystem.getNeutralMode() == NeutralMode.Brake && driveVel < 0.0 && joyInput >= 0.0){
      drivetrainSubsystem.setMotorsCoast();
    }

    if(drivetrainSubsystem.getNeutralMode() == NeutralMode.Coast && driveVel >= 0.0){
      drivetrainSubsystem.setMotorsBrake();
    }

    return joyInput;
  }

  private double getLeftX() {
    return driverStationJoystick.getRawAxis(1);
  }

  private double getRightY() {
    return -driverStationJoystick.getRawAxis(2);
  }

  private double getRightX() {
    return driverStationJoystick.getRawAxis(3);
  }

  private void setJoystickButtonWhenPressed(Joystick driverStationJoystick, int button, CommandBase command) {
    new JoystickButton(driverStationJoystick, button).whenPressed(command);
  }

  private void setJoystickButtonWhileHeld(Joystick driverStationJoystick, int button, CommandBase command) {
    new JoystickButton(driverStationJoystick, button).whileHeld(command);
  }
}