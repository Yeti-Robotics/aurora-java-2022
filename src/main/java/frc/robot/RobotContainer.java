// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.commands.intake.ToggleIntakeCommand;
import frc.robot.commands.neck.NeckInCommand;
import frc.robot.commands.neck.NeckOutCommand;
import frc.robot.commands.shifting.ToggleShiftCommand;
import frc.robot.commands.shooter.SpinShooterCommand;
import frc.robot.commands.turret.MoveTurretCommand;
import frc.robot.commands.turret.TurretLockCommand;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private Joystick driverStationJoystick;
  private DrivetrainSubsystem drivetrainSubsystem;
  private ShiftingSubsystem shiftingSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private NeckSubsystem neckSubsystem;
  private TurretSubsystem turretSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private ClimberSubsystem climberSubsystem;
  public PneumaticsSubsystem pneumaticsSubsystem;
  public final LEDSubsystem ledSubsystem;
  private boolean buttonMode = true;

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
    pneumaticsSubsystem = new PneumaticsSubsystem();
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
    setJoystickButtonWhenPressed(driverStationJoystick, 10, new CommandBase() {
      public void initialize() {
        buttonMode = !buttonMode;
        configureButtonBindings();
      }

      public boolean isFinished() {
        return true;
      }
    });
    setJoystickButtonWhileHeld(driverStationJoystick, 1, new NeckInCommand(neckSubsystem, shooterSubsystem));
    setJoystickButtonWhileHeld(driverStationJoystick, 2, new SpinShooterCommand(shooterSubsystem, ShooterConstants.SHOOTER_SPEED));
    setJoystickButtonWhileHeld(driverStationJoystick, 3, buttonMode ? new IntakeInCommand(intakeSubsystem) : new IntakeOutCommand(intakeSubsystem));
    setJoystickButtonWhileHeld(driverStationJoystick, 4, new MoveTurretCommand(turretSubsystem, 0.1));

    setJoystickButtonWhileHeld(driverStationJoystick, 6, new NeckOutCommand(neckSubsystem));
    
    setJoystickButtonWhileHeld(driverStationJoystick, 9, new MoveTurretCommand(turretSubsystem, -0.1));
    setJoystickButtonWhenPressed(driverStationJoystick, 11, new ToggleShiftCommand(shiftingSubsystem));
    setJoystickButtonWhenPressed(driverStationJoystick, 12, new ToggleIntakeCommand(intakeSubsystem));
  }

  private double getLeftY() {
    return -driverStationJoystick.getRawAxis(0);
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
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
/*  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
*/
}
