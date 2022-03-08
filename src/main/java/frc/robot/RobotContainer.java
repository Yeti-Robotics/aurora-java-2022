// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.climber.ClimbDownCommand;
import frc.robot.commands.climber.ClimbUpCommand;
import frc.robot.commands.climber.ToggleMovingHookCommand;
import frc.robot.commands.climber.ToggleStaticHooksCommand;
import frc.robot.commands.climber.WinchInCommand;
import frc.robot.commands.climber.WinchOutCommand;
import frc.robot.commands.commandgroups.AllInCommand;
import frc.robot.commands.commandgroups.AllOutCommand;
import frc.robot.commands.commandgroups.WinchInAndClimbDownCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.ToggleIntakeCommand;
import frc.robot.commands.neck.NeckInCommand;
import frc.robot.commands.shifting.ToggleShiftCommand;
import frc.robot.commands.shooter.FlywheelPIDCommand;
import frc.robot.commands.shooter.SpinShooterCommand;
import frc.robot.commands.shooter.SpinShooterVelocityCommand;
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

  private double lastInputLeftY = 0.0;
  private boolean mode = true;

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
    // setConditionalJoystickButtonWhileHeld(6, new AllOutCommand(intakeSubsystem, neckSubsystem), new AllInCommand(neckSubsystem, intakeSubsystem));
    setJoystickButtonWhileHeld(6, new AllOutCommand(intakeSubsystem, neckSubsystem));
    setJoystickButtonWhileHeld(1, new AllInCommand(neckSubsystem, intakeSubsystem));
    
    setJoystickButtonWhenPressed(7, new ToggleTurretLockCommand(turretSubsystem).andThen(new HomeTurretCommand(turretSubsystem)));
    setJoystickButtonWhileHeld(2, new FlywheelPIDCommand(shooterSubsystem));

    // setJoystickButtonWhenPressed(3, new StartEndCommand(() -> mode = !mode, () -> {}));
    setJoystickButtonWhileHeld(8, new WinchOutCommand(climberSubsystem));
    setJoystickButtonWhileHeld(3, new WinchInCommand(climberSubsystem));

    setJoystickButtonWhileHeld(9, new ClimbUpCommand(climberSubsystem));
    setJoystickButtonWhileHeld(4, new ClimbDownCommand(climberSubsystem));
    
    setJoystickButtonWhenPressed(10, new ToggleStaticHooksCommand(climberSubsystem));
    setJoystickButtonWhenPressed(5, new ToggleMovingHookCommand(climberSubsystem));

    setJoystickButtonWhenPressed(11, new ToggleShiftCommand(shiftingSubsystem));
    setJoystickButtonWhenPressed(12, new ToggleIntakeCommand(intakeSubsystem));
  }

  private double getLeftY() {
    // prevents tipping when stopping backward movement abruptly
    if(lastInputLeftY < 0 && Math.abs(-driverStationJoystick.getRawAxis(0)) <= 0.05){ // 0.05 == joystick deadband
      drivetrainSubsystem.setMotorsCoast();
    }

    if(Math.abs(-driverStationJoystick.getRawAxis(0)) > 0.05){
      drivetrainSubsystem.setMotorsBrake();
    }

    lastInputLeftY = -driverStationJoystick.getRawAxis(0);

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

  private void setJoystickButtonWhenPressed(int button, CommandBase command) {
    new JoystickButton(driverStationJoystick, button).whenPressed(command);
  }

  private void setConditionalJoystickButtonWhenPressed(int button, CommandBase command1, CommandBase command2) {
    new JoystickButton(driverStationJoystick, button).whenPressed(new ConditionalCommand(command1, command2, () -> mode));
  }

  private void setJoystickButtonWhileHeld(int button, CommandBase command) {
    new JoystickButton(driverStationJoystick, button).whileHeld(command);
  }

  private void setConditionalJoystickButtonWhileHeld(int button, Command command1, Command command2) {
    new JoystickButton(driverStationJoystick, button).whileHeld(new ConditionalCommand(command1, command2, () -> mode));
  }

  public Command getAutonomousCommand() {
    Trajectory customTrajectory = Robot.trajectory;

    RamseteCommand ramseteCommand = new RamseteCommand(
      customTrajectory, 
      drivetrainSubsystem::getPose,
      new RamseteController(AutoConstants.RAMSETE_B, AutoConstants.RAMSETE_ZETA),
      new SimpleMotorFeedforward(
        AutoConstants.AUTO_KS,
        AutoConstants.AUTO_KV,
        AutoConstants.AUTO_KA), 
      AutoConstants.KINEMATICS, 
      drivetrainSubsystem::getWheelSpeeds,
      new PIDController(AutoConstants.AUTO_P, 0, 0), 
      new PIDController(AutoConstants.AUTO_P, 0, 0), 
      drivetrainSubsystem::tankDriveVolts, 
      drivetrainSubsystem);

    drivetrainSubsystem.resetOdometry(customTrajectory.getInitialPose());
    return ramseteCommand.andThen(() -> drivetrainSubsystem.tankDriveVolts(0, 0));
  }

}