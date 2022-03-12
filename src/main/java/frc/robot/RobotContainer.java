// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.*;
import frc.robot.utils.AutoBuilder;
import frc.robot.commands.climber.ClimbDownCommand;
import frc.robot.commands.climber.ClimbUpCommand;
import frc.robot.commands.climber.ToggleMovingHookCommand;
import frc.robot.commands.climber.ToggleStaticHooksCommand;
import frc.robot.commands.climber.WinchInCommand;
import frc.robot.commands.climber.WinchOutCommand;
import frc.robot.commands.commandgroups.AllInCommand;
import frc.robot.commands.commandgroups.AllOutCommand;
import frc.robot.commands.intake.ToggleIntakeCommand;
import frc.robot.commands.shifting.ToggleShiftCommand;
import frc.robot.commands.shooter.ToggleFlywheelPIDCommand;
import frc.robot.commands.turret.HomeTurretCommand;
import frc.robot.commands.turret.ToggleTurretLockCommand;
import frc.robot.commands.turret.TurretLockCommand;
import frc.robot.utils.JoyButton;
import frc.robot.utils.JoyButton.ActiveState;

import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
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

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
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
                drivetrainSubsystem.setDefaultCommand(
                        new RunCommand(() -> drivetrainSubsystem.tankDrive(getLeftY(), getRightY()),
                                drivetrainSubsystem));
                break;
            case CHEEZY:
                drivetrainSubsystem.setDefaultCommand(
                        new RunCommand(() -> drivetrainSubsystem.cheezyDrive(getLeftY(), getRightX()),
                                drivetrainSubsystem));
                break;
            case ARCADE:
                drivetrainSubsystem.setDefaultCommand(
                        new RunCommand(() -> drivetrainSubsystem.arcadeDrive(getLeftY(), getRightX()),
                                drivetrainSubsystem));
                break;
        }

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        setConditionalButton(1, new AllInCommand(neckSubsystem, intakeSubsystem), ActiveState.WHILE_HELD, new ToggleIntakeCommand(intakeSubsystem), ActiveState.WHEN_PRESSED);
        setJoystickButtonWhenPressed(5, new StartEndCommand(() -> mode = !mode, () -> {}));
    }

    private double getLeftY() {
        // prevents tipping when stopping backward movement abruptly
        if (lastInputLeftY < 0 && Math.abs(-driverStationJoystick.getRawAxis(0)) <= 0.05) { // 0.05 == joystick deadband
            drivetrainSubsystem.setMotorsCoast();
        }

        if (Math.abs(-driverStationJoystick.getRawAxis(0)) > 0.05) {
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
        new JoystickButton(driverStationJoystick, button)
                .whenPressed(new ConditionalCommand(command1, command2, () -> mode));
    }

    private void setJoystickButtonWhileHeld(int button, CommandBase command) {
        new JoystickButton(driverStationJoystick, button).whileHeld(command);
    }

    private void setConditionalJoystickButtonWhileHeld(int button, Command command1, Command command2) {
        new JoystickButton(driverStationJoystick, button)
                .whileHeld(new ConditionalCommand(command1, command2, () -> mode));
    }

    private void setConditionalButton(
            int button,
            Command trueCommand,
            ActiveState trueActiveState,
            Command falseCommand,
            ActiveState falseActiveState
    ) {
        new JoyButton(driverStationJoystick, button)
                .conditionalPressed(trueCommand, trueActiveState, falseCommand, falseActiveState, () -> mode);
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    drivetrainSubsystem.setMotorsBrake();

    AutoBuilder builder = new AutoBuilder(); 
    builder.setRobotContainer(this);
    builder.setAutoMode(Robot.autoChooser.getSelected());
    return builder.build();
  }
    // SequentialCommandGroup twoBallAutoCommand = new SequentialCommandGroup(new
    // WaitCommand(1),
    // new IntakeInCommand(intakeSubsystem).withTimeout(1), new
    // NeckInCommand(neckSubsystem).withTimeout(1),
    // new WaitCommand(1));

    // SequentialCommandGroup threeBallAutoCommand = new SequentialCommandGroup(new
    // WaitCommand(1),
    // new IntakeInCommand(intakeSubsystem).withTimeout(1), new
    // NeckInCommand(neckSubsystem).withTimeout(1),
    // new WaitCommand(1));

    // SequentialCommandGroup fourBallAutoCommand = new SequentialCommandGroup(new
    // WaitCommand(1),
    // new IntakeInCommand(intakeSubsystem).withTimeout(1), new
    // NeckInCommand(neckSubsystem).withTimeout(1),
    // new WaitCommand(1), new IntakeInCommand(intakeSubsystem).withTimeout(1),
    // new NeckInCommand(neckSubsystem).withTimeout(1), new WaitCommand(1));

    // new PathFollowingCommand(drivetrainSubsystem,
    // AutoConstants.twoBallPrimary).alongWith(twoBallAutoCommand),
    // new RunCommand(() -> drivetrainSubsystem.tankDriveVolts(0, 0),
    // drivetrainSubsystem),
    // new SpinShooterCommand(shooterSubsystem, 0.5).withTimeout(1),
    // new SequentialCommandGroup(
    // new PathFollowingCommand(drivetrainSubsystem, AutoConstants.threeBallPrimary)
    // .alongWith(threeBallAutoCommand),
    // new RunCommand(() -> drivetrainSubsystem.tankDriveVolts(0, 0),
    // drivetrainSubsystem),
    // new SpinShooterCommand(shooterSubsystem, 0.5).withTimeout(1)));
}
