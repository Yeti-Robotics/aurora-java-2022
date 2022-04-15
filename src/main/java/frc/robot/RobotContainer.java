// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.LED.ShooterLEDCommand;
import frc.robot.commands.climber.ClimbDownCommand;
import frc.robot.commands.climber.ClimbUpCommand;
import frc.robot.commands.commandgroups.AllInCommand;
import frc.robot.commands.commandgroups.AllOutCommand;
import frc.robot.commands.intake.ToggleIntakeCommand;
import frc.robot.commands.shifting.ToggleShiftCommand;
import frc.robot.commands.shooter.ToggleFlywheelHighCommand;
import frc.robot.commands.shooter.ToggleFlywheelLowCommand;
import frc.robot.commands.turret.HomeTurretCommand;
import frc.robot.commands.turret.SnapTurretLeftCommand;
import frc.robot.commands.turret.SnapTurretRightCommand;
import frc.robot.commands.turret.ToggleTurretLockCommand;
import frc.robot.commands.turret.TurretLockCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.NeckSubsystem;
import frc.robot.subsystems.ShiftingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterMode;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.AutoBuilder;
import frc.robot.utils.JoyButton;
import frc.robot.utils.JoyButton.ActiveState;

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
    public LEDSubsystem ledSubsystem;

    private double lastInputLeftY = 0.0;
    public boolean shooterMode = true; // false = turretMode

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        ledSubsystem = new LEDSubsystem();
        driverStationJoystick = new Joystick(OIConstants.DRIVER_STATION_JOY);
        intakeSubsystem = new IntakeSubsystem();
        shiftingSubsystem = new ShiftingSubsystem();
        neckSubsystem = new NeckSubsystem();
        turretSubsystem = new TurretSubsystem();
        shooterSubsystem = new ShooterSubsystem();
        climberSubsystem = new ClimberSubsystem();
        drivetrainSubsystem = new DrivetrainSubsystem();

        turretSubsystem.setDefaultCommand(new TurretLockCommand(turretSubsystem));

        switch (drivetrainSubsystem.getDriveMode()) {
            case TANK:
                drivetrainSubsystem.setDefaultCommand(
                        new RunCommand(
                                () -> drivetrainSubsystem.tankDrive(getLeftY(), getRightY()),
                                drivetrainSubsystem));
                break;
            case CHEEZY:
                drivetrainSubsystem.setDefaultCommand(
                        new RunCommand(
                                () -> drivetrainSubsystem.cheezyDrive(getLeftY(), getRightX()),
                                drivetrainSubsystem));
                break;
            case ARCADE:
                drivetrainSubsystem.setDefaultCommand(
                        new RunCommand(
                                () -> drivetrainSubsystem.arcadeDrive(getLeftY(), getRightX()),
                                drivetrainSubsystem));
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
        ShooterLEDCommand shooterLEDCommand = new ShooterLEDCommand(ledSubsystem);
        setJoystickButtonWhenPressed(12, new ToggleIntakeCommand(intakeSubsystem));
        setConditionalButton(
                11,
                new ToggleShiftCommand(shiftingSubsystem),
                ActiveState.WHEN_PRESSED,
                new InstantCommand(),
                ActiveState.WHEN_PRESSED);

        setConditionalButton(
                6,
                new AllOutCommand(intakeSubsystem, neckSubsystem),
                ActiveState.WHILE_HELD,
                new ClimbUpCommand(climberSubsystem),
                ActiveState.WHILE_HELD);
        setConditionalButton(
                1,
                new AllInCommand(intakeSubsystem, neckSubsystem),
                ActiveState.WHILE_HELD,
                new ClimbDownCommand(climberSubsystem),
                ActiveState.WHILE_HELD);

        setConditionalButton(
                7,
                new ToggleTurretLockCommand(turretSubsystem)
                        .andThen(new HomeTurretCommand(turretSubsystem, false)),
                ActiveState.WHEN_PRESSED,
                new InstantCommand(),
                ActiveState.WHILE_HELD);
        setConditionalButton(
                2,
                new ToggleFlywheelHighCommand(shooterLEDCommand),
                ActiveState.WHEN_PRESSED,
                new InstantCommand(),
                ActiveState.WHILE_HELD);

        setConditionalButton(
                8,
                new HomeTurretCommand(turretSubsystem, true),
                ActiveState.WHEN_PRESSED,
                new InstantCommand(() -> climberSubsystem.toggleClimberBrake()),
                ActiveState.WHEN_PRESSED);
        setConditionalButton(
                3,
                new ToggleFlywheelLowCommand(shooterLEDCommand),
                ActiveState.WHEN_PRESSED,
                new InstantCommand(),
                ActiveState.WHEN_PRESSED);

        setConditionalButton(
                9,
                new SnapTurretLeftCommand(turretSubsystem),
                ActiveState.WHEN_PRESSED,
                new InstantCommand(),
                ActiveState.WHEN_PRESSED);
        setConditionalButton(
                4,
                new InstantCommand(
                        () -> {
                            ShooterSubsystem.shooterMode = ShooterMode.LAUNCHPAD;
                            ShooterSubsystem.isShooting = !ShooterSubsystem.isShooting;
                        }),
                ActiveState.WHEN_PRESSED,
                new InstantCommand(),
                ActiveState.WHILE_HELD);

        // 10 = kill switch for climbing
        setConditionalButton(
                10,
                new SnapTurretRightCommand(turretSubsystem),
                ActiveState.WHEN_PRESSED,
                new InstantCommand(),
                ActiveState.WHEN_PRESSED);
        setJoystickButtonWhenPressed(5, new InstantCommand(() -> shooterMode = !shooterMode));
    }

    private double getLeftY() {
        // prevents tipping when stopping backward movement abruptly
        if (lastInputLeftY < 0
                && Math.abs(-driverStationJoystick.getRawAxis(0))
                        <= 0.05) { // 0.05 == joystick deadband
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

    // commandOnTrue runs when shooterMode is true
    private void setConditionalJoystickButtonWhenPressed(
            int button, Command commandOnTrue, Command commandOnFalse) {
        new JoystickButton(driverStationJoystick, button)
                .whenPressed(
                        new ConditionalCommand(commandOnTrue, commandOnFalse, () -> shooterMode));
    }

    private void setJoystickButtonWhileHeld(int button, CommandBase command) {
        new JoystickButton(driverStationJoystick, button).whileHeld(command);
    }

    // commandOnTrue runs when shooterMode is true
    private void setConditionalJoystickButtonWhileHeld(
            int button, Command commandOnTrue, Command commandOnFalse) {
        new JoystickButton(driverStationJoystick, button)
                .whileHeld(
                        new ConditionalCommand(commandOnTrue, commandOnFalse, () -> shooterMode));
    }

    private void setConditionalButton(
            int button,
            Command trueCommand,
            ActiveState trueActiveState,
            Command falseCommand,
            ActiveState falseActiveState) {
        new JoyButton(driverStationJoystick, button)
                .conditionalPressed(
                        trueCommand,
                        trueActiveState,
                        falseCommand,
                        falseActiveState,
                        () -> shooterMode);
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
}
