// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.LED.ShooterLEDCommand;
import frc.robot.commands.drivetrain.DriveForDistanceCommand;
import frc.robot.commands.drivetrain.TurnForAngleCommand;
import frc.robot.commands.intake.ToggleIntakeCommand;
import frc.robot.commands.shooter.ToggleFlywheelHighCommand;
import frc.robot.commands.turret.TurretLockCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NeckSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class DeadReckon2BallAuto extends CommandBase {

  private final IntakeSubsystem intakeSubsystem;
  private final NeckSubsystem neckSubsystem;
  private final DrivetrainSubsystem drivetrainSubsystem;
  private TurretSubsystem turretSubsystem;
  private ShooterLEDCommand shooterLEDCommand;

  /** Creates a new AllOutCommand. */
  public DeadReckon2BallAuto(
      IntakeSubsystem intakeSubsystem,
      NeckSubsystem neckSubsystem,
      DrivetrainSubsystem drivetrainSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.neckSubsystem = neckSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(neckSubsystem, intakeSubsystem, drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new ToggleIntakeCommand(intakeSubsystem);
    new DriveForDistanceCommand(drivetrainSubsystem, 1.143, 50);
    new AllInCommand(intakeSubsystem, neckSubsystem).withTimeout(1);
    new ToggleIntakeCommand(intakeSubsystem);
    new TurnForAngleCommand(drivetrainSubsystem, 160);
    new DriveForDistanceCommand(drivetrainSubsystem, 0.5715, 50);
    new TurretLockCommand(turretSubsystem);
    new ToggleFlywheelHighCommand(shooterLEDCommand);
    new WaitCommand(0.75);
    new AllInCommand(intakeSubsystem, neckSubsystem).withTimeout(0.3);
    new WaitCommand(0.5);
    new AllInCommand(intakeSubsystem, neckSubsystem).withTimeout(0.7);
    new ToggleFlywheelHighCommand(shooterLEDCommand);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
