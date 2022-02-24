// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoRoutines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.neck.NeckInCommand;
import frc.robot.commands.shooter.SpinShooterCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.NeckSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TwoBallAutoCommand extends SequentialCommandGroup {  
  /** Creates a new twoBallAutoCommand. */
  public DrivetrainSubsystem drivetrainSubsystem;
  public IntakeSubsystem intakeSubsystem;
  public NeckSubsystem neckSubsystem;
  public TurretSubsystem turretSubsystem;
  public ShooterSubsystem shooterSubsystem;
  public LEDSubsystem ledSubsystem;
  public Command SpinShooterCommand;
  public Command NeckInCommand;
  public Command IntakeInCommand;

  public TwoBallAutoCommand(DrivetrainSubsystem drivetrainSubsystem, IntakeSubsystem intakeSubsystem, NeckSubsystem neckSubsystem, 
  TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem, LEDSubsystem ledSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.neckSubsystem = neckSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.ledSubsystem = ledSubsystem;
    addRequirements(drivetrainSubsystem);
    addRequirements(intakeSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      //pathfollowitgcommand,
      new SequentialCommandGroup(new WaitCommand(1), IntakeInCommand,  NeckInCommand, new WaitCommand(1), SpinShooterCommand, new WaitCommand(0.1), SpinShooterCommand); // do the waits stuffs 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
