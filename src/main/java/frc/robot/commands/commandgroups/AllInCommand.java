// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NeckSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AllInCommand extends CommandBase {

  private final NeckSubsystem neckSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private long startTime;
  private final double intakeSpeed;

  public AllInCommand(
      IntakeSubsystem intakeSubsystem, NeckSubsystem neckSubsystem, double intakeSpeed) {
    this.neckSubsystem = neckSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.intakeSpeed = intakeSpeed;
    addRequirements(neckSubsystem, intakeSubsystem);
  }

  public AllInCommand(IntakeSubsystem intakeSubsystem, NeckSubsystem neckSubsystem) {
    this.neckSubsystem = neckSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.intakeSpeed = IntakeConstants.INTAKE_SPEED;
    addRequirements(neckSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.rollIn(intakeSpeed);
    neckSubsystem.stopNeck();

    if (ShooterSubsystem.atSetPoint) {
      // if (!neckSubsystem.getUpperBeamBreak() && startTime == 0) {
      // 	startTime = System.currentTimeMillis();
      // 	neckSubsystem.moveUp(0.8);
      // }

      // if (System.currentTimeMillis() - startTime >= 500) {
      // 	neckSubsystem.moveUp(0.8);
      // 	startTime = 0;
      // }
      neckSubsystem.moveUp(0.6);
    } else if (neckSubsystem.getLowerBeamBreak()) {
      neckSubsystem.moveUp(0.3);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    neckSubsystem.stopNeck();
    intakeSubsystem.stopRoll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
