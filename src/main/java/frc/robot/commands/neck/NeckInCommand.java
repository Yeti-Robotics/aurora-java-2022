// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.neck;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NeckSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class NeckInCommand extends CommandBase {
  private NeckSubsystem neckSubsystem;
  private ShooterSubsystem shooterSubsystem;

  public NeckInCommand(NeckSubsystem neckSubsystem, ShooterSubsystem shooterSubsystem) {
    this.neckSubsystem = neckSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(neckSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(shooterSubsystem.getFlywheelRPM() > 10.0 || (shooterSubsystem.getFlywheelRPM() < 10.0 && !neckSubsystem.getUpperBeamBreak())){
      neckSubsystem.moveFrontUp();
    }
    neckSubsystem.moveRearUp();
  }

  @Override
  public void end(boolean interrupted) {
    neckSubsystem.stopNeck();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
