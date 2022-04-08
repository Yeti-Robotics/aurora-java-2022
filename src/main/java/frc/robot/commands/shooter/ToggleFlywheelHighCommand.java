// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.LED.ShooterLEDCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToggleFlywheelHighCommand extends InstantCommand {
  private final ShooterLEDCommand shooterLEDCommand;

  public ToggleFlywheelHighCommand(ShooterLEDCommand shooterLEDCommand) {
    this.shooterLEDCommand = shooterLEDCommand;
  }

  @Override
  public void initialize() {
    if (!ShooterSubsystem.isShooting) {
      shooterLEDCommand.schedule();
    } else {
      shooterLEDCommand.cancel();
    }
    ShooterSubsystem.shooterMode = ShooterMode.LIMELIGHT;
    ShooterSubsystem.isShooting = !ShooterSubsystem.isShooting;
  }
}
