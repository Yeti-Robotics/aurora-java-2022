// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToggleFlywheelLowCommand extends InstantCommand {
  public ToggleFlywheelLowCommand() {}

  @Override
  public void initialize() {
    ShooterSubsystem.isShooting = !ShooterSubsystem.isShooting;
    ShooterSubsystem.isHighGoal = false;
  }
}
