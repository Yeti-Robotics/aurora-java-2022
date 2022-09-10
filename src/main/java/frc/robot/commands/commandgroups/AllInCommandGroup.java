// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NeckSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AllInCommandGroup extends SequentialCommandGroup {
  IntakeSubsystem intakeSubsystem;
  NeckSubsystem neckSubsystem;

  public AllInCommandGroup(IntakeSubsystem intakeSubsystem, NeckSubsystem neckSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.neckSubsystem = neckSubsystem;
    addCommands(
        new InstantCommand(
            () -> {
              intakeSubsystem.stopRoll();
              neckSubsystem.stopNeck();
            }),
        new ConditionalCommand(
            new RunCommand(
                    () -> {
                      neckSubsystem.moveUp(0.8);
                    })
                .andThen(
                    new ConditionalCommand(
                        new WaitCommand(0.25),
                        new InstantCommand(),
                        () -> !neckSubsystem.getUpperBeamBreak())),
            new RunCommand(
                () -> {
                  intakeSubsystem.rollIn();
                  if (neckSubsystem.getLowerBeamBreak()) neckSubsystem.moveUp(0.3);
                }),
            () -> ShooterSubsystem.atSetPoint()));
  }

  @Override
  public void end(boolean interrupted) {
    // TODO Auto-generated method stub
    intakeSubsystem.stopRoll();
    neckSubsystem.stopNeck();
  }
}
