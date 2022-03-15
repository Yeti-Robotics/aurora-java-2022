// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climber.ToggleStaticHooksCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShiftingSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoMidClimbCommandGroup extends SequentialCommandGroup {
  
  private ClimberSubsystem climberSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private ShiftingSubsystem shiftingSubsystem;
  private Joystick joystick;

  public AutoMidClimbCommandGroup(
    ClimberSubsystem climberSubsystem,
    IntakeSubsystem intakeSubsystem,
    ShiftingSubsystem shiftingSubsystem,
    Joystick driverStationJoystick
  ) {
    this.climberSubsystem =  climberSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.shiftingSubsystem = shiftingSubsystem;
    joystick = driverStationJoystick;
    addCommands(
      new ConditionalCommand(new ToggleStaticHooksCommand(climberSubsystem), new InstantCommand(), () -> climberSubsystem.getStationaryPosition() == Value.kForward), 
      new InstantCommand(() -> intakeSubsystem.retract()), 
      new ClimbAndWinchInCommand(climberSubsystem, shiftingSubsystem), 
      new ToggleStaticHooksCommand(climberSubsystem)
    );
  }

  @Override
  public boolean isFinished() {
    return joystick.getRawButton(10);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    climberSubsystem.stopClimb();
  }
}
