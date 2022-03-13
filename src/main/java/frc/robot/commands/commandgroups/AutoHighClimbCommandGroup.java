package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.climber.ToggleStaticHooksCommand;
import frc.robot.commands.climber.WinchOutCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShiftingSubsystem;
import frc.robot.utils.JoyButton;

public class AutoHighClimbCommandGroup extends SequentialCommandGroup {
    private final Joystick joystick;

    public AutoHighClimbCommandGroup(
        ClimberSubsystem climberSubsystem,
        IntakeSubsystem intakeSubsystem,
        ShiftingSubsystem shiftingSubsystem,
        DrivetrainSubsystem drivetrainSubsystem, 
        Joystick driverStationJoystick
    ) {
        this.joystick = driverStationJoystick;
        addCommands(
          new AutoMidClimbCommandGroup(climberSubsystem, intakeSubsystem, shiftingSubsystem, driverStationJoystick),
          new WinchOutCommand(climberSubsystem).withTimeout(1),
          new ClimbAndWinchInCommand(climberSubsystem, shiftingSubsystem).withTimeout(0.5),
          new ToggleStaticHooksCommand(climberSubsystem),
          new ParallelDeadlineGroup(new WaitUntilCommand(() -> true)), new ClimbAndWinchInCommand(climberSubsystem, shiftingSubsystem), 
          new ToggleStaticHooksCommand(climberSubsystem)
        );
      }

      @Override
      public boolean isFinished() {
          return joystick.getRawButton(9);
      }
}
