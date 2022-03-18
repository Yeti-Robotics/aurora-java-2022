package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climber.ClimbUpCommand;
import frc.robot.commands.climber.WinchOutCommand;
import frc.robot.subsystems.ClimberSubsystem;

public class AutoHighClimbCommandGroup extends SequentialCommandGroup {
    private Joystick joystick;
    private ClimberSubsystem climberSubsystem;

    public AutoHighClimbCommandGroup(
        ClimberSubsystem climberSubsystem,
        Joystick driverStationJoystick
    ) {
        this.climberSubsystem = climberSubsystem;
        this.joystick = driverStationJoystick;
        addCommands(
          new ClimbUpCommand(climberSubsystem).withTimeout(0.25), 
          new WinchOutCommand(climberSubsystem).withTimeout(1.15), 
          new ClimbUpCommand(climberSubsystem).withInterrupt(() -> climberSubsystem.atEncoderLimit())
        );
      }

      @Override
      public boolean isFinished() {
          return super.isFinished() || joystick.getRawButton(10);
      }

      @Override
      public void end(boolean interrupted) {
          super.end(interrupted);
          climberSubsystem.stopClimb();
          climberSubsystem.stopWinch();
      }
}
