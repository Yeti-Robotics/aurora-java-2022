package frc.robot.commands.shifting;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ShiftingSubsystem;

public class ToggleShiftCommand extends CommandBase {
    private final ShiftingSubsystem shiftingSubsystem;

    public ToggleShiftCommand(ShiftingSubsystem shiftingSubsystem) {
        this.shiftingSubsystem = shiftingSubsystem;
        addRequirements(shiftingSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (ShiftingSubsystem.getShifterPosition() == ShiftingSubsystem.ShiftStatus.HIGH) {
            shiftingSubsystem.shiftDown();
        } else {
            shiftingSubsystem.shiftUp();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {}
}
