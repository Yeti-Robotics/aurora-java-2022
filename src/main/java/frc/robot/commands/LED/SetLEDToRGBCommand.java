package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.LEDSubsystem;

public class SetLEDToRGBCommand extends CommandBase {
    private final LEDSubsystem lEDSubsystem;
    private final int r, g, b;

    public SetLEDToRGBCommand(LEDSubsystem lEDSubsystem, int r, int g, int b) {
        this.lEDSubsystem = lEDSubsystem;
        this.r = r;
        this.g = g;
        this.b = b;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.lEDSubsystem);
    }

    @Override
    public void initialize() {
        for (int i = 0; i < lEDSubsystem.getBufferLength(); i++) {
            lEDSubsystem.setRGB(i, r, g, b);
        }
        lEDSubsystem.sendData();
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {}
}
