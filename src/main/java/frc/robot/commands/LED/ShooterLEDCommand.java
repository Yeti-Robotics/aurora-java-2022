package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterLEDCommand extends CommandBase {
    private final LEDSubsystem ledSubsystem;

    public ShooterLEDCommand(LEDSubsystem ledSubsystem) {
        this.ledSubsystem = ledSubsystem;

        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
        for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
            ledSubsystem.setRGB(i, 255, 0, 0);
        }
        ledSubsystem.sendData();
    }

    @Override
    public void execute() {
        if (ShooterSubsystem.atSetPoint) {
            for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
                ledSubsystem.setRGB(i, 0, 255, 0);
            }
            ledSubsystem.sendData();
        } else {
            for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
                ledSubsystem.setRGB(i, 255, 0, 0);
            }
            ledSubsystem.sendData();
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
