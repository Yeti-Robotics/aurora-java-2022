package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterLEDCommand extends CommandBase {
//    private final LEDSubsystem ledSubsystem;
//    private final ShooterSubsystem shooterSubsystem;
//    int rpmError = 50;
//
//    public ShooterLEDCommand(LEDSubsystem ledSubsystem, ShooterSubsystem shooterSubsystem) {
//        this.ledSubsystem = ledSubsystem;
//        this.shooterSubsystem = shooterSubsystem;
//
//        addRequirements(ledSubsystem);
//    }
//
//    @Override
//    public void initialize() {
//        shooterSubsystem.setSetPoint(6750);
//        for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
//            ledSubsystem.setRGB(i, 255, 0, 0);
//        }
//        ledSubsystem.sendData();
//    }
//
//    @Override
//    public void execute() {
//        System.out.println("rpm: " + shooterSubsystem.getFlywheelRPM());
//        System.out.println("setpoint: " + shooterSubsystem.setPoint);
//        if ((shooterSubsystem.getFlywheelRPM() > shooterSubsystem.setPoint && shooterSubsystem.getFlywheelRPM() - rpmError <= shooterSubsystem.setPoint) || shooterSubsystem.getFlywheelRPM() + rpmError >= shooterSubsystem.setPoint) {
//            // within error of setpoint
//            System.out.println("green");
//            for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
//                ledSubsystem.setRGB(i, 0, 255, 0);
//            }
//            ledSubsystem.sendData();
//        } else {
//            // not near setpoint
//            System.out.println("red");
//            for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
//                ledSubsystem.setRGB(i, 255, 0, 0);
//            }
//            ledSubsystem.sendData();
//        }
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//        System.out.println("ShooterLEDCommand.end");
//    }
//
//    @Override
//    public boolean isFinished() {
//        return false;
//    }
}
