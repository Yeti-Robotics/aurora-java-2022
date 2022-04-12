package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class ShiftingSubsystem extends SubsystemBase {
    private DoubleSolenoid shifter;

    public enum ShiftStatus {
        HIGH, LOW
    }

    public static ShiftStatus shiftStatus;

    private ShiftingSubsystem() {
        shifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, DriveConstants.SOLENOID_SHIFTER[0], DriveConstants.SOLENOID_SHIFTER[1]);
        shiftUp();
    }

    private static final ShiftingSubsystem instance = new ShiftingSubsystem();
    public static ShiftingSubsystem getInstance() {
        return instance;
    }

    public void shiftUp() {
        shifter.set(DoubleSolenoid.Value.kReverse);
        shiftStatus = ShiftStatus.HIGH;
    }

    public void shiftDown() {
        shifter.set(DoubleSolenoid.Value.kForward);
        shiftStatus = ShiftStatus.LOW;
    }

    public static ShiftStatus getShifterPosition() {
        return shiftStatus;
    }
}
