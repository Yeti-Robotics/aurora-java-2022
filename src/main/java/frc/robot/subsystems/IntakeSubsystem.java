package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    public enum IntakeStatus {
        OUT,
        IN
    }

    private IntakeStatus intakeStatus;

    private TalonFX intakeFalcon;
    private DoubleSolenoid pistons;

    public IntakeSubsystem() {
        pistons =
                new DoubleSolenoid(
                        PneumaticsModuleType.CTREPCM,
                        IntakeConstants.INTAKE_PISTONS_SOLENOID[0],
                        IntakeConstants.INTAKE_PISTONS_SOLENOID[1]);
        intakeFalcon = new TalonFX(IntakeConstants.INTAKE_FALCON);
        intakeStatus = IntakeStatus.IN;
        pistons.set(Value.kForward);
        intakeFalcon.setInverted(true);
    }

    public void extend() {
        pistons.set(DoubleSolenoid.Value.kReverse);
        intakeStatus = IntakeStatus.OUT;
    }

    public void retract() {
        pistons.set(DoubleSolenoid.Value.kForward);
        intakeStatus = IntakeStatus.IN;
    }

    public void rollIn() {
        intakeFalcon.set(ControlMode.PercentOutput, IntakeConstants.INTAKE_SPEED);
    }

    public void rollIn(double speed) {
        intakeFalcon.set(ControlMode.PercentOutput, speed);
    }

    public void rollOut() {
        intakeFalcon.set(ControlMode.PercentOutput, -IntakeConstants.INTAKE_OUT_SPEED);
    }

    public void stopRoll() {
        intakeFalcon.set(ControlMode.PercentOutput, 0.0);
    }

    public void toggleIntake() {
        pistons.toggle();
    }

    public IntakeStatus getIntakePostion() {
        return intakeStatus;
    }

    public IntakeStatus getIntakePositionDown() {
        return IntakeStatus.OUT;
    }

    public IntakeStatus getIntakePositionUp() {
        return IntakeStatus.IN;
    }
}
