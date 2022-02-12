package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    
    public enum IntakeStatus {
        DOWN, UP
    }

    private IntakeStatus intakeStatus;

    private TalonFX intakeFalcon;
    private DoubleSolenoid pistons;

    public IntakeSubsystem() {
        pistons = new DoubleSolenoid(PneumaticsModuleType.REVPH, IntakeConstants.INTAKE_PISTONS_SOLENOID[0], IntakeConstants.INTAKE_PISTONS_SOLENOID[1]);
        intakeFalcon = new TalonFX(IntakeConstants.INTAKE_FALCON);
        intakeStatus = IntakeStatus.DOWN;
    }

    public void extend(){
        pistons.set(DoubleSolenoid.Value.kForward);
        intakeStatus = IntakeStatus.DOWN;
    }

    public void retract(){
        pistons.set(DoubleSolenoid.Value.kReverse);
        intakeStatus = IntakeStatus.UP;
    }

    public void rollIn(){
        intakeFalcon.set(ControlMode.PercentOutput, IntakeConstants.INTAKE_SPEED);
    }

    public void rollOut(){
        intakeFalcon.set(ControlMode.PercentOutput, IntakeConstants.INTAKE_SPEED);
    }

    public void stopRoll(){
        intakeFalcon.set(ControlMode.PercentOutput, 0.0);
    }

    public IntakeStatus getIntakePostion(){
        return intakeStatus;
    }

    public IntakeStatus getIntakePositionDown(){
        return IntakeStatus.DOWN;
    }

    public IntakeStatus getIntakePositionUp(){
        return IntakeStatus.UP;

    }
}
