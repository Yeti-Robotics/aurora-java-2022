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
    public IntakeStatus intakeStatus;

    private TalonFX intakeFalcon;
    private DoubleSolenoid pistons;

    public IntakeSubsystem() {
        pistons = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, IntakeConstants.INTAKE_PISTONS_SOLENOID[0], IntakeConstants.INTAKE_PISTONS_SOLENOID[1]);
        intakeFalcon = new TalonFX(IntakeConstants.INTAKE_FALCON);
        intakeFalcon.setInverted(true);
    }
}
