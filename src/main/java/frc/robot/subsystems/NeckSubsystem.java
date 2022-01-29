package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NeckConstants;

public class NeckSubsystem extends SubsystemBase {
    private TalonSRX neckBelTalonSRX;
    private TalonSRX neckRollerTalonSRX;
    private DigitalInput neckBeamBreak;

    public NeckSubsystem() {
        neckBelTalonSRX = new TalonSRX(NeckConstants.NECK_BELT_TALON);
        neckRollerTalonSRX = new TalonSRX(NeckConstants.NECK_ROLLER_TALON);
        neckBeamBreak = new DigitalInput(NeckConstants.NECK_BEAM_BREAK);

    }
    
}
