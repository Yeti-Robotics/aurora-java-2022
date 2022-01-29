package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NeckConstants;

public class NeckSubsystem extends SubsystemBase {
    private TalonSRX neckBackRollerTalonSRX;
    private TalonSRX neckFrontRollerTalonSRX;
    private DigitalInput neckBeamBreak;

    public NeckSubsystem() {
        neckBackRollerTalonSRX = new TalonSRX(NeckConstants.NECK_MOTOR_BACK);
        neckFrontRollerTalonSRX = new TalonSRX(NeckConstants.NECK_MOTOR_FRONT);
        neckBeamBreak = new DigitalInput(NeckConstants.NECK_BEAM_BREAK);

    }
    public void moveUp(){
        neckBackRollerTalonSRX.set(ControlMode.PercentOutput, NeckConstants.NECK_UP_SPEED);
        neckFrontRollerTalonSRX.set(ControlMode.PercentOutput, NeckConstants.NECK_UP_SPEED);
    }
    public void moveDown(){
        neckBackRollerTalonSRX.set(ControlMode.PercentOutput, NeckConstants.NECK_DOWN_SPEED);
        neckFrontRollerTalonSRX.set(ControlMode.PercentOutput, NeckConstants.NECK_DOWN_SPEED);
    }
    public void stopNeck(){
        neckBackRollerTalonSRX.set(ControlMode.PercentOutput, 0);
        neckFrontRollerTalonSRX.set(ControlMode.PercentOutput, 0);
    }
    public boolean getbeambreak(){
        return neckBeamBreak.get();
    }
    
}
