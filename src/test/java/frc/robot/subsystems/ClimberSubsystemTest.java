package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static frc.robot.Constants.ClimberConstants.CLIMB_SPEED;
import static org.mockito.Mockito.*;

public class ClimberSubsystemTest {
    ClimberSubsystem climber;
    WPI_TalonFX climberFalcon1 = mock(WPI_TalonFX.class);
    WPI_TalonFX climberFalcon2 = mock(WPI_TalonFX.class);
    DoubleSolenoid climberBrake;

    @Before
    public void setup() {
        assert HAL.initialize(500, 0);
        climberBrake = new DoubleSolenoid(
                PneumaticsModuleType.CTREPCM,
                Constants.ClimberConstants.CLIMBER_BRAKE[0],
                Constants.ClimberConstants.CLIMBER_BRAKE[1]);
        climber = new ClimberSubsystem(climberFalcon1, climberFalcon2, climberBrake);
    }

    @After
    public void shutdown() throws Exception {
        climber.close();
    }

    @Test
    public void testClimbUpHappyCase() {
        when(climberFalcon1.getSelectedSensorPosition()).thenReturn(100.0);
        when(climberFalcon2.getSelectedSensorPosition()).thenReturn(100.0);

        climber.climbUp();

        verify(climberFalcon1).set(ControlMode.PercentOutput, CLIMB_SPEED);
        verify(climberFalcon2, never()).set(any(ControlMode.class), anyDouble());
    }

    @Test
    public void testClimbUpBrakeSet() {
        climberBrake.set(DoubleSolenoid.Value.kReverse);
        when(climberFalcon1.getSelectedSensorPosition()).thenReturn(100.0);
        when(climberFalcon2.getSelectedSensorPosition()).thenReturn(100.0);

        climber.climbUp();

        verify(climberFalcon1).set(ControlMode.PercentOutput, 0.0);
        verify(climberFalcon2, never()).set(any(ControlMode.class), anyDouble());
    }

    @Test
    public void testClimbUp() {
        climberBrake.set(DoubleSolenoid.Value.kForward);
        when(climberFalcon1.getSelectedSensorPosition()).thenReturn(98900.0);
        when(climberFalcon2.getSelectedSensorPosition()).thenReturn(98900.0);

        climber.climbUp();

        verify(climberFalcon1).set(ControlMode.PercentOutput, 0.0);
        verify(climberFalcon2, never()).set(any(ControlMode.class), anyDouble());
    }

    @Test
    public void testClimbDownHappyCase() {
        when(climberFalcon1.getSelectedSensorPosition()).thenReturn(20.0);
        when(climberFalcon2.getSelectedSensorPosition()).thenReturn(20.0);

        climber.climbDown();

        verify(climberFalcon1).set(ControlMode.PercentOutput, -CLIMB_SPEED);
        verify(climberFalcon2, never()).set(any(ControlMode.class), anyDouble());
    }

    @Test
    public void testClimbDownBrakeSet() {
        climberBrake.set(DoubleSolenoid.Value.kReverse);
        when(climberFalcon1.getSelectedSensorPosition()).thenReturn(20.0);
        when(climberFalcon2.getSelectedSensorPosition()).thenReturn(20.0);

        climber.climbDown();

        verify(climberFalcon1).set(ControlMode.PercentOutput, 0.0);
        verify(climberFalcon2, never()).set(any(ControlMode.class), anyDouble());
    }

    @Test
    public void testClimbDown() {
        climberBrake.set(DoubleSolenoid.Value.kForward);
        when(climberFalcon1.getSelectedSensorPosition()).thenReturn(10.0);
        when(climberFalcon2.getSelectedSensorPosition()).thenReturn(10.0);

        climber.climbDown();

        verify(climberFalcon1).set(ControlMode.PercentOutput, 0.0);
        verify(climberFalcon2, never()).set(any(ControlMode.class), anyDouble());
    }
}
