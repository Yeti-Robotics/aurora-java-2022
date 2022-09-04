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

import static frc.robot.Constants.IntakeConstants.INTAKE_SPEED;
import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

public class IntakeSubsystemTest {
    IntakeSubsystem intake;
    DoubleSolenoid pistons;
    WPI_TalonFX intakeFalcon = mock(WPI_TalonFX.class);


    @Before
    public void setup() {
        assert HAL.initialize(500, 0);
        pistons = new DoubleSolenoid(
                PneumaticsModuleType.CTREPCM,
                Constants.IntakeConstants.INTAKE_PISTONS_SOLENOID[0],
                Constants.IntakeConstants.INTAKE_PISTONS_SOLENOID[1]);
        pistons.set(DoubleSolenoid.Value.kForward);
        intakeFalcon.setInverted(true);
        intake = new IntakeSubsystem(pistons, intakeFalcon);
        intakeFalcon.configFactoryDefault();
    }

    @After
    public void shutdown() throws Exception {
        intake.close();
    }

    @Test
    public void testExtend() {
        intake.extend();

        assertEquals(IntakeSubsystem.IntakeStatus.OUT, intake.getIntakePostion());
        assertEquals(DoubleSolenoid.Value.kReverse, pistons.get());
    }

    @Test
    public void testRetract() {
        intake.retract();

        assertEquals(IntakeSubsystem.IntakeStatus.IN, intake.getIntakePostion());
        assertEquals(DoubleSolenoid.Value.kForward, pistons.get());
    }

    @Test
    public void testRollIn() {
        intake.rollIn();

        verify(intakeFalcon).set(ControlMode.PercentOutput, INTAKE_SPEED);
    }

    @Test
    public void testRollInSpeed() {
        intake.rollIn(0.94);

        verify(intakeFalcon).set(ControlMode.PercentOutput, 0.94);
    }

    @Test
    public void testStopRoll() {
        intake.stopRoll();

        verify(intakeFalcon).set(ControlMode.PercentOutput, 0.0);
    }

    @Test
    public void testRollOut() {
        intake.rollOut();

        verify(intakeFalcon).set(ControlMode.PercentOutput, -INTAKE_SPEED);
    }

    @Test
    public void testToggleIntake() {
        assertEquals(IntakeSubsystem.IntakeStatus.IN, intake.getIntakePostion());
        assertEquals(DoubleSolenoid.Value.kForward, pistons.get());

        intake.toggleIntake();

        assertEquals(IntakeSubsystem.IntakeStatus.OUT, intake.getIntakePostion());
        assertEquals(DoubleSolenoid.Value.kReverse, pistons.get());

        intake.toggleIntake();

        assertEquals(IntakeSubsystem.IntakeStatus.IN, intake.getIntakePostion());
        assertEquals(DoubleSolenoid.Value.kForward, pistons.get());
    }
}
