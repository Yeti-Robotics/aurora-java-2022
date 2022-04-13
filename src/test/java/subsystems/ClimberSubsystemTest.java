package subsystems;

import static org.junit.Assert.*;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants.ClimberConstants;
import org.junit.*;
import frc.robot.subsystems.ClimberSubsystem;

// all works great except it seems updating encoders doesn't work at all
public class ClimberSubsystemTest {
    public static final double DELTA = 1e-2;
    ClimberSubsystem climberSubsystem;
    WPI_TalonFX climberFalcon1, climberFalcon2;
    DoubleSolenoid climberBrake;

    @Before
    public void before() {
        System.out.println("Preparing test...");
        assert HAL.initialize(500, 0);

        climberFalcon1 = new WPI_TalonFX(ClimberConstants.CLIMBER_1);
        climberFalcon2 = new WPI_TalonFX(ClimberConstants.CLIMBER_2);

        climberBrake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.CLIMBER_BRAKE[0], ClimberConstants.CLIMBER_BRAKE[1]);

        climberSubsystem = new ClimberSubsystem(climberFalcon1, climberFalcon2, climberBrake);
    }

    @After
    public void after() {
        System.out.println("Cleaning up test...");
        climberSubsystem.close();
    }

    @Test
    public void initialization() {
        // all objects are initialized
        assertNotNull(climberFalcon1);
        assertNotNull(climberFalcon2);
        assertNotNull(climberBrake);

        // falcons are inverted in correct direction
        assertTrue(climberFalcon1.getInverted());
        assertFalse(climberFalcon2.getInverted());

        // brake is not locked by default
        assertEquals(Value.kForward, climberBrake.get());
    }

    // test won't wor because can't update encoders to allow for motor movement
//    @Test
//    public void climbDownWhenWithinLimits() {
//        assertSame(Value.kForward, climberBrake.get());
//
//        // set within climb down range
//        // encoder updating doesn't seem to work
//        climberFalcon1.setSelectedSensorPosition(15.0);
//        climberFalcon2.setSelectedSensorPosition(15.0);
//
//        assertTrue((climberSubsystem.getAverageEncoder() + 15.0) >= ClimberConstants.CLIMBER_TOLERANCE);
//        climberSubsystem.climbDown();
//        assertEquals(-ClimberConstants.CLIMB_SPEED, climberFalcon1.get(), DELTA);
//    }

    @Test
    public void dontClimbDownWhenBrake() {
        climberBrake.set(Value.kReverse);
        climberSubsystem.climbDown();
        assertEquals(0.0, climberFalcon1.get(), DELTA);
        assertEquals(0.0, climberFalcon2.get(), DELTA);
    }

    @Test
    public void climbUpWhenWithinLimits() {
        assertSame(Value.kForward, climberBrake.get());
        assertTrue(climberSubsystem.getAverageEncoder() <= ClimberConstants.CLIMBER_TOLERANCE);
        climberSubsystem.climbUp();
        assertEquals(ClimberConstants.CLIMB_SPEED, climberFalcon1.get(), DELTA);
    }

    @Test
    public void dontClimbUpWhenBrake() {
        climberBrake.set(Value.kReverse);
        climberSubsystem.climbUp();
        assertEquals(0.0, climberFalcon1.get(), DELTA);
    }

    @Test
    public void stopClimbWorks() {
        climberFalcon1.set(0.1);
        climberSubsystem.stopClimb();
        assertEquals(0.0, climberFalcon1.get(), DELTA);
    }

//    @Test
//    public void getLeftEncoderWorks() {
//        double testVal = 2.0;
//        // encoder updating doesn't seem to work
//        climberFalcon2.setSelectedSensorPosition(testVal);
//        assertEquals(testVal, climberSubsystem.getLeftEncoder(), DELTA);
//    }
//
//    @Test
//    public void getRightEncoderWorks() {
//        double testVal = 2.0;
//        // encoder updating doesn't seem to work
//        climberFalcon1.setSelectedSensorPosition(testVal);
//        assertEquals(testVal, climberSubsystem.getRightEncoder(), DELTA);
//    }
//
//    @Test
//    public void getAvgEncoderWorks() {
//        double testVal1 = 2.0;
//        double testVal2 = 4.0;
//        // encoder updating doesn't seem to work
//        climberFalcon1.setSelectedSensorPosition(testVal1);
//        climberFalcon2.setSelectedSensorPosition(testVal2);
//        assertEquals((testVal1 + testVal2) / 2, climberSubsystem.getAverageEncoder(), DELTA);
//    }

    @Test
    public void resetEncodersWorks() {
        double testVal1 = 2.0;
        double testVal2 = 4.0;
        // encoder updating doesn't seem to work
        climberFalcon1.setSelectedSensorPosition(testVal1);
        climberFalcon2.setSelectedSensorPosition(testVal2);
        climberSubsystem.resetEncoders();

        // expect 0.0 instead of 3.0
        assertEquals(0.0, climberSubsystem.getAverageEncoder(), DELTA);
    }

    @Test
    public void toggleBrakeWorks() {
        assertSame(Value.kForward, climberBrake.get());
        climberSubsystem.toggleClimberBrake();
        assertSame(Value.kReverse, climberBrake.get());
    }
}
