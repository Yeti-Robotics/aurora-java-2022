package frc.robot.di;

import javax.inject.Named;
import javax.inject.Singleton;

import dagger.Module;
import dagger.Provides;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

@Module
public class RobotModule {
    @Singleton
    @Provides
    public RobotContainer provideRobotContainer() {
        return new RobotContainer();
    }

    @Provides
    public PowerDistribution provideRevPDH() {
        return new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
    }

    @Provides
    public Joystick provideJoystick() {
        return new Joystick(Constants.OIConstants.DRIVER_STATION_JOY);
    }

}
