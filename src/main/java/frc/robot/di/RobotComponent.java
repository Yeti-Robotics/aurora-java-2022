package frc.robot.di;

import javax.inject.Singleton;

import dagger.Component;
import frc.robot.Robot;

@Singleton
@Component(
    modules = {
        SubsystemsModule.class,
        CommandsModule.class,
        RobotModule.class
    }
)
public
interface RobotComponent {
    // Get rid of this, use DaggerRobotComponent.create();
    @Component.Builder
    interface Builder {
        public RobotComponent build();
    }

    void inject(Robot robot);
}
