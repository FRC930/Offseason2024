package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * CommandFactoryUtility
 */
public class CommandFactoryUtility {


    //TODO implement indexer
    public static Command createShootCommand(ShooterSubsystem shooter) {
        return shooter.newSetSpeedsCommand(50, 50)
        //.andThen(WaitCommand.newWaitUntilSetpointCommand()); indexer wait IMPLEMENT
        .andThen(new WaitCommand(1))
        .andThen(shooter.newSetSpeedsCommand(0, 0));
    }

    public static Command createStartIntakeCommand(IntakeSubsystem intake) {
        return intake.newSetSpeedCommand(0.9);
    }

    public static Command createStopIntakeCommand(IntakeSubsystem intake) {
        return intake.newSetSpeedCommand(0.0);
    }
    
}