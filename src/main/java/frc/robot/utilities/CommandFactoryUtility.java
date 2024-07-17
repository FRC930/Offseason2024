package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * CommandFactoryUtility
 */
public class CommandFactoryUtility {

    public static Command createShootCommand(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return shooter.newSetSpeedsCommand(50.0, 50.0)
        .andThen(shooter.newWaitUntilSetpointCommand(2.5))
        .andThen(indexer.newSetSpeedCommand(0.5))
        .andThen(indexer.newWaitUntilNoNoteCommand())
        .andThen(new WaitCommand(1.0))
        .andThen(shooter.newSetSpeedsCommand(0.0, 0.0));
    }

    public static Command createFeedCommand(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return shooter.newSetSpeedsCommand(60.0, 75.0)
        .andThen(shooter.newWaitUntilSetpointCommand(2.5))
        .andThen(indexer.newSetSpeedCommand(0.5))
        .andThen(indexer.newWaitUntilNoNoteCommand())
        .andThen(new WaitCommand(1.0))
        .andThen(shooter.newSetSpeedsCommand(0.0, 0.0));
    }

    public static Command createStartIntakeCommand(IntakeSubsystem intake, IndexerSubsystem indexer) {
        return intake.newSetSpeedCommand(0.5)
            .andThen(indexer.newSetSpeedCommand(0.5))
            .andThen(indexer.newWaitUntilNoteCommand())
            .andThen(createStopIntakeCommand(intake, indexer));
    }

    public static Command createStopIntakeCommand(IntakeSubsystem intake, IndexerSubsystem indexer) {
        return intake.newSetSpeedCommand(0.0)
            .andThen(indexer.newSetSpeedCommand(0.0));
    }
    
}