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
        return shooter.newSetSpeedsCommand(0.5, 0.5)
        .andThen(new WaitCommand(0.5))
        .andThen(indexer.newSetSpeedCommand(1.0))
        .andThen(indexer.newWaitUntilNoNoteCommand())
        .andThen(new WaitCommand(1.0))
        .andThen(createStopShootCommand(shooter, indexer));
    }

    public static Command createFeedCommand(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return shooter.newSetSpeedsCommand(0.6, 0.6)
        .andThen(new WaitCommand(0.5))
        .andThen(indexer.newSetSpeedCommand(1.0))
        .andThen(indexer.newWaitUntilNoNoteCommand())
        .andThen(new WaitCommand(0.25))
        .andThen(shooter.newSetSpeedsCommand(0.0, 0.0))
        .andThen(indexer.newSetSpeedCommand(0.0));
    }

    public static Command createWoofShootCommand(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return shooter.newSetSpeedsCommand(0.3, 1.0)
        .andThen(new WaitCommand(0.5))
        .andThen(indexer.newSetSpeedCommand(1.0))
        .andThen(indexer.newWaitUntilNoNoteCommand())
        .andThen(new WaitCommand(0.25))
        .andThen(shooter.newSetSpeedsCommand(0.0, 0.0))
        .andThen(indexer.newSetSpeedCommand(0.0));
    }

    public static Command createStartIntakeCommand(IntakeSubsystem intake, IndexerSubsystem indexer) {
        return intake.newSetSpeedCommand(0.7)
            .andThen(indexer.newSetSpeedCommand(0.3))
            .andThen(indexer.newWaitUntilNoteCommand())
            .andThen(new WaitCommand(0.00))
            .andThen(createStopIntakeCommand(intake, indexer));
    }

    public static Command createEjectCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, IntakeSubsystem intake) {
        return shooter.newSetSpeedsCommand(0.15, 0.23)
                .andThen(indexer.newSetSpeedCommand(1.0))
                .andThen(intake.newSetSpeedCommand(-0.5));
    }

    public static Command createEjectShooterCommand(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return shooter.newSetSpeedsCommand(-0.15, -0.15)
                .andThen(indexer.newSetSpeedCommand(-0.3));
    }

    public static Command createStopIntakeCommand(IntakeSubsystem intake, IndexerSubsystem indexer) {
        return intake.newSetSpeedCommand(0.0)
            .andThen(indexer.newSetSpeedCommand(0.0));
    }

    public static Command createStopShootCommand(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return shooter.newSetSpeedsCommand(0.0, 0.0)
            .andThen(indexer.newSetSpeedCommand(0.0));
    }

    public static Command createStopEjectCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, IntakeSubsystem intake) {
        return shooter.newSetSpeedsCommand(0.0, 0.0)
            .andThen(indexer.newSetSpeedCommand(0.0))
            .andThen(intake.newSetSpeedCommand(0.0));
    }
    
}