package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utilities.CommandFactoryUtility;

public class AutoCommandManager {

    SendableChooser<Command> m_chooser = new SendableChooser<>();

    public AutoCommandManager(
        CommandSwerveDrivetrain drivetrain, 
        ShooterSubsystem shooter, 
        IndexerSubsystem indexer, 
        IntakeSubsystem intake) {
        configureNamedCommands(
            drivetrain, 
            shooter, 
            indexer,  
            intake);
            
        PathPlannerAuto midTwo = new PathPlannerAuto("MidTwo");
        PathPlannerAuto sourceTwo = new PathPlannerAuto("SourceTwo");

        m_chooser.setDefaultOption("None", new InstantCommand());

        m_chooser.addOption("MidTwo", midTwo);
        m_chooser.addOption("SourceTwo", sourceTwo);

        SmartDashboard.putData("SelectAuto", m_chooser);
    }

    public SendableChooser<Command> getChooser() {
        return m_chooser;
    }

    public Command getAutoManagerSelected(){
        return m_chooser.getSelected();
    }

    public void configureNamedCommands(
        CommandSwerveDrivetrain drivetrain,
        ShooterSubsystem shooter, 
        IndexerSubsystem indexer, 
        IntakeSubsystem intake) { 

        NamedCommands.registerCommand("shoot", CommandFactoryUtility.createShootCommand(shooter, indexer));
        NamedCommands.registerCommand("stopShoot", CommandFactoryUtility.createStopShootCommand(shooter, indexer));
        NamedCommands.registerCommand("intake", CommandFactoryUtility.createStartIntakeCommand(intake, indexer));
        NamedCommands.registerCommand("stopIntake", CommandFactoryUtility.createStopIntakeCommand(intake, indexer));

    }
}