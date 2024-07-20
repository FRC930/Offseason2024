// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.IOs.implementations.RollerMotorIORobot;
import frc.robot.IOs.implementations.RollerMotorIOSim;
import frc.robot.IOs.implementations.TalonVelocityIORobot;
import frc.robot.IOs.implementations.TalonVelocityIOSim;
import frc.robot.IOs.implementations.TimeOfFlightIORobot;
import frc.robot.IOs.implementations.TimeOfFlightIOSim;
import frc.robot.commands.Orchestra.OrchestraCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utilities.CommandFactoryUtility;


public class RobotContainer {
  private static final double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private static final double JOYSTICK_DEADBAND = 0.1;
  private static final double JOYSTICK_ROTATIONAL_DEADBAND = 0.1; 
  private static final double PERCENT_SPEED = 1.0;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController m_driverController = new CommandXboxController(0); // My joystick
  private final CommandXboxController m_coDriverController = new CommandXboxController(1); // My joystick
  private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final Slot0Configs m_shooterTopS0C = new Slot0Configs()
  .withKP(1.0)
  .withKI(0.0)
  .withKD(0.0)
  .withKS(0.0)
  .withKA(0.0)
  .withKG(0.0)
  .withKV(0.0);
  private final Slot0Configs m_shooterBottomS0C = new Slot0Configs()
  .withKP(1.0)
  .withKI(0.0)
  .withKD(0.0)
  .withKS(0.0)
  .withKA(0.0)
  .withKG(0.0)
  .withKV(0.0);

  private final MotionMagicConfigs m_shooterTopMMC = new MotionMagicConfigs()
    .withMotionMagicAcceleration(1.0)
    .withMotionMagicCruiseVelocity(1.0);

  private final MotionMagicConfigs m_shooterBottomMMC = new MotionMagicConfigs()
    .withMotionMagicAcceleration(1.0)
    .withMotionMagicCruiseVelocity(1.0);

  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem(
      // (Robot.isReal() ? new TalonVelocityIORobot(5, 1.0, m_shooterTopS0C, null, m_shooterTopMMC) : new TalonVelocityIOSim(5, 1.0, m_shooterTopS0C, null, m_shooterTopMMC)),
      // (Robot.isReal() ? new TalonVelocityIORobot(6, 1.0, m_shooterBottomS0C, null, m_shooterBottomMMC) : new TalonVelocityIOSim(6, 1.0, m_shooterBottomS0C, null, m_shooterBottomMMC))
      (Robot.isReal() ? new RollerMotorIORobot(5, "rio") : new RollerMotorIOSim(5, "rio")),
      (Robot.isReal() ? new RollerMotorIORobot(6, "rio") : new RollerMotorIOSim(6, "rio"))
  );

  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(
      (Robot.isReal() ? new RollerMotorIORobot(7, "rio") : new RollerMotorIOSim(7, "rio"))
  );

  private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem(
      (Robot.isReal() ? new RollerMotorIORobot(3, "rio") : new RollerMotorIOSim(3, "rio")), 
      (Robot.isReal() ? new TimeOfFlightIORobot(1, 250.0) : new TimeOfFlightIOSim(1))
  );

  private AutoCommandManager m_autoManager = new AutoCommandManager(
      m_drivetrain, 
      m_shooterSubsystem, 
      m_indexerSubsystem,
      m_intakeSubsystem);

  private void 
  configureBindings() {
    m_drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
    // Code originally from team number 1091 to help deal with deadband on joystick for swerve drive (ty)
    m_drivetrain.applyRequest(
      joystickDriveWithDeadband(
        m_driverController::getLeftY,
        m_driverController::getLeftX,
        m_driverController::getRightX)
    ));

    if (Utils.isSimulation()) {
      m_drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90.0)));
    }
    m_drivetrain.registerTelemetry(logger::telemeterize);

    m_driverController.start().toggleOnTrue((new OrchestraCommand("Yarhar", "HesAPirate.chrp", m_shooterSubsystem)));
    // m_driverController.start().toggleOnTrue((new OrchestraCommand("Cantina", "Cantina.chrp", m_shooterSubsystem)));
    //m_driverController.start().toggleOnTrue((new OrchestraCommand("SelfDestruct", "SELF_DESTRUCT.chrp", m_shooterSubsystem)));


    m_coDriverController.leftBumper().onTrue(m_intakeSubsystem.newSetSpeedCommand(75.0)).onFalse(m_intakeSubsystem.newSetSpeedCommand(0.0));
    m_coDriverController.rightBumper().onTrue(m_shooterSubsystem.newSetSpeedsCommand(75.0, 75.0)).onFalse(m_shooterSubsystem.newSetSpeedsCommand(0.0, 0.0));
    
    m_driverController.leftBumper().onTrue(CommandFactoryUtility.createStartIntakeCommand(m_intakeSubsystem, m_indexerSubsystem))
        .onFalse(CommandFactoryUtility.createStopIntakeCommand(m_intakeSubsystem, m_indexerSubsystem));

    m_driverController.rightBumper().onTrue(CommandFactoryUtility.createShootCommand(m_shooterSubsystem, m_indexerSubsystem))
        .onFalse(CommandFactoryUtility.createStopShootCommand(m_shooterSubsystem, m_indexerSubsystem));

    m_driverController.leftTrigger().onTrue(CommandFactoryUtility.createEjectCommand(m_shooterSubsystem, m_indexerSubsystem, m_intakeSubsystem))
        .onFalse(CommandFactoryUtility.createStopAllCommand(m_shooterSubsystem, m_indexerSubsystem, m_intakeSubsystem));

    m_driverController.rightTrigger().onTrue(CommandFactoryUtility.createWoofShootCommand(m_shooterSubsystem, m_indexerSubsystem))
        .onFalse(CommandFactoryUtility.createStopShootCommand(m_shooterSubsystem, m_indexerSubsystem));

    m_driverController.y().onTrue(CommandFactoryUtility.createFeedCommand(m_shooterSubsystem, m_indexerSubsystem))
        .onFalse(CommandFactoryUtility.createStopShootCommand(m_shooterSubsystem, m_indexerSubsystem));

    m_driverController.x().whileTrue(m_drivetrain.applyRequest(() -> brake));

    // reset the field-centric heading on left bumper press
    m_driverController.leftStick().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative()));

    m_driverController.a().onTrue(CommandFactoryUtility.createEjectShooterCommand(m_shooterSubsystem, m_indexerSubsystem, m_intakeSubsystem))
        .onFalse(CommandFactoryUtility.createStopAllCommand(m_shooterSubsystem, m_indexerSubsystem, m_intakeSubsystem));
  }

  public static Translation2d getLinearVelocity(double xValue, double yValue) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(
                    Math.hypot(xValue, yValue), JOYSTICK_DEADBAND);
    Rotation2d linearDirection =
            new Rotation2d(-xValue, -yValue);

    // Square values
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Calculate new linear velocity
    Translation2d linearVelocity =
            new Pose2d(new Translation2d(), linearDirection)
                    .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                    .getTranslation();
    return linearVelocity;
  }

  public static double scaleLinearVelocity(double value) {
    return value*MaxSpeed*PERCENT_SPEED;
  }

  private Supplier<SwerveRequest> joystickDriveWithDeadband(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    return () -> {  
        double xValue = xSupplier.getAsDouble();   
        double yValue = ySupplier.getAsDouble();
        double omegaValue = omegaSupplier.getAsDouble();       
           
        Translation2d linearVelocity = getLinearVelocity(xValue, yValue);
                        
        // Squaring the omega value and applying a deadband 
        double omega = MathUtil.applyDeadband(-omegaValue, JOYSTICK_ROTATIONAL_DEADBAND);
        omega = Math.copySign(omega * omega, omega);


        return drive.withVelocityX(scaleLinearVelocity(linearVelocity.getX()))
          .withVelocityY(scaleLinearVelocity(linearVelocity.getY()))
          .withRotationalRate(omega * MaxAngularRate); // Drive counterclockwise with negative X (left)
      };
  }

  public RobotContainer() {
    configureBindings();
  }

  public void robotPeriodic() {
    SmartDashboard.putNumber("SensorRange", m_indexerSubsystem.getRange());
  }

  public Command getAutonomousCommand() {
    Command autoCommand = m_autoManager.getAutoManagerSelected();
    return autoCommand;
  }

}
