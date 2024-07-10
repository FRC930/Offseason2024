// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.IOs.implementations.RollerMotorIORobot;
import frc.robot.IOs.implementations.TalonVelocityIORobot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandXboxController coDriverController = new CommandXboxController(1); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final Slot0Configs m_shooterTopS0C = new Slot0Configs()
  .withKP(0.0)
  .withKI(0.0)
  .withKD(0.0)
  .withKS(0.0)
  .withKA(0.0)
  .withKG(0.0)
  .withKV(0.0);
  private final Slot0Configs m_shooterBottomS0C = new Slot0Configs()
  .withKP(0.0)
  .withKI(0.0)
  .withKD(0.0)
  .withKS(0.0)
  .withKA(0.0)
  .withKG(0.0)
  .withKV(0.0);

  private final MotionMagicConfigs m_shooterTopMMC = new MotionMagicConfigs()
    .withMotionMagicAcceleration(0.0)
    .withMotionMagicCruiseVelocity(0.0);

  private final MotionMagicConfigs m_shooterBottomMMC = new MotionMagicConfigs()
    .withMotionMagicAcceleration(0.0)
    .withMotionMagicCruiseVelocity(0.0);

  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem(
    new TalonVelocityIORobot(0, 1.0, m_shooterTopS0C, null, m_shooterTopMMC),
    new TalonVelocityIORobot(1, 1.0, m_shooterBottomS0C, null, m_shooterBottomMMC));

  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(new RollerMotorIORobot(2, "canbus"));

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);


    coDriverController.leftBumper().onTrue(m_intakeSubsystem.newSetSpeedCommand(75)).onFalse(m_intakeSubsystem.newSetSpeedCommand(0.0));
    coDriverController.rightBumper().onTrue(m_shooterSubsystem.newSetSpeedsCommand(75, 75)).onFalse(m_shooterSubsystem.newSetSpeedsCommand(0.0, 0.0));
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
