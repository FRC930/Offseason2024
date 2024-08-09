package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.utilities.RobotOdometryUtility;

public class SwerveAutoRotateCommand extends Command{
    private final AprilTagFieldLayout m_AprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private final double m_MAX_ROTATION_SPEED = 0.9;
    private PIDController pid = new PIDController(0.02, 0.0, 0.0);
    private FieldCentric m_driveTrain;

    private Alliance alliance;

    private Pose2d m_targestPose;

    private DoubleSupplier m_xSupplier;
    private DoubleSupplier m_ySupplier;

    private double tx;
    private double ty;
    private double rx;
    private double ry;

    private boolean m_scoringInSpeaker;

    public SwerveAutoRotateCommand(FieldCentric drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, boolean scoringInSpeaker) {
        m_driveTrain = drivetrain;
        m_xSupplier = xSupplier;
        m_ySupplier = ySupplier;
        m_scoringInSpeaker = scoringInSpeaker;
    }

    @Override
    public void initialize() {
        Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
        if (optionalAlliance.isPresent()){
            alliance = optionalAlliance.get();
        } else {
            alliance = Alliance.Blue;
        }

        m_targestPose = 
            alliance == Alliance.Red ? 
            m_AprilTagFieldLayout.getTagPose(4).get().toPose2d() : 
            m_AprilTagFieldLayout.getTagPose(7).get().toPose2d();
    }

    @Override
    public void execute() {
        Translation2d linearVelocity = RobotContainer.getLinearVelocity(m_xSupplier.getAsDouble(), m_ySupplier.getAsDouble());
        
        m_driveTrain.withVelocityX(RobotContainer.scaleLinearVelocity(linearVelocity.getX()))
                .withVelocityY(RobotContainer.scaleLinearVelocity(linearVelocity.getY()))
                .withRotationalRate(
                        MathUtil.clamp(
                                pid.calculate(-getTurningValue(RobotOdometryUtility.getInstance().getRobotOdometry()), 0.0),
                                -m_MAX_ROTATION_SPEED, m_MAX_ROTATION_SPEED) * RobotContainer.MaxAngularRate);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private double getTurningValue(Pose2d pose) {
        tx = m_targestPose.getX();
        ty = m_targestPose.getY();
        rx = pose.getX();
        ry = pose.getY();

        if (m_scoringInSpeaker) {
            return Math.IEEEremainder(
                Math.toDegrees(Math.atan2(ty - ry, tx - rx)) - pose.getRotation().getDegrees() + 180, 
                360);
        } else {
            double rotation =  alliance == Alliance.Red ? 40.0 : 140.0;
            return Math.IEEEremainder(rotation - pose.getRotation().getDegrees() + 180, 360);
        }
    }
}
