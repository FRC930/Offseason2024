package frc.robot.subsystems;

import java.util.logging.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.RobotOdometryUtility;
import frc.robot.utilities.StartInTeleopUtility;

public class LimeLightSubsystem extends SubsystemBase {
  private CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  private StartInTeleopUtility m_StartInTeleopUtility = new StartInTeleopUtility(drivetrain::seedFieldRelative);
  private int visioncounter = 0;

  /**
   * Determines whether this subsystem should do anything at all
   */
  public static final boolean updatesOdometryEver = true;
  public static final boolean updatesOdometryInAuto = false;


  public LimeLightSubsystem() {

  }

  // @Override
  // public void periodic() {
  //   doOdometryUpdate();
  // }

  private void doOdometryUpdate() {
    //If we aren't updating odometry with cameras, return
    if(!updatesOdometryEver) {
        return;
    }
    //Don't update odometry in auto if we don't want to
    if(!updatesOdometryInAuto && DriverStation.isAutonomous()) {
        return;
    }

    //If all guard clauses pass, update odometry
    updatePoseWithMegaTag2("limelight-front", true);
  }
  
  private void updatePoseWithMegaTag2(String limeLightName, boolean usePose) {
        boolean doRejectUpdate = false;

        LimelightHelpers.SetRobotOrientation(limeLightName,
                RobotOdometryUtility.getInstance().getRobotOdometry().getRotation().getDegrees(), 0,
                0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limeLightName);
        double fpgaTimestamp = Timer.getFPGATimestamp();//mt2.timestampSeconds;

        double rotationalRate = TunerConstants.DriveTrain.getPigeon2().getRate();
        //Logger.recordOutput("LimeLightOdometry/rotationalRate", rotationalRate);
        if (Math.abs(rotationalRate) >= 200)
        {
            doRejectUpdate = true;
        }

        // distance from current pose to vision estimated pose
        // Translation2d translation =
        // TunerConstants.DriveTrain.getState().Pose.getTranslation();
        // double poseDifference = translation.getDistance(mt2.pose.getTranslation());

        double xyStds;
        // if (mt2.tagCount >= 2) {
        xyStds = 0.2;
        // }

        // 1 target with large area and close to estimated pose
        // else if (mt2.tagCount == 1 && mt2.avgTagArea > 0.8 && poseDifference < 0.5) {
        // xyStds = 1.0;
        // degStds = 12;
        // }
        // conditions don't match to add a vision measurement
        // else {
        // SmartDashboard.putBoolean(limeLightName + "/Updated", false);
        // return;
        // }
        
        if (usePose && !doRejectUpdate && mt2.tagCount > 0) {
            m_StartInTeleopUtility.updateTags();

            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, 99999.0));
            drivetrain.addVisionMeasurement(mt2.pose, fpgaTimestamp - (mt2.timestampSeconds / 1000.0));
            
            SmartDashboard.putNumber(limeLightName + "/VisionCounter", visioncounter);
            SmartDashboard.putBoolean(limeLightName + "/Updated", true);
            SmartDashboard.putNumber(limeLightName + "/DelayMs", Timer.getFPGATimestamp() - fpgaTimestamp);
            SmartDashboard.putNumber(limeLightName + "/Test", fpgaTimestamp);
            //Logger.recordOutput("LimeLightOdometry/" + limeLightName + "/UpdatCounts", this.visioncounter);
            this.visioncounter++;
        }

        //Logger.recordOutput("LimeLightOdometry/" + limeLightName + "/Pose", mt2.pose);
    }
}
