package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.utilities.AprilTagUtility;

public class LimeLightSubsystem extends SubsystemBase {
  private AprilTagUtility m_AprilTagUtility = new AprilTagUtility();

  /**
   * Determines whether this subsystem should do anything at all
   */
  public static final boolean updatesOdometryEver = true;
  public static final boolean updatesOdometryInAuto = false;


  public LimeLightSubsystem() {
    m_AprilTagUtility.m_visionUpdatesOdometry = false;
  }

  @Override
  public void periodic() {
    doOdometryUpdate();
  }

  public void doOdometryUpdate() {
    //If we aren't updating odometry with cameras, return
    if(!updatesOdometryEver) {
        return;
    }
    //Don't update odometry in auto if we don't want to
    if(!updatesOdometryInAuto && DriverStation.isAutonomous()) {
        m_AprilTagUtility.m_visionUpdatesOdometry = false;
        return;
    }

    //If all guard clauses pass, update odometry
    m_AprilTagUtility.updatePoseWithMegaTag2("limelight-front", true);
  }

}
