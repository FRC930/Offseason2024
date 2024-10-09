package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IOs.TalonPosIO;
import frc.robot.IOs.TalonRollerIO;

public class ClimberSubsystem extends SubsystemBase {
    
    private TalonRollerIO m_climberLeft;
    private TalonRollerIO m_climberRight;

    public ClimberSubsystem( TalonRollerIO climberLeft, TalonRollerIO climberRight) {

        m_climberLeft = climberLeft;
        m_climberRight = climberRight;

    }
   
    public void setTarget(double lPos, double rPos) {
        m_climberLeft.setSpeed(lPos);
        m_climberRight.setSpeed(rPos);
    }
   
    public void setOneTarget(boolean isLeft, double pos) {
        if (isLeft) {
            m_climberLeft.setSpeed(pos);
        } else {
            m_climberRight.setSpeed(pos);
        }
    }

    // public double getTarget() {
    //     return m_climberLeft.getTarget();
    // }

    // public double getLeftPos() {
    //     return m_climberLeft.getPos();
    // }

    // public double getRightPos() {
    //     return m_climberRight.getPos();
    // }

    public double getLeftVoltage() {
        return m_climberLeft.getVoltage();
    }

    public double getRightVoltage() {
        return m_climberRight.getVoltage();
    }

    public void stopMotors() {
        m_climberLeft.setSpeed(0.0);
        m_climberRight.setSpeed(0.0);
    }

    public Command newSetTargetCommand(double lPos, double rPos) {
        return new InstantCommand(() -> setTarget(lPos, rPos), this);
    }

    public Command newSetLeftTargetCommand(double lPos) {
        return new InstantCommand(() -> setOneTarget(true, lPos), this);
    }

    public Command newSetRightTargetCommand(double rPos) {
        return new InstantCommand(() -> setOneTarget(false, rPos), this);
    }
    
    public Command newStopMotorCommand() {
        return new InstantCommand(() -> stopMotors(), this);
    }
}

