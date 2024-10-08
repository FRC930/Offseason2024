package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IOs.TalonPosIO;

public class ClimberSubsystem extends SubsystemBase {
    
    private TalonPosIO m_climberLeft;
    private TalonPosIO m_climberRight;

    public ClimberSubsystem( TalonPosIO climberLeft, TalonPosIO climberRight) {

        m_climberLeft = climberLeft;
        m_climberRight = climberRight;

    }
   
    public void setTarget(double pos) {
        m_climberLeft.setTarget(pos);
        m_climberRight.setTarget(pos);
    }

    public double getTarget() {
        return m_climberLeft.getTarget();
    }

    public double getLeftPos() {
        return m_climberLeft.getPos();
    }

    public double getRightPos() {
        return m_climberRight.getPos();
    }

    public double getLeftVoltage() {
        return m_climberLeft.getVoltage();
    }

    public double getRightVoltage() {
        return m_climberRight.getVoltage();
    }

    public void stopMotors() {
        m_climberLeft.stopMotor();
        m_climberRight.stopMotor();
    }

    public Command newSetTargetCommand(double pos) {
        return new InstantCommand(() -> setTarget(pos), this);
    }
    
    public Command newStopMotorCommand() {
        return new InstantCommand(() -> stopMotors(), this);
    }
}

