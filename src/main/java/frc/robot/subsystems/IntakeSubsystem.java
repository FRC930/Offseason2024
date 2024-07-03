package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IOs.TalonRollerIO;


public class IntakeSubsystem extends SubsystemBase {

    private TalonRollerIO m_intakeMotor;

    public IntakeSubsystem(TalonRollerIO intake)  {
        m_intakeMotor = intake;

        

        m_intakeMotor.getTalon().setNeutralMode(NeutralModeValue.Coast);

        // m_intakeMotor.getTalon().setInverted(true);
    }

     /**
    * <h3>setIntakeSpeed</h3>
    * @param speed the speed the motor will be set to
    */
    public void setSpeed(double speed) {
        m_intakeMotor.setSpeed(speed);
    }

    /**
    * <h3>stop</h3>
    * This sets the intake's speed to 0
    */
    public void stop() {
        setSpeed(0);
    }

    /**
    * <h3>getSpeed</h3>
    * @return the speed of the intake
    */
    public double getSpeed() {
        return m_intakeMotor.getSpeed();
    }

    /**
     * <h3>getVoltage</h3>
     * @return current applied voltage to Talon
     */
    public double getVoltage() {
        return m_intakeMotor.getVoltage();
    }

    public void periodic() {
        m_intakeMotor.runSim();
        
    }

    public InstantCommand newSetSpeedCommand(double speed) {
        return new InstantCommand(() -> setSpeed(speed), this);
    }
}
