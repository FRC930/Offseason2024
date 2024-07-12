package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.IOs.TalonRollerIO;
import frc.robot.IOs.TimeOfFlightIO;

public class IndexerSubsystem extends SubsystemBase{


    private TalonRollerIO m_indexerMotor;
    private TimeOfFlightIO m_indexerSensor;

    public IndexerSubsystem(TalonRollerIO indexer, TimeOfFlightIO sensor)  {
        m_indexerMotor = indexer;
        m_indexerSensor = sensor;
        

        m_indexerMotor.getTalon().setNeutralMode(NeutralModeValue.Coast);

        // m_indexerMotor.getTalon().setInverted(true);
    }

     /**
    * <h3>setIndexerSpeed</h3>
    * @param speed the speed the motor will be set to
    */
    public void setSpeed(double speed) {
        m_indexerMotor.setSpeed(speed);
    }

    /**
    * <h3>stop</h3>
    * This sets the indexer's speed to 0
    */
    public void stop() {
        setSpeed(0);
    }

    /**
    * <h3>getSpeed</h3>
    * @return the speed of the indexer
    */
    public double getSpeed() {
        return m_indexerMotor.getSpeed();
    }

    /**
     * <h3>getVoltage</h3>
     * @return current applied voltage to Talon
     */
    public double getVoltage() {
        return m_indexerMotor.getVoltage();
    }

    public void periodic() {
        m_indexerMotor.runSim();
        
    }

    public boolean isNote() {
       return m_indexerSensor.get();
        
    }

    public InstantCommand newSetSpeedCommand(double speed) {
        return new InstantCommand(() -> setSpeed(speed), this);
    }
    

    public Command newWaitUntilNoteCommand() {
        return new WaitUntilCommand(() -> isNote());
    }

}
