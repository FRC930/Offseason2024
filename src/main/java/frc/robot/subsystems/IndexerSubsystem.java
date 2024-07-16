package frc.robot.subsystems;



import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.IOs.TalonRollerIO;
import frc.robot.IOs.TimeOfFlightIO;

/**
 * <h3>IndexerSubsystem</h3>
 * This subsystem controls the indexer
 */
public class IndexerSubsystem extends SubsystemBase {
    private TimeOfFlightIO m_sensorIO;
    private boolean m_sensorStatus;
    private TalonRollerIO m_rollerTopIO;

    private Slot0Configs slot = new Slot0Configs()
        .withKP(10.0)
        .withKI(0.0) 
        .withKD(0.0) 
        .withKA(0.0) 
        .withKG(0.0)
        .withKS(0.1) 
        .withKV(0.0);
    /**
     * <h3>IndexerSubsystem</h3>
     * Contains the indexer motor and time of flight through IOs, 
     * allowing it to take a physical motor or sim representation
     * @param IndexerMotor RollerMotorIORobot (Physical) or RollerMotorIOSi (Simulation) for indexer motor
     * @param ToF TimeOfFlightIORobot (Physical) or TimeOfFlightIOSim (Simulation) for indexer sensor
     */
    public IndexerSubsystem(TalonRollerIO starIndexerMotor, TalonRollerIO IndexerMotor, TimeOfFlightIO ToF) {
        m_rollerTopIO = IndexerMotor;
        m_sensorIO = ToF;

        TalonFX topTalon = IndexerMotor.getTalon();
        
        TalonFXConfiguration cfg = new TalonFXConfiguration(); //Creates a new blank TalonFX congfiguration that will be applied to the motors in a bit
        cfg.withSlot0(slot); // The PID and FF configs
        cfg.Feedback.SensorToMechanismRatio = 0.5; //The ratio between the motor turning and the elevator moving. We may have to invert this
        


        topTalon.setInverted(true);
        topTalon.setNeutralMode(NeutralModeValue.Brake); // Applies brake mode to belt
        
        
    }

    /**
     * <h3>setAmpSpeed</h3>
     * @param speed PercentOutput speed to apply
     */
    public void setIndexerSpeed(double speed) {
        m_rollerTopIO.setSpeed(speed);
    }

     public void setIndexerVoltage(double voltage) {
        m_rollerTopIO.getTalon().setVoltage(voltage);
    }

    /**
     * <h3>setSpeed</h3>
     * @param speed PercentOutput speed to apply
     */
    public void setSpeed(double speed) {
        setIndexerSpeed(speed);
    }

    /**
     * <h3>stop</h3>
     * Sets speed of motor to 0
     */
    public void stop() {
        setSpeed(0);
    }


    /**
     * <h3>getSensor</h3>
     * @return value of indexer sensor
     */
    public boolean getSensor() {
        return m_sensorStatus;
    }

    public double getSensorDistance() {
        return m_sensorIO.getRange();
    }

    @Override
    public void periodic() {
        // m_sensorStatus = m_sensorIO.get();
        // if(m_sensorStatus) {
        //     if(!m_turnedOn) {
        //         LimelightHelpers.setLEDMode_ForceBlink("limelight-left"); 
        //         LimelightHelpers.setLEDMode_ForceBlink("limelight-back"); 
        //         m_turnedOn = true;
        //     } else {
        //         m_counter++;
        //         if(m_counter>10) {
        //             LimelightHelpers.setLEDMode_ForceOff("limelight-left"); 
        //             LimelightHelpers.setLEDMode_ForceOff("limelight-back"); 
        //         }
        //     }
        // } else {
        //     if(m_turnedOn) {
        //         LimelightHelpers.setLEDMode_ForceOff("limelight-left"); 
        //         LimelightHelpers.setLEDMode_ForceOff("limelight-back"); 
        //         m_turnedOn = false;
        //         m_counter = 0;
        //     }
        // }
        // Logger.recordOutput(this.getClass().getSimpleName() + "/Star/Velocity" ,getSpeed());
        // Logger.recordOutput(this.getClass().getSimpleName() + "/Star/SetPoint" ,Phoenix6Utility.getVelocityFromController(m_starRollerIO.getTalon(),0.0));
        // Logger.recordOutput(this.getClass().getSimpleName() + "/Star/Voltage" ,getVoltage());
        // Logger.recordOutput(this.getClass().getSimpleName() + "/Top/Velocity" ,m_rollerTopIO.getSpeed());
        // Logger.recordOutput(this.getClass().getSimpleName() + "/Top/Voltage" ,m_rollerTopIO.getVoltage());
        // Logger.recordOutput(this.getClass().getSimpleName() + "/Sensor", getSensor());
        // Logger.recordOutput(this.getClass().getSimpleName() + "/LastSensorCheck", Timer.getFPGATimestamp());
        // Logger.recordOutput(this.getClass().getSimpleName() + "/SensorRange", m_sensorIO.getRange());
    }

    public Command newSetSpeedCommand(double speed) {
        return new InstantCommand(() -> setSpeed(speed), this);
    }

    public Command newSetTopVoltageCommand(double speed) {
        return new InstantCommand(() -> setIndexerVoltage(speed), this);
    }

    public Command newUntilNoteFoundCommand() {
        if(Robot.isReal()) {
            return new WaitUntilCommand(() -> getSensor());  // DO not set subsystem since just getting sensor value
        } else {
            return new WaitCommand(.5);
        }
    }

    public Command newUntilNoNoteFoundCommand() {
        if(Robot.isReal()) {
            return new WaitUntilCommand(() -> !getSensor());  // DO not set subsystem since just getting sensor value
        } else {
            return new WaitCommand(.1);
        }
    }

}