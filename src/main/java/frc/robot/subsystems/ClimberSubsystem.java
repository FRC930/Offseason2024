package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.CANVenom.BrakeCoastMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IOs.TalonPosIO;
import frc.robot.IOs.TalonRollerIO;
import frc.robot.IOs.implementations.TalonRollerIOSim;
import frc.robot.utilities.Phoenix6Utility;

public class ClimberSubsystem extends SubsystemBase {
    
    private static final double CLIMBER_FORWARD_SOFT_LIMIT = 100.0;
    private static final double CLIMBER_REVERSE_SOFT_LIMIT = 15.0;
    private TalonRollerIO m_climberLeft;
    private TalonRollerIO m_climberRight;

    private static final boolean useSmartDashboardSpeeds = false;

    private static final double statorCurrentLimit = 90.0;
    private static final double inputCurrentLimit = 60.0;


    public ClimberSubsystem(TalonRollerIO climberLeft, TalonRollerIO climberRight) {
        m_climberLeft = climberLeft;
        m_climberRight = climberRight;

        Phoenix6Utility.applyConfigAndRetry(m_climberLeft.getTalon(), () -> Phoenix6Utility.configCurrentLimits(m_climberLeft.getTalon(), true, statorCurrentLimit, true, inputCurrentLimit));
        Phoenix6Utility.applyConfigAndRetry(m_climberRight.getTalon(), () -> Phoenix6Utility.configCurrentLimits(m_climberRight.getTalon(), true, statorCurrentLimit, true, inputCurrentLimit));

        m_climberRight.getTalon().setInverted(true);

        Phoenix6Utility.configSoftLimits(climberLeft.getTalon(), CLIMBER_REVERSE_SOFT_LIMIT, CLIMBER_FORWARD_SOFT_LIMIT);
        Phoenix6Utility.configSoftLimits(climberRight.getTalon(), CLIMBER_REVERSE_SOFT_LIMIT, CLIMBER_FORWARD_SOFT_LIMIT);

        
        MotorOutputConfigs configs = new MotorOutputConfigs();

        m_climberLeft.getTalon().getConfigurator().refresh(configs);
        configs.NeutralMode = NeutralModeValue.Brake;
        Phoenix6Utility.applyConfigAndRetry(m_climberLeft.getTalon(),() -> m_climberLeft.getTalon().getConfigurator().apply(configs));

        m_climberRight.getTalon().getConfigurator().refresh(configs);
        configs.NeutralMode = NeutralModeValue.Brake;
        Phoenix6Utility.applyConfigAndRetry(m_climberRight.getTalon(),() -> m_climberRight.getTalon().getConfigurator().apply(configs));

        //Phoenix6Utility.configSlot0(climberLeft.getTalon(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        //Phoenix6Utility.configSlot0(climberRight.getTalon(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        SmartDashboard.putBoolean("ClimberSubsystem/UsingSmartDashboardOverrides", useSmartDashboardSpeeds);
        if(useSmartDashboardSpeeds) {
            SmartDashboard.putNumber("ClimberSubsystem/MotorSpeedOverride", 0.0);
        }
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

    /**
     * Disables soft limits and slowly moves motors on start. Sets encoder zeros and reenables soft limits on end. <p>
     */
    public Command newZeroMotorsCommand() {
        return new StartEndCommand(
            () -> {
                setSoftLimitsEnabled(m_climberLeft,false);
                setSoftLimitsEnabled(m_climberRight,false);
                setOneTarget(true, -0.1);
                setOneTarget(false, -0.1);
            }, 
            () -> {
                setSoftLimitsEnabled(m_climberLeft,true);
                setSoftLimitsEnabled(m_climberRight,true);
                m_climberLeft.setSpeed(0);
                m_climberRight.setSpeed(0);
            }, 
            this
        );
    }

    /**
     * Disables soft limits and slowly moves left motor on start. Sets encoder zero and reenables soft limits on end. <p>
     */
    public Command newZeroLeftMotorCommand() {
        return new StartEndCommand(
            () -> {
                setSoftLimitsEnabled(m_climberLeft,false);
                setOneTarget(true, -0.1);
            }, 
            () -> {
                setSoftLimitsEnabled(m_climberLeft,true);
                m_climberLeft.setSpeed(0);
            }, 
            this
        );
    }

    /**
     * Disables soft limits and slowly moves left motor on start. Sets encoder zero and reenables soft limits on end. <p>
     */
    public Command newZeroRightMotorCommand() {
        return new StartEndCommand(
            () -> {
                setSoftLimitsEnabled(m_climberRight,false);
                setOneTarget(false, -0.1);
            }, 
            () -> {
                setSoftLimitsEnabled(m_climberRight,true);
                m_climberRight.setSpeed(0);
            }, 
            this
        );
    }

    private static void setSoftLimitsEnabled(TalonRollerIO motor, boolean set) {
        Phoenix6Utility.applyConfigAndRetry(motor.getTalon(),
            () -> {
                SoftwareLimitSwitchConfigs configs = new SoftwareLimitSwitchConfigs();
                configs.ForwardSoftLimitEnable = set;
                configs.ForwardSoftLimitThreshold = CLIMBER_FORWARD_SOFT_LIMIT;
                configs.ReverseSoftLimitEnable = set;
                configs.ReverseSoftLimitThreshold = CLIMBER_REVERSE_SOFT_LIMIT;
                return motor.getTalon().getConfigurator().apply(configs);
            }
        );
    }
}

