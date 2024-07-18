package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Orchestra.PlaysMusic;
import frc.robot.IOs.TalonRollerIO;

public class ShooterSubsystem extends SubsystemBase implements PlaysMusic{
   
   /**
    * <h3>ShooterSubsystem</h3>
    * This subsystem controls the shooter
    */
   
    private TalonRollerIO IO_Top;
    private TalonRollerIO IO_Bottom;

    private final double VELOCITY_DEADBAND = 0.1;
    private boolean m_reachedSetPoint = false;

    public ShooterSubsystem(TalonRollerIO TopIO, TalonRollerIO BottomIO) { 
        IO_Top = TopIO;
        IO_Bottom = BottomIO;

        IO_Top.getTalon().setInverted(false);
        IO_Bottom.getTalon().setInverted(false);
    }

    /**
     * <h3>setSpeed</h3>
     * @param topSpeed the speed the top wheel will be set to in rot/s
     * @param bottomSpeed the speed the bottom wheel will be set to in rot/s 
     */
    public void setSpeed(double topSpeed, double bottomSpeed) {
        IO_Top.setSpeed(topSpeed);
        IO_Bottom.setSpeed(bottomSpeed);
    }

    /**
     * <h3>setSpeedWithSlot</h3>
     * @param topSpeed the speed the top wheel will be set to in rot/s
     * @param bottomSpeed the speed the bottom wheel will be set to in rot/s 
     * @param slot the slot to
     */
    
    public void setVoltage(double topVoltage, double bottomVoltage) {
        IO_Top.getTalon().setVoltage(topVoltage);
        IO_Bottom.getTalon().setVoltage(bottomVoltage);
    }

    /**
     * <h3>getTopMotorSpeed</h3>
     * @return The current motor speed of the top wheel in rps
     */
    public double getTopMotorSpeed() {
        return IO_Top.getSpeed();
    }

    /**
     * <h3>getBottomMotorSpeed</h3>
     * @return The current motor speed of the bottom wheel in rps
     */
    public double getBottomMotorSpeed() {
        return IO_Bottom.getSpeed();
    }

    /**
     * <h3>getTopVoltage</h3>
     * @return The current voltage of the top motor
     */
    public double getTopVoltage() {
        return IO_Top.getVoltage();
    }

    /**
     * <h3>getBottomVoltage</h3>
     * @return The current voltage of the bottom motor
     */
    public double getBottomVoltage() {
        return IO_Bottom.getVoltage();
    }

    // /**
    //  * <h3>getTopTargetVelocity</h3>
    //  * @return The current voltage of the bottom motor
    //  */
    // public double getTopTargetVelocity() {
    //     return IO_Top.getTargetVelocity();
    // }

    // /**
    //  * <h3>getBottomTargetVelocity</h3>
    //  * @return The current voltage of the bottom motor
    //  */
    // public double getBottomTargetVelocity() {
    //     return IO_Bottom.getTargetVelocity();
    // }

    /**
     * <h3>stop</h3>
     * This sets the shooter's speed to 0
     */
    public void stop() {
        setSpeed(0.0,0.0);
    }

    @Override
    public void periodic() {
        IO_Top.runSim();
        IO_Bottom.runSim();
        
    
    }

    public Command newSetSpeedsCommand(double topSpeed, double bottomSpeed) {
        return new InstantCommand(() -> setSpeed(topSpeed, bottomSpeed), this);
    }

    // public boolean atSetpoint() {
    //     m_reachedSetPoint = MathUtil.applyDeadband(getBottomTargetVelocity() - getBottomMotorSpeed(), VELOCITY_DEADBAND) == 0.0
    //         && MathUtil.applyDeadband(getTopTargetVelocity() - getTopMotorSpeed(),VELOCITY_DEADBAND) == 0.0; 
        
    //     return m_reachedSetPoint;
    // }

    // public Command newWaitUntilSetpointCommand(double timeout) {
    //     return new WaitCommand(timeout).until(() -> atSetpoint());
    // }

    //Orchestra
    @Override
    public TalonFX[] getInstruments() {
        return new TalonFX[] {IO_Top.getTalon(),IO_Bottom.getTalon()};
    }
    @Override
    public Subsystem[] getSubsystems() {
        return new Subsystem[] {this};
    }
}
