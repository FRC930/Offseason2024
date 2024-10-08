package frc.robot.IOs.implementations;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TalonVelocityIOSim extends TalonVelocityIORobot {
    private DCMotorSim m_motorSim;
    private final double kMotorResistance = 0.002;

    public TalonVelocityIOSim(int MotorID, double gearRatio, Slot0Configs config, Slot1Configs config1, MotionMagicConfigs mmConfigs) {
        super(MotorID, gearRatio, config, config1, mmConfigs,false, new MotionMagicVelocityVoltage(0,0,true,0,0,false,false,false));
        m_motorSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), gearRatio, 0.001);
    }

    @Override
    public void runSim() {
        /// DEVICE SPEED SIMULATION
        var motorVoltage = m_motor.getSimState().getMotorVoltage();
        m_motorSim.setInputVoltage(motorVoltage);

        m_motorSim.update(getPeriod());
    
        /// SET SIM PHYSICS INPUTS
        final double position_rot = m_motorSim.getAngularPositionRotations();
        final double velocity_rps = Units.radiansToRotations(m_motorSim.getAngularVelocityRadPerSec());
    
        m_motor.getSimState().setRawRotorPosition(position_rot);
        m_motor.getSimState().setRotorVelocity(velocity_rps);
        
        double supplyVoltage = 12 - m_motor.getSimState().getSupplyCurrent() * kMotorResistance;
        m_motor.getSimState().setSupplyVoltage(supplyVoltage);
    }
}
