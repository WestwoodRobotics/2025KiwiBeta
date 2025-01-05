package frc.robot.subsystems.axe;

import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.config.SparkMaxConfig;

import CustomLibs.QualityOfLife.NeoSparkMax;


import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AxeConstants;


public class Axe extends SubsystemBase {

    private NeoSparkMax axeMotor;

    private double encoderOffset;

    public Axe() {
        this.axeMotor = new NeoSparkMax(AxeConstants.kAxeMotorPort, NeoSparkMax.MotorType.kBrushless);
        //this.axePIDController = new PIDController(AxeConstants.kP, AxeConstants.kI, AxeConstants.kD);
        this.axeMotor.setPID(AxeConstants.kP, AxeConstants.kI, AxeConstants.kD);
        this.axeMotor.configure();
        
        // Create and apply motor configuration
        //NeoSparkBaseConfig config = axeMotor.getCurrentConfig();
        
        // Apply the configuration to the motor
        //axeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        encoderOffset = 0;
    }

    public void setAxePower(double power) {
        axeMotor.set(power);
    }

    public double getAxePosition() {
        return axeMotor.getEncoder().getPosition() - encoderOffset;
    }

    public double getRawAxePosition(){
        return axeMotor.getEncoder().getPosition();
    }

    public NeoSparkMax getAxeMotor() {
        return axeMotor;
    }

    public void resetEncoder(){
        encoderOffset = axeMotor.getEncoder().getPosition();
    }

    @Override
    public void periodic(){
        //System.out.println(this.getAxePosition());
    }
}