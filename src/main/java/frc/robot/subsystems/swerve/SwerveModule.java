package frc.robot.subsystems.swerve;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;


import CustomLibs.QualityOfLife.NeoSparkBase.PersistMode;
import CustomLibs.QualityOfLife.NeoSparkBase.ResetMode;
import CustomLibs.QualityOfLife.NeoSparkBaseConfig;
import CustomLibs.QualityOfLife.NeoSparkClosedLoopController;

import com.revrobotics.spark.config.SparkMaxConfig;

import CustomLibs.QualityOfLife.NeoSparkMax;
import CustomLibs.QualityOfLife.NeoSparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;
import CustomLibs.QualityOfLife.NeoSparkBase;
/**
 * Represents a swerve module with driving and turning capabilities.
 * This class encapsulates the functionality for controlling a single swerve module,
 * including motor control, encoder feedback, and state management.
 */
public class SwerveModule {
    private final NeoSparkMax drivingMotorController;
    private final NeoSparkMax turningMotorController;

    private final RelativeEncoder drivingMotorEncoder;
    private final AbsoluteEncoder turningMotorEncoder;

    private final NeoSparkClosedLoopController drivingMotorPIDController;
    private final NeoSparkClosedLoopController turningMotorPIDController;

    private double moduleChassisAngularOffset = 0;
    private SwerveModuleState moduleDesiredState = new SwerveModuleState(0.0, new Rotation2d());

    /**
     * Constructs a SwerveModule with specified CAN IDs for driving and turning motors.
     *
     * @param drivingCANId         The CAN ID for the driving motor.
     * @param turningCANId         The CAN ID for the turning motor.
     * @param chassisAngularOffset The angular offset of the module relative to the robot chassis.
     */
    public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
        drivingMotorController = new NeoSparkMax(drivingCANId, NeoSparkMax.MotorType.kBrushless);
        drivingMotorController.setPIDF(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD, ModuleConstants.kDrivingFF);
        drivingMotorController.setOutputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);
        drivingMotorController.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
        drivingMotorController.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
        drivingMotorController.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
        drivingMotorController.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
        drivingMotorController.configure(drivingMotorController.getCurrentConfig());

        turningMotorController = new NeoSparkMax(turningCANId, NeoSparkMax.MotorType.kBrushless);
        turningMotorController.setPIDF(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD, ModuleConstants.kTurningFF);
        turningMotorController.setOutputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput);
        turningMotorController.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
        turningMotorController.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
        turningMotorController.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
        turningMotorController.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
        turningMotorController.setPositionWrappingEnabled(true);
        turningMotorController.setPositionWrappingInputRange(ModuleConstants.kTurningEncoderPositionPIDMinInput, ModuleConstants.kTurningEncoderPositionPIDMaxInput);
        turningMotorController.configure(turningMotorController.getCurrentConfig());

        // Get encoders and PID controllers
        drivingMotorEncoder = drivingMotorController.getEncoder();
        turningMotorEncoder = turningMotorController.getAbsoluteEncoder();

        drivingMotorPIDController = drivingMotorController.getClosedLoopController();
        turningMotorPIDController = turningMotorController.getClosedLoopController();

        moduleChassisAngularOffset = chassisAngularOffset;
        moduleDesiredState.angle = new Rotation2d(turningMotorEncoder.getPosition());

        drivingMotorEncoder.setPosition(0);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                drivingMotorEncoder.getVelocity(),
                new Rotation2d(turningMotorEncoder.getPosition() - moduleChassisAngularOffset));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                drivingMotorEncoder.getPosition(),
                new Rotation2d(turningMotorEncoder.getPosition() - moduleChassisAngularOffset));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState(
                desiredState.speedMetersPerSecond,
                desiredState.angle.plus(Rotation2d.fromRadians(moduleChassisAngularOffset))
        );

        correctedDesiredState.optimize(new Rotation2d(turningMotorEncoder.getPosition()));

        SwerveModuleState optimizedDesiredState = correctedDesiredState;

        drivingMotorPIDController.setReference(
                optimizedDesiredState.speedMetersPerSecond,
                NeoSparkBase.ControlType.kVelocity
        );
        turningMotorPIDController.setReference(
                optimizedDesiredState.angle.getRadians(),
                NeoSparkBase.ControlType.kPosition
        );

        moduleDesiredState = desiredState;
    }

    /**
     * Zeroes all the SwerveModule encoders.
     */
    public void resetEncoders() {
        drivingMotorEncoder.setPosition(0);
    }

    /**
     * Moves the module to what it thinks is the position (0, 0).
     */
    public void moveToZero() {
        SwerveModuleState zeroState = new SwerveModuleState(0.0, new Rotation2d());
        setDesiredState(zeroState);
    }

    public RelativeEncoder getDriveEncoder() {
        return drivingMotorController.getEncoder();
    }

    public double getDriveEncoderPosition() {
        return drivingMotorEncoder.getPosition();
    }

    public double getDriveEncoderVelocity() {
        return drivingMotorEncoder.getVelocity();
    }

    public double getTurningEncoderPosition() {
        return turningMotorEncoder.getPosition();
    }

    public double getTurningMotorControllerP(){
        return turningMotorController.getP();
    }


}