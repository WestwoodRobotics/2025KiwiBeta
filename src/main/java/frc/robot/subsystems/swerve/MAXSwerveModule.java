// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import CustomLibs.QualityOfLife.NeoSparkClosedLoopController;
import CustomLibs.QualityOfLife.NeoSparkMax;
import CustomLibs.QualityOfLife.NeoSparkFlex;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import CustomLibs.QualityOfLife.NeoSparkBase;

import frc.robot.Constants.ModuleConstants;

public class MAXSwerveModule {
  private final NeoSparkMax m_drivingSpark;
  private final NeoSparkMax m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final NeoSparkClosedLoopController m_drivingClosedLoopController;
  private final NeoSparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    

    // m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    // m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_drivingSpark = new NeoSparkMax(drivingCANId, NeoSparkMax.MotorType.kBrushless);
    m_drivingSpark.setPIDF(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD, ModuleConstants.kDrivingFF);
    m_drivingSpark.setOutputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);
    m_drivingSpark.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    m_drivingSpark.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    m_drivingSpark.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    m_drivingSpark.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
    m_drivingSpark.configure(m_drivingSpark.getCurrentConfig());

    m_turningSpark = new NeoSparkMax(turningCANId, NeoSparkMax.MotorType.kBrushless);
    m_turningSpark.setPIDF(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD, ModuleConstants.kTurningFF);
    m_turningSpark.setOutputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput);
    m_turningSpark.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    m_turningSpark.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
    m_turningSpark.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    m_turningSpark.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
    m_turningSpark.setPositionWrappingEnabled(true);
    m_turningSpark.setPositionWrappingInputRange(ModuleConstants.kTurningEncoderPositionPIDMinInput, ModuleConstants.kTurningEncoderPositionPIDMaxInput);
    m_turningSpark.configure(m_turningSpark.getCurrentConfig());

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();
    
    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingSpark.setReference(correctedDesiredState.speedMetersPerSecond, NeoSparkBase.ControlType.kVelocity);
    m_turningSpark.setReference(correctedDesiredState.angle.getRadians(), NeoSparkBase.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}