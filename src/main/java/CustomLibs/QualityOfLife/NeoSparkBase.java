/*
 * Copyright (c) 2018-2024 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

 package CustomLibs.QualityOfLife;

 import com.revrobotics.REVLibError;
 import com.revrobotics.RelativeEncoder;
 import com.revrobotics.jni.CANSparkJNI;
 import com.revrobotics.jni.REVLibJNI;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import frc.robot.subsystems.utils.SparkModels;
 
 public abstract class NeoSparkBase extends NeoSparkLowLevel {
   public enum ResetMode {
     kNoResetSafeParameters(0),
     kResetSafeParameters(1);
 
     @SuppressWarnings("MemberName")
     public final int value;
 
     ResetMode(int value) {
       this.value = value;
     }
   }
 
   public enum PersistMode {
     kNoPersistParameters(0),
     kPersistParameters(1);
 
     @SuppressWarnings("MemberName")
     public final int value;
 
     PersistMode(int value) {
       this.value = value;
     }
   }
 
   public static class Faults {
     public final boolean other;
     public final boolean motorType;
     public final boolean sensor;
     public final boolean can;
     public final boolean temperature;
     public final boolean gateDriver;
     public final boolean escEeprom;
     public final boolean firmware;
 
     public Faults(int faults) {
       other = (faults & 0x1) != 0;
       motorType = (faults & 0x2) != 0;
       sensor = (faults & 0x4) != 0;
       can = (faults & 0x8) != 0;
       temperature = (faults & 0x10) != 0;
       gateDriver = (faults & 0x20) != 0;
       escEeprom = (faults & 0x40) != 0;
       firmware = (faults & 0x80) != 0;
     }
   }
 
   public static class Warnings {
     public final boolean brownout;
     public final boolean overcurrent;
     public final boolean escEeprom;
     public final boolean extEeprom;
     public final boolean sensor;
     public final boolean stall;
     public final boolean hasReset;
     public final boolean other;
 
     public Warnings(int warnings) {
       brownout = (warnings & 0x1) != 0;
       overcurrent = (warnings & 0x2) != 0;
       escEeprom = (warnings & 0x4) != 0;
       extEeprom = (warnings & 0x8) != 0;
       sensor = (warnings & 0x10) != 0;
       stall = (warnings & 0x20) != 0;
       hasReset = (warnings & 0x40) != 0;
       other = (warnings & 0x80) != 0;
     }
   }
 
   public enum ControlType {
     kDutyCycle(0),
     kVelocity(1),
     kVoltage(2),
     kPosition(3),
     /**
      * @deprecated It is recommended to migrate to MAXMotion instead.
      */
     @Deprecated(forRemoval = true)
     kSmartMotion(4),
     kCurrent(5),
     /**
      * @deprecated It is recommended to migrate to MAXMotion Velocity mode instead.
      */
     @Deprecated(forRemoval = true)
     kSmartVelocity(6),
     kMAXMotionPositionControl(7),
     kMAXMotionVelocityControl(8);
 
     @SuppressWarnings("MemberName")
     public final int value;
 
     ControlType(int value) {
       this.value = value;
     }
   }
 
   protected NeoSparkRelativeEncoder encoder;
   protected final Object encoderLock = new Object();
 
   protected NeoSparkAnalogSensor analogSensor;
   protected final Object analogSensorLock = new Object();
 
   protected NeoSparkAbsoluteEncoder absoluteEncoder;
   protected final Object absoluteEncoderLock = new Object();
 
   protected NeoSparkClosedLoopController closedLoopController;
   protected final Object closedLoopControllerLock = new Object();
 
   protected NeoSparkLimitSwitch forwardLimitSwitch;
   protected final Object forwardLimitSwitchLock = new Object();
 
   protected NeoSparkLimitSwitch reverseLimitSwitch;
   protected final Object reverseLimitSwitchLock = new Object();
 
   // Only used for get() and set() API
   private double m_setpoint = 0.0;
 
   /**
    * Create a new object to control a SPARK motor Controller
    *
    * @param deviceId The device ID.
    * @param type The motor type connected to the controller. Brushless motor wires must be connected
    *     to their matching colors and the hall sensor must be plugged in. Brushed motors must be
    *     connected to the Red and Black terminals only.
    */
   public NeoSparkBase(int deviceId, MotorType type, SparkModels model) {

     super(deviceId, type, (model == SparkModels.SparkMax) ? SparkModel.SparkMax : SparkModel.SparkFlex);
   }
 
   /** ** Speed Controller Interface *** */
   /**
    * Common interface for setting the speed of a speed controller.
    *
    * @param speed The speed to set. Value should be between -1.0 and 1.0.
    */
   @Override
   public void set(double speed) {
     throwIfClosed();
     // Only for 'get' API
     m_setpoint = speed;
     setpointCommand(speed, NeoSparkBase.ControlType.kDutyCycle);
   }
 
   /**
    * Sets the voltage output of the SpeedController. This is equivillant to a call to
    * SetReference(output, rev::ControlType::kVoltage). The behavior of this call differs slightly
    * from the WPILib documetation for this call since the device internally sets the desired voltage
    * (not a compensation value). That means that this *can* be a 'set-and-forget' call.
    *
    * @param outputVolts The voltage to output.
    */
   @Override
   public void setVoltage(double outputVolts) {
     throwIfClosed();
     setpointCommand(outputVolts, NeoSparkBase.ControlType.kVoltage);
   }
 
   /**
    * Common interface for getting the current set speed of a speed controller.
    *
    * @return The current set speed. Value is between -1.0 and 1.0.
    */
   @Override
   public double get() {
     throwIfClosed();
     return m_setpoint;
   }
 
   /**
    * Common interface for inverting direction of a speed controller.
    *
    * <p>This call has no effect if the controller is a follower. To invert a follower, see the
    * follow() method.
    *
    * @param isInverted The state of inversion, true is inverted.
    */
   @Override
   public void setInverted(boolean isInverted) {
     throwIfClosed();
     CANSparkJNI.c_Spark_SetInverted(sparkHandle, isInverted);
   }
 
   /**
    * Common interface for returning the inversion state of a speed controller.
    *
    * <p>This call has no effect if the controller is a follower.
    *
    * @return isInverted The state of inversion, true is inverted.
    */
   @Override
   public boolean getInverted() {
     throwIfClosed();
     return CANSparkJNI.c_Spark_GetInverted(sparkHandle);
   }
 
   /** Common interface for disabling a motor. */
   @Override
   public void disable() {
     throwIfClosed();
     set(0);
   }
 
   @Override
   public void stopMotor() {
     throwIfClosed();
     set(0);
   }
 
   /** ***** Extended Functions ****** */
 
   /**
    * Set the configuration for the SPARK.
    *
    * <p>If {@code resetMode} is {@link ResetMode#kResetSafeParameters}, this method will reset safe
    * writable parameters to their default values before setting the given configuration. The
    * following parameters will not be reset by this action: CAN ID, Motor Type, Idle Mode, PWM Input
    * Deadband, and Duty Cycle Offset.
    *
    * <p>If {@code persistMode} is {@link PersistMode#kPersistParameters}, this method will save all
    * parameters to the SPARK's non-volatile memory after setting the given configuration. This will
    * allow parameters to persist across power cycles.
    *
    * @param config The desired SPARK configuration
    * @param resetMode Whether to reset safe parameters before setting the configuration
    * @param persistMode Whether to persist the parameters after setting the configuration
    * @return {@link REVLibError#kOk} if successful
    */
   public REVLibError configure(
       NeoSparkBaseConfig config, ResetMode resetMode, PersistMode persistMode) {
     throwIfClosed();
     REVLibError status =
         REVLibError.fromInt(
             CANSparkJNI.c_Spark_Configure(
                 this.sparkHandle,
                 config.flatten(),
                 resetMode == ResetMode.kResetSafeParameters,
                 persistMode == PersistMode.kPersistParameters));
 
     if (status != REVLibError.kOk) {
       throw new IllegalStateException(REVLibJNI.c_REVLib_ErrorFromCode(status.value));
     }
 
     return status;
   }
 


   
   /**
    * Returns an object for interfacing with the encoder connected to the encoder pins or front port
    * of the SPARK MAX or the motor interface of the SPARK Flex.
    *
    * @return An object for interfacing with an encoder
    */
   public RelativeEncoder getEncoder() {
     throwIfClosed();
     synchronized (encoderLock) {
       if (encoder == null) {
         encoder = new NeoSparkRelativeEncoder(this);
       }
       return encoder;
     }
   }
 
   /**
    * Returns an object for interfacing with a connected analog sensor.
    *
    * @return An object for interfacing with a connected analog sensor.
    */
   public NeoSparkAnalogSensor getAnalog() {
     throwIfClosed();
     synchronized (analogSensorLock) {
       if (analogSensor == null) {
         analogSensor = new NeoSparkAnalogSensor(this);
       }
       return analogSensor;
     }
   }
 
   /**
    * Returns an object for interfacing with a connected absolute encoder.
    *
    * @return An object for interfacing with a connected absolute encoder
    */
   public NeoSparkAbsoluteEncoder getAbsoluteEncoder() {
     throwIfClosed();
     synchronized (absoluteEncoderLock) {
       if (absoluteEncoder == null) {
         absoluteEncoder = new NeoSparkAbsoluteEncoder(this);
       }
       return absoluteEncoder;
     }
   }
 
   /**
    * @return An object for interfacing with the integrated closed loop controller.
    */
   public NeoSparkClosedLoopController getClosedLoopController() {
     throwIfClosed();
     synchronized (closedLoopControllerLock) {
       if (closedLoopController == null) {
         closedLoopController = new NeoSparkClosedLoopController(this);
       }
       return closedLoopController;
     }
   }
 
   /**
    * Returns an object for interfacing with the forward limit switch connected to the appropriate
    * pins on the data port.
    *
    * @return An object for interfacing with the forward limit switch.
    */
   public NeoSparkLimitSwitch getForwardLimitSwitch() {
     throwIfClosed();
     synchronized (forwardLimitSwitchLock) {
       if (forwardLimitSwitch == null) {
         forwardLimitSwitch = new NeoSparkLimitSwitch(this, NeoSparkLimitSwitch.Direction.kForward);
       }
       return forwardLimitSwitch;
     }
   }
 
   /**
    * Returns an object for interfacing with the reverse limit switch connected to the appropriate
    * pins on the data port.
    *
    * @return An object for interfacing with the reverse limit switch.
    */
   public NeoSparkLimitSwitch getReverseLimitSwitch() {
     throwIfClosed();
     synchronized (reverseLimitSwitchLock) {
       if (reverseLimitSwitch == null) {
         reverseLimitSwitch = new NeoSparkLimitSwitch(this, NeoSparkLimitSwitch.Direction.kReverse);
       }
       return reverseLimitSwitch;
     }
   }
 
   /** Get the Motor Interface type */
   // package-private
   int getMotorInterface() {
     throwIfClosed();
     return CANSparkJNI.c_Spark_GetMotorInterface(sparkHandle);
   }
 
   /**
    * Resume follower mode if the SPARK has a valid follower mode config.
    *
    * <p>NOTE: Follower mode will start automatically upon configuring follower mode. If the SPARK
    * experiences a power cycle and has follower mode configured, follower mode will automatically
    * restart. This method is only useful after {@link #pauseFollowerMode()} has been called.
    *
    * @return {@link REVLibError#kOk} if successful
    */
   public REVLibError resumeFollowerMode() {
     throwIfClosed();
     return REVLibError.fromInt(CANSparkJNI.c_Spark_StartFollowerMode(sparkHandle));
   }
 
   /**
    * Pause follower mode.
    *
    * <p>NOTE: If the SPARK experiences a power cycle and has follower mode configured, follower mode
    * will automatically restart.
    *
    * @return {@link REVLibError#kOk} if successful
    */
   public REVLibError pauseFollowerMode() {
     throwIfClosed();
     return REVLibError.fromInt(CANSparkJNI.c_Spark_StopFollowerMode(sparkHandle));
   }
 
   /**
    * Returns whether the controller is following another controller
    *
    * @return True if this device is following another controller false otherwise
    */
   public boolean isFollower() {
     throwIfClosed();
     return CANSparkJNI.c_Spark_IsFollower(sparkHandle);
   }
 
   /**
    * Get whether the SPARK has one or more active faults.
    *
    * @return true if there is an active fault
    * @see #getFaults()
    */
   public boolean hasActiveFault() {
     throwIfClosed();
     return CANSparkJNI.c_Spark_GetFaults(sparkHandle) != 0;
   }
 
   /**
    * Get whether the SPARK has one or more sticky faults.
    *
    * @return true if there is a sticky fault
    * @see #getStickyFaults()
    */
   public boolean hasStickyFault() {
     throwIfClosed();
     return CANSparkJNI.c_Spark_GetStickyFaults(sparkHandle) != 0;
   }
 
   /**
    * Get whether the SPARK has one or more active warnings.
    *
    * @return true if there is an active warning
    * @see #getWarnings()
    */
   public boolean hasActiveWarning() {
     throwIfClosed();
     return CANSparkJNI.c_Spark_GetWarnings(sparkHandle) != 0;
   }
 
   /**
    * Get whether the SPARK has one or more sticky warnings.
    *
    * @return true if there is a sticky warning
    * @see #getStickyWarnings()
    */
   public boolean hasStickyWarning() {
     throwIfClosed();
     return CANSparkJNI.c_Spark_GetStickyWarnings(sparkHandle) != 0;
   }
 
   /**
    * Get the active faults that are currently present on the SPARK. Faults are fatal errors that
    * prevent the motor from running.
    *
    * @return A struct with each fault and their active value
    */
   public Faults getFaults() {
     throwIfClosed();
     return new Faults(CANSparkJNI.c_Spark_GetFaults(sparkHandle));
   }
 
   /**
    * Get the sticky faults that were present on the SPARK at one point since the sticky faults were
    * last cleared. Faults are fatal errors that prevent the motor from running.
    *
    * <p>Sticky faults can be cleared with {@link SparkBase#clearFaults()}.
    *
    * @return A struct with each fault and their sticky value
    */
   public Faults getStickyFaults() {
     throwIfClosed();
     return new Faults(CANSparkJNI.c_Spark_GetStickyFaults(sparkHandle));
   }
 
   /**
    * Get the active warnings that are currently present on the SPARK. Warnings are non-fatal errors.
    *
    * @return A struct with each warning and their active value
    */
   public Warnings getWarnings() {
     throwIfClosed();
     return new Warnings(CANSparkJNI.c_Spark_GetWarnings(sparkHandle));
   }
 
   /**
    * Get the sticky warnings that were present on the SPARK at one point since the sticky warnings
    * were last cleared. Warnings are non-fatal errors.
    *
    * <p>Sticky warnings can be cleared with {@link SparkBase#clearFaults()}.
    *
    * @return A struct with each warning and their sticky value
    */
   public Warnings getStickyWarnings() {
     throwIfClosed();
     return new Warnings(CANSparkJNI.c_Spark_GetStickyWarnings(sparkHandle));
   }
 
   /**
    * @return The voltage fed into the motor controller.
    */
   public double getBusVoltage() {
     throwIfClosed();
     return (double) CANSparkJNI.c_Spark_GetBusVoltage(sparkHandle);
   }
 
   /**
    * Simulation note: this value will not be updated during simulation unless {@link
    * SparkSim#iterate} is called
    *
    * @return The motor controller's applied output duty cycle.
    */
   public double getAppliedOutput() {
     throwIfClosed();
     return (double) CANSparkJNI.c_Spark_GetAppliedOutput(sparkHandle);
   }
 
   /**
    * @return The motor controller's output current in Amps.
    */
   public double getOutputCurrent() {
     throwIfClosed();
     return (double) CANSparkJNI.c_Spark_GetOutputCurrent(sparkHandle);
   }
 
   /**
    * @return The motor temperature in Celsius.
    */
   public double getMotorTemperature() {
     throwIfClosed();
     return (double) CANSparkJNI.c_Spark_GetMotorTemperature(sparkHandle);
   }
 
   /**
    * Clears all sticky faults.
    *
    * @return {@link REVLibError#kOk} if successful
    */
   public REVLibError clearFaults() {
     throwIfClosed();
     return REVLibError.fromInt(CANSparkJNI.c_Spark_ClearFaults(sparkHandle));
   }
 
   /**
    * Sets timeout for sending CAN messages with SetParameter* and GetParameter* calls. These calls
    * will block for up to this amoutn of time before returning a timeout erro. A timeout of 0 will
    * make the SetParameter* calls non-blocking, and instead will check the response in a separate
    * thread. With this configuration, any error messages will appear on the drivestration but will
    * not be returned by the GetLastError() call.
    *
    * @param milliseconds The timeout in milliseconds.
    * @return {@link REVLibError#kOk} if successful
    */
   public REVLibError setCANTimeout(int milliseconds) {
     throwIfClosed();
     return REVLibError.fromInt(CANSparkJNI.c_Spark_SetCANTimeout(sparkHandle, milliseconds));
   }
 
   /**
    * All device errors are tracked on a per thread basis for all devices in that thread. This is
    * meant to be called immediately following another call that has the possibility of returning an
    * error to validate if an error has occurred.
    *
    * @return the last error that was generated.
    */
   public REVLibError getLastError() {
     throwIfClosed();
     return REVLibError.fromInt(CANSparkJNI.c_Spark_GetLastError(sparkHandle));
   }
 
   SparkModel getSparkModel() {
     throwIfClosed();
     int model = CANSparkJNI.c_Spark_GetSparkModel(sparkHandle);
 
     return SparkModel.fromId(model);
   }
 
   // package-private
   enum DataPortConfig {
     kNone(-1, "none"),
     kLimitSwitchesAndAbsoluteEncoder(0, "limit switch and/or absolute encoder"),
     kAltEncoder(1, "alternate encoder");
 
     final int m_value;
     final String m_name;
 
     DataPortConfig(int value, String name) {
       m_value = value;
       m_name = name;
     }
 
     public static DataPortConfig fromInt(int id) {
       for (DataPortConfig type : values()) {
         if (type.m_value == id) {
           return type;
         }
       }
       return kNone;
     }
   }
 }
 