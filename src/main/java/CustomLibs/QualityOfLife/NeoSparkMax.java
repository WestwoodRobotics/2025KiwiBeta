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
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.config.SparkBaseConfig;
 import com.revrobotics.spark.config.SparkMaxConfigAccessor;
 import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.utils.SparkModels;
 
 public class NeoSparkMax extends NeoSparkBase {
   private NeoSparkMaxAlternateEncoder altEncoder;
   private NeoSparkBaseConfig current_config;
   private final Object altEncoderLock = new Object();
 
   // package-private
   enum DataPortConfig {
     kDefault(0),
     kAlternateEncoder(1);
 
     public final int value;
 
     DataPortConfig(int value) {
       this.value = value;
     }
   }
 
   /**
    * Accessor for SPARK parameter values. This object contains fields and methods to retrieve
    * parameters that have been applied to the device. To set parameters, see {@link SparkBaseConfig}
    * and {@link SparkBase#configure(SparkBaseConfig, SparkBase.ResetMode, SparkBase.PersistMode)}.
    *
    * <p>NOTE: This uses calls that are blocking to retrieve parameters and should be used
    * infrequently.
    */
   public final SparkMaxConfigAccessor configAccessor;
 
   /**
    * Create a new object to control a SPARK MAX motor Controller
    *
    * @param deviceId The device ID.
    * @param type The motor type connected to the controller. Brushless motor wires must be connected
    *     to their matching colors and the hall sensor must be plugged in. Brushed motors must be
    *     connected to the Red and Black terminals only.
    */
   public NeoSparkMax(int deviceId, MotorType type) {
     super(deviceId, type, SparkModels.SparkMax);
     configAccessor = new SparkMaxConfigAccessor(sparkHandle);
    
     if (CANSparkJNI.c_Spark_GetSparkModel(sparkHandle) != SparkModels.SparkMax.id) {
       DriverStation.reportWarning(
           "CANSparkMax object created for CAN ID "
               + deviceId
               + ", which is not a SPARK MAX. Some functionalities may not work.",
           true);
     }
   }
 
   /** ***** Extended Functions ****** */
   @Override
   public REVLibError configure(
     NeoSparkBaseConfig config, ResetMode kresetsafeparameters, PersistMode kpersistparameters) {
     current_config = config;
     REVLibError status = super.configure(config, kresetsafeparameters, kpersistparameters);
 
     synchronized (altEncoderLock) {
       if (altEncoder != null) {
         checkDataPortAlternateEncoder();
       }
     }
 
     synchronized (absoluteEncoderLock) {
       if (absoluteEncoder != null) {
         checkDataPortAbsoluteEncoder();
       }
     }
 
     synchronized (forwardLimitSwitchLock) {
       if (forwardLimitSwitch != null) {
         checkDataPortLimitSwitch();
       }
     }
 
     synchronized (reverseLimitSwitchLock) {
       if (reverseLimitSwitch != null) {
         checkDataPortLimitSwitch();
       }
     }
 
     return status;
   }

   public REVLibError configure(
     NeoSparkBaseConfig config) {
     current_config = config;
     REVLibError status = super.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
 
     synchronized (altEncoderLock) {
       if (altEncoder != null) {
         checkDataPortAlternateEncoder();
       }
     }
 
     synchronized (absoluteEncoderLock) {
       if (absoluteEncoder != null) {
         checkDataPortAbsoluteEncoder();
       }
     }
 
     synchronized (forwardLimitSwitchLock) {
       if (forwardLimitSwitch != null) {
         checkDataPortLimitSwitch();
       }
     }
 
     synchronized (reverseLimitSwitchLock) {
       if (reverseLimitSwitch != null) {
         checkDataPortLimitSwitch();
       }
     }
 
     return status;
   }
 
   private void checkDataPortAlternateEncoder() {
     throwIfClosed();
     if (CANSparkJNI.c_Spark_IsDataPortConfigured(sparkHandle)
         && CANSparkJNI.c_Spark_GetDataPortConfig(sparkHandle)
             != DataPortConfig.kAlternateEncoder.value) {
       throw new IllegalStateException(
           "The SPARK MAX is not configured to use an alternate encoder.");
     }
   }
 
   private void checkDataPortAbsoluteEncoder() {
     throwIfClosed();
 
     if (CANSparkJNI.c_Spark_IsDataPortConfigured(sparkHandle)
         && CANSparkJNI.c_Spark_GetDataPortConfig(sparkHandle) != DataPortConfig.kDefault.value) {
       throw new IllegalStateException(
           "The SPARK MAX is not configured to use an absolute encoder.");
     }
   }
 
   private void checkDataPortLimitSwitch() {
     throwIfClosed();
 
     if (CANSparkJNI.c_Spark_IsDataPortConfigured(sparkHandle)
         && CANSparkJNI.c_Spark_GetDataPortConfig(sparkHandle) != DataPortConfig.kDefault.value) {
       throw new IllegalStateException("The SPARK MAX is not configured to use limit switches.");
     }
   }
 
   /**
    * Returns an object for interfacing with a quadrature encoder connected to the alternate encoder
    * mode data port pins. These are defined as:
    *
    * <ul>
    *   <li>Pin 4 (Forward Limit Switch): Index
    *   <li>Pin 6 (Multi-function): Encoder A
    *   <li>Pin 8 (Reverse Limit Switch): Encoder B
    * </ul>
    *
    * <p>This call will disable support for the limit switch inputs.
    *
    * @return An object for interfacing with a quadrature encoder connected to the alternate encoder
    *     mode data port pins
    */
   public RelativeEncoder getAlternateEncoder() {
     checkDataPortAlternateEncoder();
     synchronized (altEncoderLock) {
       if (altEncoder == null) {
         altEncoder = new NeoSparkMaxAlternateEncoder(this);
       }
       return altEncoder;
     }
   }
 
   @Override
   public NeoSparkAbsoluteEncoder getAbsoluteEncoder() {
     checkDataPortAbsoluteEncoder();
     return super.getAbsoluteEncoder();
   }
 
   @Override
   public NeoSparkLimitSwitch getForwardLimitSwitch() {
     checkDataPortLimitSwitch();
     return super.getForwardLimitSwitch();
   }
 
   @Override
   public NeoSparkLimitSwitch getReverseLimitSwitch() {
     checkDataPortLimitSwitch();
     return super.getReverseLimitSwitch();
   }

   public NeoSparkBaseConfig getCurrentConfig() {
     return current_config;
   }
 }
 