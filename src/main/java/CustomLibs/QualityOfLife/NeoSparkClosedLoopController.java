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
 import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.spark.SparkBase;

import CustomLibs.QualityOfLife.NeoSparkBase.ControlType;
 
 /** Get an instance of this class by using {@link SparkBase#getClosedLoopController()}. */
 public class NeoSparkClosedLoopController {
   private final NeoSparkBase spark;
 
   public enum ArbFFUnits {
     kVoltage(0),
     kPercentOut(1);
 
     @SuppressWarnings("MemberName")
     public final int value;
 
     ArbFFUnits(int value) {
       this.value = value;
     }
 
     public static ArbFFUnits fromInt(int value) {
       switch (value) {
         case 0:
           return kVoltage;
         case 1:
           return kPercentOut;
         default:
           return kVoltage;
       }
     }
   }
 
   // package-private (can only be used by other classes in this package)
   NeoSparkClosedLoopController(NeoSparkBase device) {
     spark = device;
   }
 
   /**
    * Set the controller reference value based on the selected control mode.
    *
    * @param value The value to set depending on the control mode. For basic duty cycle control this
    *     should be a value between -1 and 1 Otherwise: Voltage Control: Voltage (volts) Velocity
    *     Control: Velocity (RPM) Position Control: Position (Rotations) Current Control: Current
    *     (Amps). Native units can be changed using the setPositionConversionFactor() or
    *     setVelocityConversionFactor() methods of the RelativeEncoder class
    * @param ctrl the control type
    * @return {@link REVLibError#kOk} if successful
    */
   public REVLibError setReference(double value, SparkBase.ControlType ctrl) {
     spark.throwIfClosed();
     return setReference(value, ctrl, 0);
   }
 
   /**
    * Set the controller reference value based on the selected control mode. This will override the
    * pre-programmed control mode but not change what is programmed to the controller.
    *
    * @param value The value to set depending on the control mode. For basic duty cycle control this
    *     should be a value between -1 and 1 Otherwise: Voltage Control: Voltage (volts) Velocity
    *     Control: Velocity (RPM) Position Control: Position (Rotations) Current Control: Current
    *     (Amps). Native units can be changed using the setPositionConversionFactor() or
    *     setVelocityConversionFactor() methods of the RelativeEncoder class
    * @param ctrl Is the control type to override with
    * @param pidSlot for this command
    * @return {@link REVLibError#kOk} if successful
    */
   public REVLibError setReference(double value, SparkBase.ControlType ctrl, int pidSlot) {
     spark.throwIfClosed();
     return setReference(value, ctrl, pidSlot, 0);
   }
 
   /**
    * Set the controller reference value based on the selected control mode. This will override the
    * pre-programmed control mode but not change what is programmed to the controller.
    *
    * @param value The value to set depending on the control mode. For basic duty cycle control this
    *     should be a value between -1 and 1 Otherwise: Voltage Control: Voltage (volts) Velocity
    *     Control: Velocity (RPM) Position Control: Position (Rotations) Current Control: Current
    *     (Amps). Native units can be changed using the setPositionConversionFactor() or
    *     setVelocityConversionFactor() methods of the RelativeEncoder class
    * @param ctrl Is the control type to override with
    * @param pidSlot for this command
    * @param arbFeedforward A value from which is represented in voltage applied to the motor after
    *     the result of the specified control mode. The units for the parameter is Volts. This value
    *     is set after the control mode, but before any current limits or ramp rates.
    * @return {@link REVLibError#kOk} if successful
    */
   public REVLibError setReference(
       double value, SparkBase.ControlType ctrl, int pidSlot, double arbFeedforward) {
     spark.throwIfClosed();
     return spark.setpointCommand(value, ctrl, pidSlot, arbFeedforward);
   }
 
   /**
    * Set the controller reference value based on the selected control mode. This will override the
    * pre-programmed control mode but not change what is programmed to the controller.
    *
    * @param value The value to set depending on the control mode. For basic duty cycle control this
    *     should be a value between -1 and 1 Otherwise: Voltage Control: Voltage (volts) Velocity
    *     Control: Velocity (RPM) Position Control: Position (Rotations) Current Control: Current
    *     (Amps). Native units can be changed using the setPositionConversionFactor() or
    *     setVelocityConversionFactor() methods of the RelativeEncoder class
    * @param ctrl Is the control type to override with
    * @param pidSlot for this command
    * @param arbFeedforward A value from which is represented in voltage applied to the motor after
    *     the result of the specified control mode. The units for the parameter is Volts. This value
    *     is set after the control mode, but before any current limits or ramp rates.
    * @param arbFFUnits The units the arbitrary feed forward term is in
    * @return {@link REVLibError#kOk} if successful
    */
   public REVLibError setReference(
       double value,
       SparkBase.ControlType ctrl,
       int pidSlot,
       double arbFeedforward,
       ArbFFUnits arbFFUnits) {
     spark.throwIfClosed();
     return spark.setpointCommand(value, ctrl, pidSlot, arbFeedforward, arbFFUnits.value);
   }
 
   /**
    * Set the I accumulator of the closed loop controller. This is useful when wishing to force a
    * reset on the I accumulator of the closed loop controller. You can also preset values to see how
    * it will respond to certain I characteristics
    *
    * <p>To use this function, the controller must be in a closed loop control mode by calling
    * setReference()
    *
    * @param iAccum The value to set the I accumulator to
    * @return {@link REVLibError#kOk} if successful
    */
   public REVLibError setIAccum(double iAccum) {
     spark.throwIfClosed();
     return REVLibError.fromInt(CANSparkJNI.c_Spark_SetIAccum(spark.sparkHandle, (float) iAccum));
   }
 
   /**
    * Get the I accumulator of the closed loop controller. This is useful when wishing to see what
    * the I accumulator value is to help with PID tuning
    *
    * @return The value of the I accumulator
    */
   public double getIAccum() {
     spark.throwIfClosed();
     return CANSparkJNI.c_Spark_GetIAccum(spark.sparkHandle);
   }
 }
 