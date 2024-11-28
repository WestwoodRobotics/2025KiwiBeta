/*
 * Copyright (c) 2024 REV Robotics
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

import java.util.ArrayList;

import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SmartMotionConfig;

public class NeoClosedLoopConfig extends NeoBaseConfig {
   public final SmartMotionConfig smartMotion = new SmartMotionConfig();
   public final MAXMotionConfig maxMotion = new MAXMotionConfig();
 
   public enum FeedbackSensor {
     kNoSensor(0),
     kPrimaryEncoder(1),
     kAnalogSensor(2),
     kAlternateOrExternalEncoder(3),
     kAbsoluteEncoder(4);
 
     @SuppressWarnings("MemberName")
     public final int value;
 
     FeedbackSensor(int value) {
       this.value = value;
     }
 
     static FeedbackSensor fromId(int id) {
       switch (id) {
         case 1:
           return kPrimaryEncoder;
         case 2:
           return kAnalogSensor;
         case 3:
           return kAlternateOrExternalEncoder;
         case 4:
           return kAbsoluteEncoder;
         default:
           return kNoSensor;
       }
     }
   }
 
   public enum ClosedLoopSlot {
     kSlot0(0),
     kSlot1(1),
     kSlot2(2),
     kSlot3(3);
 
     @SuppressWarnings("MemberName")
     public final int value;
 
     ClosedLoopSlot(int value) {
       this.value = value;
     }
   }
 
   /**
    * Applies settings from another {@link NeoClosedLoopConfig} to this one, including all of its nested
    * configurations.
    *
    * <p>Settings in the provided config will overwrite existing values in this object. Settings not
    * specified in the provided config remain unchanged.
    *
    * @param config The {@link NeoClosedLoopConfig} to copy settings from
    * @return The updated {@link NeoClosedLoopConfig} for method chaining
    */
   public NeoClosedLoopConfig apply(NeoClosedLoopConfig config) {
     super.apply(config);
     this.smartMotion.apply(config.smartMotion);
     this.maxMotion.apply(config.maxMotion);
     return this;
   }
 
   /**
    * Applies settings from a {@link SmartMotionConfig} to this {@link NeoClosedLoopConfig}.
    *
    * <p>Settings in the provided config will overwrite existing values in this object. Settings not
    * specified in the provided config remain unchanged.
    *
    * @param config The {@link SmartMotionConfig} to copy settings from
    * @return The updated {@link NeoClosedLoopConfig} for method chaining
    */
   public NeoClosedLoopConfig apply(SmartMotionConfig config) {
     this.smartMotion.apply(config);
     return this;
   }
 
   /**
    * Applies settings from a {@link MAXMotionConfig} to this {@link NeoClosedLoopConfig}.
    *
    * <p>Settings in the provided config will overwrite existing values in this object. Settings not
    * specified in the provided config remain unchanged.
    *
    * @param config The {@link MAXMotionConfig} to copy settings from
    * @return The updated {@link NeoClosedLoopConfig} for method chaining
    */
   public NeoClosedLoopConfig apply(MAXMotionConfig config) {
     this.maxMotion.apply(config);
     return this;
   }
 
   /**
    * Set the PIDF gains of the controller. This will set the gains for closed loop slot 0.
    *
    * <p>To set the gains for a specific closed loop slot, use {@link NeoClosedLoopConfig#pidf(double,
    * double, double, double, ClosedLoopSlot)}.
    *
    * @param p The proportional gain value
    * @param i The integral gain value
    * @param d The derivative gain value
    * @param ff The velocity feedforward value
    * @return The modified {@link NeoClosedLoopConfig} object for method chaining
    */
   public NeoClosedLoopConfig pidf(double p, double i, double d, double ff) {
     return pidf(p, i, d, ff, ClosedLoopSlot.kSlot0);
   }
 
   /**
    * Set the PIDF gains of the controller for a specific closed loop slot.
    *
    * @param p The proportional gain value
    * @param i The integral gain value
    * @param d The derivative gain value
    * @param ff The velocity feedforward value
    * @param slot The closed loop slot to set the values for
    * @return The modified {@link NeoClosedLoopConfig} object for method chaining
    */
   public NeoClosedLoopConfig pidf(double p, double i, double d, double ff, ClosedLoopSlot slot) {
     putParameter(SparkParameter.kP_0.value + slot.value * 8, (float) p);
     putParameter(SparkParameter.kI_0.value + slot.value * 8, (float) i);
     putParameter(SparkParameter.kD_0.value + slot.value * 8, (float) d);
     putParameter(SparkParameter.kF_0.value + slot.value * 8, (float) ff);
     return this;
   }
 
   /**
    * Set the PID gains of the controller. This will set the gains for closed loop slot 0.
    *
    * <p>To set the gains for a specific closed loop slot, use {@link NeoClosedLoopConfig#pid(double,
    * double, double, ClosedLoopSlot)}.
    *
    * @param p The proportional gain value
    * @param i The integral gain value
    * @param d The derivative gain value
    * @return The modified {@link NeoClosedLoopConfig} object for method chaining
    */
   public NeoClosedLoopConfig pid(double p, double i, double d) {
     return pid(p, i, d, ClosedLoopSlot.kSlot0);
   }
 
   /**
    * Set the PID gains of the controller for a specific closed loop slot.
    *
    * @param p The proportional gain value
    * @param i The integral gain value
    * @param d The derivative gain value
    * @param slot The closed loop slot to set the values for
    * @return The modified {@link NeoClosedLoopConfig} object for method chaining
    */
   public NeoClosedLoopConfig pid(double p, double i, double d, ClosedLoopSlot slot) {
     putParameter(SparkParameter.kP_0.value + slot.value * 8, (float) p);
     putParameter(SparkParameter.kI_0.value + slot.value * 8, (float) i);
     putParameter(SparkParameter.kD_0.value + slot.value * 8, (float) d);
     return this;
   }
 
   /**
    * Set the proportional gain of the controller.
    *
    * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
    * slot, use {@link NeoClosedLoopConfig#p(double, ClosedLoopSlot)}.
    *
    * @param p The proportional gain value
    * @return The modified {@link NeoClosedLoopConfig} object for method chaining
    */
   public NeoClosedLoopConfig p(double p) {
     return p(p, ClosedLoopSlot.kSlot0);
   }
 
   /**
    * Set the proportional gain of the controller for a specific closed loop slot.
    *
    * @param p The proportional gain value
    * @param slot The closed loop slot to set the values for
    * @return The modified {@link NeoClosedLoopConfig} object for method chaining
    */
   public NeoClosedLoopConfig p(double p, ClosedLoopSlot slot) {
     putParameter(SparkParameter.kP_0.value + slot.value * 8, (float) p);
     return this;
   }
 
   /**
    * Set the integral gain of the controller.
    *
    * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
    * slot, use {@link NeoClosedLoopConfig#i(double, ClosedLoopSlot)}.
    *
    * @param i The integral gain value
    * @return The modified {@link NeoClosedLoopConfig} object for method chaining
    */
   public NeoClosedLoopConfig i(double i) {
     return i(i, ClosedLoopSlot.kSlot0);
   }
 
   /**
    * Set the integral gain of the controller for a specific closed loop slot.
    *
    * @param i The integral gain value
    * @param slot The closed loop slot to set the values for
    * @return The modified {@link NeoClosedLoopConfig} object for method chaining
    */
   public NeoClosedLoopConfig i(double i, ClosedLoopSlot slot) {
     putParameter(SparkParameter.kI_0.value + slot.value * 8, (float) i);
     return this;
   }
 
   /**
    * Set the derivative gain of the controller.
    *
    * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
    * slot, use {@link NeoClosedLoopConfig#d(double, ClosedLoopSlot)}.
    *
    * @param d The derivative gain value
    * @return The modified {@link NeoClosedLoopConfig} object for method chaining
    */
   public NeoClosedLoopConfig d(double d) {
     return d(d, ClosedLoopSlot.kSlot0);
   }
 
   /**
    * Set the derivative gain of the controller for a specific closed loop slot.
    *
    * @param d The derivative gain value
    * @param slot The closed loop slot to set the values for
    * @return The modified {@link NeoClosedLoopConfig} object for method chaining
    */
   public NeoClosedLoopConfig d(double d, ClosedLoopSlot slot) {
     putParameter(SparkParameter.kD_0.value + slot.value * 8, (float) d);
     return this;
   }
 
   /**
    * Set the velocity feedforward gain of the controller.
    *
    * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
    * slot, use {@link NeoClosedLoopConfig#velocityFF(double, ClosedLoopSlot)}.
    *
    * @param ff The velocity feedforward gain value
    * @return The modified {@link NeoClosedLoopConfig} object for method chaining
    */
   public NeoClosedLoopConfig velocityFF(double ff) {
     return velocityFF(ff, ClosedLoopSlot.kSlot0);
   }
 
   /**
    * Set the velocity feedforward gain of the controller for a specific closed loop slot.
    *
    * @param ff The velocity feedforward gain value
    * @param slot The closed loop slot to set the values for
    * @return The modified {@link NeoClosedLoopConfig} object for method chaining
    */
   public NeoClosedLoopConfig velocityFF(double ff, ClosedLoopSlot slot) {
     putParameter(SparkParameter.kF_0.value + slot.value * 8, (float) ff);
     return this;
   }
 
   /**
    * Set the derivative filter of the controller.
    *
    * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
    * slot, use {@link NeoClosedLoopConfig#dFilter(double, ClosedLoopSlot)}.
    *
    * @param dFilter The derivative filter value
    * @return The modified {@link NeoClosedLoopConfig} object for method chaining
    */
   public NeoClosedLoopConfig dFilter(double dFilter) {
     return dFilter(dFilter, ClosedLoopSlot.kSlot0);
   }
 
   /**
    * Set the derivative filter of the controller for a specific closed loop slot.
    *
    * @param dFilter The derivative filter value
    * @param slot The closed loop slot to set the values for
    * @return The modified {@link NeoClosedLoopConfig} object for method chaining
    */
   public NeoClosedLoopConfig dFilter(double dFilter, ClosedLoopSlot slot) {
     putParameter(SparkParameter.kDFilter_0.value + slot.value * 8, (float) dFilter);
     return this;
   }
 
   /**
    * Set the integral zone of the controller.
    *
    * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
    * slot, use {@link NeoClosedLoopConfig#dFilter(double, ClosedLoopSlot)}.
    *
    * @param iZone The integral zone value
    * @return The modified {@link NeoClosedLoopConfig} object for method chaining
    */
   public NeoClosedLoopConfig iZone(double iZone) {
     return iZone(iZone, ClosedLoopSlot.kSlot0);
   }
 
   /**
    * Set the integral zone of the controller for a specific closed loop slot.
    *
    * @param iZone The integral zone value
    * @param slot The closed loop slot to set the values for
    * @return The modified {@link NeoClosedLoopConfig} object for method chaining
    */
   public NeoClosedLoopConfig iZone(double iZone, ClosedLoopSlot slot) {
     putParameter(SparkParameter.kIZone_0.value + slot.value * 8, (float) iZone);
     return this;
   }
 
   /**
    * Set the minimum output of the controller.
    *
    * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
    * slot, use {@link NeoClosedLoopConfig#minOutput(double, ClosedLoopSlot)}.
    *
    * @param minOutput The minimum output value in the range [-1, 1]
    * @return The modified {@link NeoClosedLoopConfig} object for method chaining
    */
   public NeoClosedLoopConfig minOutput(double minOutput) {
     return minOutput(minOutput, ClosedLoopSlot.kSlot0);
   }
 
   /**
    * Set the minimum output of the controller for a specific closed loop slot.
    *
    * @param minOutput The minimum output value in the range [-1, 1]
    * @param slot The closed loop slot to set the values for
    * @return The modified {@link NeoClosedLoopConfig} object for method chaining
    */
   public NeoClosedLoopConfig minOutput(double minOutput, ClosedLoopSlot slot) {
     putParameter(SparkParameter.kOutputMin_0.value + slot.value * 8, (float) minOutput);
     return this;
   }
 
   /**
    * Set the maximum output of the controller.
    *
    * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
    * slot, use {@link NeoClosedLoopConfig#maxOutput(double, ClosedLoopSlot)}.
    *
    * @param maxOutput The maximum output value in the range [-1, 1]
    * @return The modified {@link NeoClosedLoopConfig} object for method chaining
    */
   public NeoClosedLoopConfig maxOutput(double maxOutput) {
     return maxOutput(maxOutput, ClosedLoopSlot.kSlot0);
   }
 
   /**
    * Set the maximum output of the controller for a specific closed loop slot.
    *
    * @param maxOutput The maximum output value in the range [-1, 1]
    * @param slot The closed loop slot to set the values for
    * @return The modified {@link NeoClosedLoopConfig} object for method chaining
    */
   public NeoClosedLoopConfig maxOutput(double maxOutput, ClosedLoopSlot slot) {
     putParameter(SparkParameter.kOutputMax_0.value + slot.value * 8, (float) maxOutput);
     return this;
   }
 
   /**
    * Set the output range of the controller.
    *
    * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
    * slot, use {@link NeoClosedLoopConfig#outputRange(double, double, ClosedLoopSlot)}.
    *
    * @param minOutput The minimum output value in the range [-1, 1]
    * @param maxOutput The maximum output value in the range [-1, 1]
    * @return The modified {@link NeoClosedLoopConfig} object for method chaining
    */
   public NeoClosedLoopConfig outputRange(double minOutput, double maxOutput) {
     return outputRange(minOutput, maxOutput, ClosedLoopSlot.kSlot0);
   }
 
   /**
    * Set the output range of the controller for a specific closed loop slot.
    *
    * @param minOutput The minimum output value in the range [-1, 1]
    * @param maxOutput The maximum output value in the range [-1, 1]
    * @param slot The closed loop slot to set the values for
    * @return The modified {@link NeoClosedLoopConfig} object for method chaining
    */
   public NeoClosedLoopConfig outputRange(double minOutput, double maxOutput, ClosedLoopSlot slot) {
     putParameter(SparkParameter.kOutputMin_0.value + slot.value * 8, (float) minOutput);
     putParameter(SparkParameter.kOutputMax_0.value + slot.value * 8, (float) maxOutput);
     return this;
   }
 
   /**
    * Set the maximum I accumulator of the controller. This value is used to constrain the I
    * accumulator to help manage integral wind-up.
    *
    * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
    * slot, use {@link NeoClosedLoopConfig#iMaxAccum(double, ClosedLoopSlot)}.
    *
    * @param iMaxAccum The max value to constrain the I accumulator to
    * @return The modified {@link NeoClosedLoopConfig} object for method chaining
    */
   public NeoClosedLoopConfig iMaxAccum(double iMaxAccum) {
     return iMaxAccum(iMaxAccum, ClosedLoopSlot.kSlot0);
   }
 
   /**
    * Set the maximum I accumulator of the controller for a specific closed loop slot. This value is
    * used to constrain the I accumulator to help manage integral wind-up.
    *
    * @param iMaxAccum The max value to constrain the I accumulator to
    * @param slot The closed loop slot to set the values for
    * @return The modified {@link NeoClosedLoopConfig} object for method chaining
    */
   public NeoClosedLoopConfig iMaxAccum(double iMaxAccum, ClosedLoopSlot slot) {
     putParameter(SparkParameter.kIMaxAccum_0.value + slot.value * 4, (float) iMaxAccum);
     return this;
   }
 
   /**
    * Enable or disable PID wrapping for position closed loop control.
    *
    * @param enabled True to enable position PID wrapping
    * @return The modified {@link NeoClosedLoopConfig} object for method chaining
    */
   public NeoClosedLoopConfig positionWrappingEnabled(boolean enabled) {
     putParameter(SparkParameter.kPositionPIDWrapEnable.value, enabled);
     return this;
   }
 
   /**
    * Set the minimum input value for PID wrapping with position closed loop control.
    *
    * @param minInput The value of min input for the position
    * @return The modified {@link NeoClosedLoopConfig} object for method chaining
    */
   public NeoClosedLoopConfig positionWrappingMinInput(double minInput) {
     putParameter(SparkParameter.kPositionPIDMinInput.value, (float) minInput);
     return this;
   }
 
   /**
    * Set the maximum input value for PID wrapping with position closed loop control
    *
    * @param maxInput The value of max input for the position
    * @return The modified {@link NeoClosedLoopConfig} object for method chaining
    */
   public NeoClosedLoopConfig positionWrappingMaxInput(double maxInput) {
     putParameter(SparkParameter.kPositionPIDMaxInput.value, (float) maxInput);
     return this;
   }
 
   /**
    * Set the input range for PID wrapping with position closed loop control
    *
    * @param minInput The value of min input for the position
    * @param maxInput The value of max input for the position
    * @return The modified {@link NeoClosedLoopConfig} object for method chaining
    */
   public NeoClosedLoopConfig positionWrappingInputRange(double minInput, double maxInput) {
     putParameter(SparkParameter.kPositionPIDMinInput.value, (float) minInput);
     putParameter(SparkParameter.kPositionPIDMaxInput.value, (float) maxInput);
     return this;
   }
 
   /**
    * Set the feedback sensor of the controller. The controller will use this sensor as the source of
    * feedback for its closed loop control.
    *
    * <p>The default feedback sensor is assumed to be the primary encoder for either brushless or
    * brushed mode. This can be changed to another feedback sensor for the controller such as an
    * analog sensor, absolute encoder, or alternate/external encoder.
    *
    * @param sensor The feedback sensor
    * @return The modified {@link NeoClosedLoopConfig} object for method chaining
    */
   public NeoClosedLoopConfig feedbackSensor(FeedbackSensor sensor) {
     putParameter(SparkParameter.kClosedLoopControlSensor.value, sensor.value);
     return this;
   }
 
   @Override
   public String flatten() {
     String flattenedString = "";
 
     flattenedString += super.flatten();
     flattenedString += smartMotion.flatten();
     flattenedString += maxMotion.flatten();
 
     return flattenedString;
   }

  public NeoClosedLoopConfig f(double f) {
    this.f(f, ClosedLoopSlot.kSlot0);
    return this;

  }
  public NeoClosedLoopConfig f(double f, ClosedLoopSlot slot) {
    putParameter(SparkParameter.kF_0.value + slot.value * 8, (float) f);
    return this;
  }

  public boolean isPositionWrappingEnabled() {
    if (getParameter(SparkParameter.kPositionPIDWrapEnable.value) == null) {
      return false;
    }
    return (boolean) getParameter(SparkParameter.kPositionPIDWrapEnable.value);
  }

  public ArrayList<Double> getPositionWrappingInputRange() {
    if (getParameter(SparkParameter.kPositionPIDMinInput.value) == null || getParameter(SparkParameter.kPositionPIDMaxInput.value) == null) {
      return null;
    }
    ArrayList<Double> range = new ArrayList<>();
    range.add((double) getParameter(SparkParameter.kPositionPIDMinInput.value));
    range.add((double) getParameter(SparkParameter.kPositionPIDMaxInput.value));
    return range;
  }

  public double getPositionWrappingInputRangeMin() {
    if (getParameter(SparkParameter.kPositionPIDMinInput.value) == null) {
      return -999; // Some insane value
    }
    return ((Float) getParameter(SparkParameter.kPositionPIDMinInput.value)).doubleValue();
    
  }

  public double getPositionWrappingInputRangeMax() {
    if (getParameter(SparkParameter.kPositionPIDMaxInput.value) == null) {
      return -999; // Some insane value
    }
    return ((Float) getParameter(SparkParameter.kPositionPIDMaxInput.value)).doubleValue();
  }

  
 }
 