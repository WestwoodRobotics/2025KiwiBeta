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

 import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.AnalogSensorConfig;
import com.revrobotics.spark.config.BaseConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
 
 // TODO(Noah): Look into finding a way to express "phase" that is more clear
 
 public abstract class NeoSparkBaseConfig extends NeoBaseConfig {
   public final NeoAbsoluteEncoderConfig absoluteEncoder = new NeoAbsoluteEncoderConfig();
   public final AnalogSensorConfig analogSensor = new AnalogSensorConfig();
   public final NeoEncoderConfig encoder = new NeoEncoderConfig();
   public final NeoLimitSwitchConfig limitSwitch = new NeoLimitSwitchConfig();
   public final SoftLimitConfig softLimit = new SoftLimitConfig();
   public final NeoClosedLoopConfig closedLoop = new NeoClosedLoopConfig();
   public final SignalsConfig signals = new SignalsConfig();
 
   public enum IdleMode {
     kCoast(0),
     kBrake(1);
     
 
     @SuppressWarnings("MemberName")
     public final int value;
 
     IdleMode(int value) {
       this.value = value;
     }
 
     public static IdleMode fromId(int id) {
       if (id == 1) {
         return kBrake;
       }
       return kCoast;
     }
   }
 
   /**
    * Applies settings from another {@link NeoSparkBaseConfig} to this one, including all of its nested
    * configurations.
    *
    * <p>Settings in the provided config will overwrite existing values in this object. Settings not
    * specified in the provided config remain unchanged.
    *
    * @param config The {@link NeoSparkBaseConfig} to copy settings from
    * @return The updated {@link NeoSparkBaseConfig} for method chaining
    */
   public NeoSparkBaseConfig apply(NeoSparkBaseConfig config) {
     super.apply(config);
     this.absoluteEncoder.apply(config.absoluteEncoder);
     this.analogSensor.apply(config.analogSensor);
     this.encoder.apply(config.encoder);
     this.limitSwitch.apply(config.limitSwitch);
     this.softLimit.apply(config.softLimit);
     this.closedLoop.apply(config.closedLoop);
     this.signals.apply(config.signals);
     return this;
   }
 
   /**
    * Applies settings from an {@link AbsoluteEncoderConfig} to this {@link NeoSparkBaseConfig}.
    *
    * <p>Settings in the provided config will overwrite existing values in this object. Settings not
    * specified in the provided config remain unchanged.
    *
    * @param config The {@link AbsoluteEncoderConfig} to copy settings from
    * @return The updated {@link NeoSparkBaseConfig} for method chaining
    */
   public NeoSparkBaseConfig apply(NeoAbsoluteEncoderConfig config) {
     this.absoluteEncoder.apply(config);
     return this;
   }
 
   /**
    * Applies settings from an {@link AnalogSensorConfig} to this {@link NeoSparkBaseConfig}.
    *
    * <p>Settings in the provided config will overwrite existing values in this object. Settings not
    * specified in the provided config remain unchanged.
    *
    * @param config The {@link AnalogSensorConfig} to copy settings from
    * @return The updated {@link NeoSparkBaseConfig} for method chaining
    */
   public NeoSparkBaseConfig apply(AnalogSensorConfig config) {
     this.analogSensor.apply(config);
     return this;
   }
 
   /**
    * Applies settings from an {@link EncoderConfig} to this {@link NeoSparkBaseConfig}.
    *
    * <p>Settings in the provided config will overwrite existing values in this object. Settings not
    * specified in the provided config remain unchanged.
    *
    * @param config The {@link EncoderConfig} to copy settings from
    * @return The updated {@link NeoSparkBaseConfig} for method chaining
    */
   public NeoSparkBaseConfig apply(NeoEncoderConfig config) {
     this.encoder.apply(config);
     return this;
   }
 
   /**
    * Applies settings from a {@link LimitSwitchConfig} to this {@link NeoSparkBaseConfig}.
    *
    * <p>Settings in the provided config will overwrite existing values in this object. Settings not
    * specified in the provided config remain unchanged.
    *
    * @param config The {@link LimitSwitchConfig} to copy settings from
    * @return The updated {@link NeoSparkBaseConfig} for method chaining
    */
   public NeoSparkBaseConfig apply(NeoLimitSwitchConfig config) {
     this.limitSwitch.apply(config);
     return this;
   }
 
   /**
    * Applies settings from a {@link SoftLimitConfig} to this {@link NeoSparkBaseConfig}.
    *
    * <p>Settings in the provided config will overwrite existing values in this object. Settings not
    * specified in the provided config remain unchanged.
    *
    * @param config The {@link SoftLimitConfig} to copy settings from
    * @return The updated {@link NeoSparkBaseConfig} for method chaining
    */
   public NeoSparkBaseConfig apply(SoftLimitConfig config) {
     this.softLimit.apply(config);
     return this;
   }
 
   /**
    * Applies settings from a {@link ClosedLoopConfig} to this {@link NeoSparkBaseConfig}.
    *
    * <p>Settings in the provided config will overwrite existing values in this object. Settings not
    * specified in the provided config remain unchanged.
    *
    * @param config The {@link ClosedLoopConfig} to copy settings from
    * @return The updated {@link NeoSparkBaseConfig} for method chaining
    */
   public NeoSparkBaseConfig apply(NeoClosedLoopConfig config) {
     this.closedLoop.apply(config);
     return this;
   }
 
   /**
    * Applies settings from a {@link SignalsConfig} to this {@link NeoSparkBaseConfig}.
    *
    * <p>Settings in the provided config will overwrite existing values in this object. Settings not
    * specified in the provided config remain unchanged.
    *
    * @param config The {@link SignalsConfig} to copy settings from
    * @return The updated {@link NeoSparkBaseConfig} for method chaining
    */
   public NeoSparkBaseConfig apply(SignalsConfig config) {
     this.signals.apply(config);
     return this;
   }
 
   /**
    * Sets the idle mode setting for the SPARK.
    *
    * @param idleMode Coast or brake
    * @return The modified {@link NeoSparkBaseConfig} object for method chaining
    */
   public NeoSparkBaseConfig idleMode(IdleMode idleMode) {
     putParameter(SparkParameter.kIdleMode.value, idleMode.value);
     return this;
   }
 
   /**
    * Common interface for inverting direction of a speed controller.
    *
    * <p>This call has no effect if the controller is a follower. To invert a follower, see the
    * follow() method.
    *
    * @param inverted True for inverted
    * @return The modified {@link NeoSparkBaseConfig} object for method chaining
    */
   public NeoSparkBaseConfig inverted(boolean inverted) {
     putParameter(SparkParameter.kInverted.value, inverted);
     return this;
   }
 
   /**
    * Sets the current limit in Amps.
    *
    * <p>The motor controller will reduce the controller voltage output to avoid surpassing this
    * limit. This limit is enabled by default and used for brushless only. This limit is highly
    * recommended when using the NEO brushless motor.
    *
    * <p>The NEO Brushless Motor has a low internal resistance, which can mean large current spikes
    * that could be enough to cause damage to the motor and controller. This current limit provides a
    * smarter strategy to deal with high current draws and keep the motor and controller operating in
    * a safe region.
    *
    * @param stallLimit The current limit in Amps.
    * @return The modified {@link NeoSparkBaseConfig} object for method chaining
    */
   public NeoSparkBaseConfig smartCurrentLimit(int stallLimit) {
     return smartCurrentLimit(stallLimit, 0, 20000);
   }
 
   /**
    * Sets the current limit in Amps.
    *
    * <p>The motor controller will reduce the controller voltage output to avoid surpassing this
    * limit. This limit is enabled by default and used for brushless only. This limit is highly
    * recommended when using the NEO brushless motor.
    *
    * <p>The NEO Brushless Motor has a low internal resistance, which can mean large current spikes
    * that could be enough to cause damage to the motor and controller. This current limit provides a
    * smarter strategy to deal with high current draws and keep the motor and controller operating in
    * a safe region.
    *
    * <p>The controller can also limit the current based on the RPM of the motor in a linear fashion
    * to help with controllability in closed loop control. For a response that is linear the entire
    * RPM range leave limit RPM at 0.
    *
    * @param stallLimit The current limit in Amps at 0 RPM.
    * @param freeLimit The current limit at free speed (5700RPM for NEO).
    * @return The modified {@link NeoSparkBaseConfig} object for method chaining
    */
   public NeoSparkBaseConfig smartCurrentLimit(int stallLimit, int freeLimit) {
     return smartCurrentLimit(stallLimit, freeLimit, 20000);
   }
 
   /**
    * Sets the current limit in Amps.
    *
    * <p>The motor controller will reduce the controller voltage output to avoid surpassing this
    * limit. This limit is enabled by default and used for brushless only. This limit is highly
    * recommended when using the NEO brushless motor.
    *
    * <p>The NEO Brushless Motor has a low internal resistance, which can mean large current spikes
    * that could be enough to cause damage to the motor and controller. This current limit provides a
    * smarter strategy to deal with high current draws and keep the motor and controller operating in
    * a safe region.
    *
    * <p>The controller can also limit the current based on the RPM of the motor in a linear fashion
    * to help with controllability in closed loop control. For a response that is linear the entire
    * RPM range leave limit RPM at 0.
    *
    * @param stallLimit The current limit in Amps at 0 RPM.
    * @param freeLimit The current limit at free speed (5700RPM for NEO).
    * @param limitRpm RPM less than this value will be set to the stallLimit, RPM values greater than
    *     limitRpm will scale linearly to freeLimit
    * @return The modified {@link NeoSparkBaseConfig} object for method chaining
    */
   public NeoSparkBaseConfig smartCurrentLimit(int stallLimit, int freeLimit, int limitRpm) {
     putParameter(SparkParameter.kSmartCurrentStallLimit.value, stallLimit);
     putParameter(SparkParameter.kSmartCurrentFreeLimit.value, freeLimit);
     putParameter(SparkParameter.kSmartCurrentConfig.value, limitRpm);
     return this;
   }
 
   /**
    * Sets the secondary current limit in Amps.
    *
    * <p>The motor controller will disable the output of the controller briefly if the current limit
    * is exceeded to reduce the current. This limit is a simplified 'on/off' controller. This limit
    * is enabled by default but is set higher than the default Smart Current Limit.
    *
    * <p>The time the controller is off after the current limit is reached is determined by the
    * parameter limitCycles, which is the number of PWM cycles (20kHz). The recommended value is the
    * default of 0 which is the minimum time and is part of a PWM cycle from when the over current is
    * detected. This allows the controller to regulate the current close to the limit value.
    *
    * <p>The total time is set by the equation <code>
    * t = (50us - t0) + 50us * limitCycles
    * t = total off time after over current
    * t0 = time from the start of the PWM cycle until over current is detected
    * </code>
    *
    * @param limit The current limit in Amps.
    * @return The modified {@link NeoSparkBaseConfig} object for method chaining
    */
   public NeoSparkBaseConfig secondaryCurrentLimit(double limit) {
     return secondaryCurrentLimit(limit, 0);
   }
 
   /**
    * Sets the secondary current limit in Amps.
    *
    * <p>The motor controller will disable the output of the controller briefly if the current limit
    * is exceeded to reduce the current. This limit is a simplified 'on/off' controller. This limit
    * is enabled by default but is set higher than the default Smart Current Limit.
    *
    * <p>The time the controller is off after the current limit is reached is determined by the
    * parameter limitCycles, which is the number of PWM cycles (20kHz). The recommended value is the
    * default of 0 which is the minimum time and is part of a PWM cycle from when the over current is
    * detected. This allows the controller to regulate the current close to the limit value.
    *
    * <p>The total time is set by the equation <code>
    * t = (50us - t0) + 50us * limitCycles
    * t = total off time after over current
    * t0 = time from the start of the PWM cycle until over current is detected
    * </code>
    *
    * @param limit The current limit in Amps.
    * @param chopCycles The number of additional PWM cycles to turn the driver off after overcurrent
    *     is detected.
    * @return The modified {@link NeoSparkBaseConfig} object for method chaining
    */
   public NeoSparkBaseConfig secondaryCurrentLimit(double limit, int chopCycles) {
     putParameter(SparkParameter.kCurrentChop.value, (float) limit);
     putParameter(SparkParameter.kCurrentChopCycles.value, chopCycles);
     return this;
   }
 
   /**
    * Sets the ramp rate for open loop control modes.
    *
    * <p>This is the maximum rate at which the motor controller's output is allowed to change.
    *
    * @param rate Time in seconds to go from 0 to full throttle.
    * @return The modified {@link NeoSparkBaseConfig} object for method chaining
    */
   public NeoSparkBaseConfig openLoopRampRate(double rate) {
     if (rate != 0) {
       rate = 1.0 / rate;
     }
 
     putParameter(SparkParameter.kRampRate.value, (float) rate);
     return this;
   }
 
   /**
    * Sets the ramp rate for closed loop control modes.
    *
    * <p>This is the maximum rate at which the motor controller's output is allowed to change.
    *
    * @param rate Time in seconds to go from 0 to full throttle.
    * @return The modified {@link NeoSparkBaseConfig} object for method chaining
    */
   public NeoSparkBaseConfig closedLoopRampRate(double rate) {
     if (rate != 0) {
       rate = 1.0 / rate;
     }
 
     putParameter(SparkParameter.kClosedLoopRampRate.value, (float) rate);
     return this;
   }
 
   /**
    * Sets the voltage compensation setting for all modes on the SPARK and enables voltage
    * compensation.
    *
    * @param nominalVoltage Nominal voltage to compensate output to
    * @return The modified {@link NeoSparkBaseConfig} object for method chaining
    */
   public NeoSparkBaseConfig voltageCompensation(double nominalVoltage) {
     putParameter(SparkParameter.kCompensatedNominalVoltage.value, (float) nominalVoltage);
     putParameter(SparkParameter.kVoltageCompMode.value, 2);
     return this;
   }
 
   /**
    * Disables the voltage compensation setting for all modes on the SPARK.
    *
    * @return The modified {@link NeoSparkBaseConfig} object for method chaining
    */
   public NeoSparkBaseConfig disableVoltageCompensation() {
     putParameter(SparkParameter.kVoltageCompMode.value, 0);
     return this;
   }
 
   /**
    * Causes this controller's output to mirror the provided leader.
    *
    * <p>Only voltage output is mirrored. Settings changed on the leader do not affect the follower.
    *
    * <p>The motor will spin in the same direction as the leader. This can be changed by passing a
    * true constant after the leader parameter.
    *
    * <p>Following anything other than a CAN-enabled SPARK is not officially supported.
    *
    * @param leader The motor controller to follow.
    * @return The modified {@link NeoSparkBaseConfig} object for method chaining
    */
   public NeoSparkBaseConfig follow(SparkBase leader) {
     return follow(leader.getDeviceId());
   }
 
   /**
    * Causes this controller's output to mirror the provided leader.
    *
    * <p>Only voltage output is mirrored. Settings changed on the leader do not affect the follower.
    *
    * <p>Following anything other than a CAN-enabled SPARK is not officially supported.
    *
    * @param leader The motor controller to follow.
    * @param invert Set the follower to output opposite of the leader
    * @return The modified {@link NeoSparkBaseConfig} object for method chaining
    */
   public NeoSparkBaseConfig follow(SparkBase leader, boolean invert) {
     return follow(leader.getDeviceId(), invert);
   }
 
   /**
    * Causes this controller's output to mirror the provided leader.
    *
    * <p>Only voltage output is mirrored. Settings changed on the leader do not affect the follower.
    *
    * <p>The motor will spin in the same direction as the leader. This can be changed by passing a
    * true constant after the deviceID parameter.
    *
    * <p>Following anything other than a CAN-enabled SPARK is not officially supported.
    *
    * @param leaderCanId The CAN ID of the device to follow.
    * @return The modified {@link NeoSparkBaseConfig} object for method chaining
    */
   public NeoSparkBaseConfig follow(int leaderCanId) {
     return follow(leaderCanId, false);
   }
 
   /**
    * Causes this controller's output to mirror the provided leader.
    *
    * <p>Only voltage output is mirrored. Settings changed on the leader do not affect the follower.
    *
    * <p>Following anything other than a CAN-enabled SPARK is not officially supported.
    *
    * @param leaderCanId The CAN ID of the device to follow.
    * @param invert Set the follower to output opposite of the leader
    * @return The modified {@link NeoSparkBaseConfig} object for method chaining
    */
   public NeoSparkBaseConfig follow(int leaderCanId, boolean invert) {
     putParameter(SparkParameter.kFollowerModeLeaderId.value, leaderCanId);
     putParameter(SparkParameter.kFollowerModeIsInverted.value, invert);
     return this;
   }
 
   /**
    * Disables follower mode on the controller.
    *
    * @return The modified {@link NeoSparkBaseConfig} object for method chaining
    */
   public NeoSparkBaseConfig disableFollowerMode() {
     putParameter(SparkParameter.kFollowerModeLeaderId.value, 0);
     putParameter(SparkParameter.kFollowerModeIsInverted.value, false);
     return this;
   }
 
   // TODO(jan): External leaders
 
   @Override
   public String flatten() {
     String flattenedString = "";
 
     flattenedString += super.flatten();
     flattenedString += absoluteEncoder.flatten();
     flattenedString += analogSensor.flatten();
     flattenedString += encoder.flatten();
     flattenedString += limitSwitch.flatten();
     flattenedString += softLimit.flatten();
     flattenedString += closedLoop.flatten();
     flattenedString += signals.flatten();
 
     return flattenedString;
   }

   public IdleMode getIdleMode() {
    if ((Integer) getParameter(SparkParameter.kIdleMode.value) == null){
      return IdleMode.kCoast;
    }
    return IdleMode.fromId((Integer) getParameter(SparkParameter.kIdleMode.value));
   }

   public double getSmartCurrentLimit() {
    if (getParameter(SparkParameter.kSmartCurrentStallLimit.value) == null){
      return -999; //Some insane value
    }
    return ((Integer) getParameter(SparkParameter.kSmartCurrentStallLimit.value)).doubleValue();
   }

   public NeoClosedLoopConfig getClosedLoopConfig() {
    return closedLoop;
   }
   

   
 }