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

import com.revrobotics.jni.CANSparkJNI;

public class NeoEncoderConfig extends NeoBaseConfig {
   /**
    * Applies settings from another {@link NeoEncoderConfig} to this one.
    *
    * <p>Settings in the provided config will overwrite existing values in this object. Settings not
    * specified in the provided config remain unchanged.
    *
    * @param config The {@link NeoEncoderConfig} to copy settings from
    * @return The updated {@link NeoEncoderConfig} for method chaining
    */
   public NeoEncoderConfig apply(NeoEncoderConfig config) {
     super.apply(config);
     return this;
   }
 
   /**
    * Set the counts per revolutions of the encoder.
    *
    * <p>NOTE: This only applies to an encoder used in brushed mode.
    *
    * @param cpr The counts per rotation
    * @return The modified {@link NeoEncoderConfig} object for method chaining
    */
   public NeoEncoderConfig countsPerRevolution(int cpr) {
     putParameter(SparkParameter.kEncoderCountsPerRev.value, cpr);
     return this;
   }
 
   /**
    * Set the phase of the encoder so that it is in phase with the motor itself.
    *
    * <p>NOTE: This only applies to an encoder used in brushed mode.
    *
    * @param inverted The phase of the encoder
    * @return The modified {@link NeoEncoderConfig} object for method chaining
    */
   public NeoEncoderConfig inverted(boolean inverted) {
     putParameter(SparkParameter.kEncoderInverted.value, inverted);
     return this;
   }
 
   /**
    * Set the conversion factor for the position of the encoder. Position is returned in native units
    * of rotations and will be multiplied by this conversion factor.
    *
    * @param factor The conversion factor to multiply the native units by
    * @return The modified {@link NeoEncoderConfig} object for method chaining
    */
   public NeoEncoderConfig positionConversionFactor(double factor) {
     putParameter(SparkParameter.kPositionConversionFactor.value, (float) factor);
     return this;
   }
 
   /**
    * Set the conversion factor for the velocity of the encoder. Velocity is returned in native units
    * of rotations per minute and will be multiplied by this conversion factor.
    *
    * @param factor The conversion factor to multiply the native units by
    * @return The modified {@link NeoEncoderConfig} object for method chaining
    */
   public NeoEncoderConfig velocityConversionFactor(double factor) {
     putParameter(SparkParameter.kVelocityConversionFactor.value, (float) factor);
     return this;
   }
 
   /**
    * Set the sampling depth of the velocity calculation process of the encoder. This value sets the
    * number of samples in the average for velocity readings. This value must be in the range [1,
    * 64]. The default value is 64.
    *
    * @param depth The velocity calculation process's sampling depth
    * @return The modified {@link NeoEncoderConfig} object for method chaining
    */
   public NeoEncoderConfig quadratureAverageDepth(int depth) {
     putParameter(SparkParameter.kEncoderAverageDepth.value, depth);
     return this;
   }
 
   /**
    * Set the position measurement period used to calculate the velocity of the encoder. This value
    * is in units of milliseconds and must be in a range [1, 100]. The default value is 100ms
    *
    * <p>The basic formula to calculate velocity is change in position / change in time. This
    * parameter sets the change in time for measurement.
    *
    * @param periodMs Measurement period in milliseconds
    * @return The modified {@link NeoEncoderConfig} object for method chaining
    */
   public NeoEncoderConfig quadratureMeasurementPeriod(int periodMs) {
     putParameter(SparkParameter.kEncoderSampleDelta.value, periodMs << 1);
     return this;
   }
 
   /**
    * Set the sampling depth of the velocity calculation process of the encoder. This value sets the
    * number of samples in the average for velocity readings. This value must be either 1, 2, 4, or 8
    * (default).
    *
    * @param depth The velocity calculation process's sampling depth
    * @return The modified {@link NeoEncoderConfig} object for method chaining
    */
   public NeoEncoderConfig uvwAverageDepth(int depth) {
     int depthIndex;
     switch (depth) {
       case (1):
         depthIndex = 0;
         break;
       case (2):
         depthIndex = 1;
         break;
       case (4):
         depthIndex = 2;
         break;
       default:
         depthIndex = 3;
     }
     putParameter(SparkParameter.kHallSensorAverageDepth.value, depthIndex);
     return this;
   }
 
   /**
    * Set the position measurement period used to calculate the velocity of the encoder. This value
    * is in units of milliseconds and must be in a range [8, 64]. The default value is 32ms.
    *
    * <p>The basic formula to calculate velocity is change in position / change in time. This
    * parameter sets the change in time for measurement.
    *
    * @param periodMs Measurement period in milliseconds
    * @return The modified {@link NeoEncoderConfig} object for method chaining
    */
   public NeoEncoderConfig uvwMeasurementPeriod(int periodMs) {
     // Convert from period (ms) to native units (seconds)
     double rate = (double) periodMs / 1000;
     putParameter(SparkParameter.kHallSensorSampleRate.value, (float) rate);
     return this;
   }


   public double getPositionConversionFactor(){
    if (getParameter(SparkParameter.kPositionConversionFactor.value) == null){
      return -999; // Some insane value
    }
    return ((Float) getParameter(SparkParameter.kPositionConversionFactor.value)).doubleValue();
   }

   // ...existing code...
   public double getVelocityConversionFactor(){
    if (getParameter(SparkParameter.kVelocityConversionFactor.value) == null){
       return -999; // Some default "invalid" value
    }
    // Convert from Float to double without direct cast to Double
    return ((Float) getParameter(SparkParameter.kVelocityConversionFactor.value)).doubleValue();
  }



 }
 