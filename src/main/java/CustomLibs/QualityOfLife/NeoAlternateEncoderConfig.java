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
 import CustomLibs.QualityOfLife.NeoSparkMaxConfig.DataPortConfig;
 
 public class NeoAlternateEncoderConfig extends NeoBaseConfig {
   public enum Type {
     kQuadrature(0);
 
     @SuppressWarnings("MemberName")
     public final int value;
 
     Type(int value) {
       this.value = value;
     }
 
     public static Type fromId(int id) {
       return kQuadrature;
     }
   }
 
   /**
    * Applies settings from another {@link NeoAlternateEncoderConfig} to this one.
    *
    * <p>Settings in the provided config will overwrite existing values in this object. Settings not
    * specified in the provided config remain unchanged.
    *
    * @param config The {@link NeoAlternateEncoderConfig} to copy settings from
    * @return The updated {@link NeoAlternateEncoderConfig} for method chaining
    */
   public NeoAlternateEncoderConfig apply(NeoAlternateEncoderConfig config) {
     super.apply(config);
     return this;
   }
 
   /**
    * Configures the data port to use the alternate encoder, which is specifically required for SPARK
    * MAX.
    *
    * <p>NOTE: This method is only necessary when using an alternate encoder with a SPARK MAX without
    * configuring any of its settings
    *
    * <p>IMPORTANT: SPARK MAX does not support using an alternate encoder in conjunction with an
    * absolute encoder and/or limit switches.
    *
    * @return The modified {@link NeoAlternateEncoderConfig} object for method chaining
    */
   public NeoAlternateEncoderConfig setSparkMaxDataPortConfig() {
     putParameter(SparkParameter.kDataPortConfig.value, DataPortConfig.kAlternateEncoder.value);
     return this;
   }
 
   /**
    * Set the counts per revolutions of the alternate encoder.
    *
    * @param cpr The counts per rotation
    * @return The modified {@link NeoAlternateEncoderConfig} object for method chaining
    */
   public NeoAlternateEncoderConfig countsPerRevolution(int cpr) {
     setSparkMaxDataPortConfig();
     putParameter(SparkParameter.kAltEncoderCountsPerRev.value, cpr);
     return this;
   }
 
   /**
    * Set the phase of the alternate encoder so that it is in phase with the motor itself.
    *
    * @param inverted The phase of the encoder
    * @return The modified {@link NeoAlternateEncoderConfig} object for method chaining
    */
   public NeoAlternateEncoderConfig inverted(boolean inverted) {
     setSparkMaxDataPortConfig();
     putParameter(SparkParameter.kAltEncoderInverted.value, inverted);
     return this;
   }
 
   /**
    * Set the conversion factor for the position of the alternate encoder. Position is returned in
    * native units of rotations and will be multiplied by this conversion factor.
    *
    * @param factor The conversion factor to multiply the native units by
    * @return The modified {@link NeoAlternateEncoderConfig} object for method chaining
    */
   public NeoAlternateEncoderConfig positionConversionFactor(double factor) {
     setSparkMaxDataPortConfig();
     putParameter(SparkParameter.kAltEncodePositionFactor.value, (float) factor);
     return this;
   }
 
   /**
    * Set the conversion factor for the velocity of the alternate encoder. Velocity is returned in
    * native units of rotations per minute and will be multiplied by this conversion factor.
    *
    * @param factor The conversion factor to multiply the native units by
    * @return The modified {@link NeoAlternateEncoderConfig} object for method chaining
    */
   public NeoAlternateEncoderConfig velocityConversionFactor(double factor) {
     setSparkMaxDataPortConfig();
     putParameter(SparkParameter.kAltEncoderVelocityFactor.value, (float) factor);
     return this;
   }
 
   /**
    * Set the sampling depth of the velocity calculation process of the alternate encoder. This value
    * sets the number of samples in the average for velocity readings. For a quadrature encoder, this
    * can be any value from 1 to 64 (default).
    *
    * @param depth The velocity calculation process's sampling depth
    * @return The modified {@link NeoAlternateEncoderConfig} object for method chaining
    */
   public NeoAlternateEncoderConfig averageDepth(int depth) {
     setSparkMaxDataPortConfig();
     putParameter(SparkParameter.kAltEncoderAverageDepth.value, depth);
     return this;
   }
 
   /**
    * Set the position measurement period used to calculate the velocity of the alternate encoder.
    * For a quadrature encoder, this number may be between 1 and 100 (default).
    *
    * <p>The basic formula to calculate velocity is change in position / change in time. This
    * parameter sets the change in time for measurement.
    *
    * @param periodMs Measurement period in milliseconds
    * @return The modified {@link NeoAlternateEncoderConfig} object for method chaining
    */
   public NeoAlternateEncoderConfig measurementPeriod(int periodMs) {
     setSparkMaxDataPortConfig();
     putParameter(SparkParameter.kAltEncoderSampleDelta.value, periodMs);
     return this;
   }
 }
 