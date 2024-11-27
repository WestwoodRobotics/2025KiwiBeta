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

import com.revrobotics.spark.config.AlternateEncoderConfig;

public class NeoSparkMaxConfig extends NeoSparkBaseConfig {
   public final NeoAlternateEncoderConfig alternateEncoder = new NeoAlternateEncoderConfig();
 
   // package-private
   enum DataPortConfig {
     kInvalid(-1),
     kLimitSwitchesAndAbsoluteEncoder(0),
     kAlternateEncoder(1);
 
     public final int value;
 
     DataPortConfig(int value) {
       this.value = value;
     }
   }
 
   private void sanitize() {
     // Check if data port configuration is invalid
     if (alternateEncoder.getParameter(SparkParameter.kDataPortConfig.value) != null
         && (absoluteEncoder.getParameter(SparkParameter.kDataPortConfig.value) != null
             || limitSwitch.getParameter(SparkParameter.kDataPortConfig.value) != null)) {
       alternateEncoder.removeParameter(SparkParameter.kDataPortConfig.value);
       absoluteEncoder.removeParameter(SparkParameter.kDataPortConfig.value);
       limitSwitch.removeParameter(SparkParameter.kDataPortConfig.value);
 
       // Driver will handle this accordingly
       this.putParameter(SparkParameter.kDataPortConfig.value, DataPortConfig.kInvalid.value);
     }
   }
 
   /**
    * Applies settings from another {@link NeoSparkMaxConfig} to this one, including all of its nested
    * configurations.
    *
    * <p>Settings in the provided config will overwrite existing values in this object. Settings not
    * specified in the provided config remain unchanged.
    *
    * @param config The {@link NeoSparkMaxConfig} to copy settings from
    * @return The updated {@link NeoSparkMaxConfig} for method chaining
    */
   public NeoSparkMaxConfig apply(NeoSparkMaxConfig config) {
     super.apply(config);
     this.alternateEncoder.apply(config.alternateEncoder);
     return this;
   }
 
   /**
    * Applies settings from an {@link AlternateEncoderConfig} to this {@link NeoSparkMaxConfig}.
    *
    * <p>Settings in the provided config will overwrite existing values in this object. Settings not
    * specified in the provided config remain unchanged.
    *
    * @param config The {@link AlternateEncoderConfig} to copy settings from
    * @return The updated {@link NeoSparkMaxConfig} for method chaining
    */
   public NeoSparkMaxConfig apply(NeoAlternateEncoderConfig config) {
     this.alternateEncoder.apply(config);
     return this;
   }
 
   @Override
   public String flatten() {
     sanitize();
 
     String flattenedString = "";
 
     flattenedString += super.flatten();
     flattenedString += alternateEncoder.flatten();
 
     return flattenedString;
   }
 }
 