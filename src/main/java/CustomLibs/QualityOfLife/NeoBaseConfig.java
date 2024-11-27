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
 import java.util.HashMap;
 import java.util.Map;
 
 public abstract class NeoBaseConfig {
   private Map<Integer, Object> parameters = new HashMap<>();
 
   // package-private
   void putParameter(int parameterId, Object value) {
     parameters.put(parameterId, value);
   }
 
   // package-private
   Object getParameter(int parameterId) {
     return parameters.get(parameterId);
   }
 
   // package-private
   void removeParameter(int parameterId) {
     parameters.remove(parameterId);
   }
 
   // package-private
   void apply(NeoBaseConfig config) {
     for (Map.Entry<Integer, Object> parameter : config.parameters.entrySet()) {
       putParameter(parameter.getKey(), parameter.getValue());
     }
   }
 
   public String flatten() {
     String flattenedString = "";
 
     for (Map.Entry<Integer, Object> parameter : parameters.entrySet()) {
       switch (CANSparkJNI.c_Spark_GetParameterType(parameter.getKey())) {
         case 1:
           flattenedString +=
               CANSparkJNI.c_Spark_FlattenParameterInt32(
                   parameter.getKey(), (int) parameter.getValue());
           break;
         case 2:
           flattenedString +=
               CANSparkJNI.c_Spark_FlattenParameterUint32(
                   parameter.getKey(), (int) parameter.getValue());
           break;
         case 3:
           flattenedString +=
               CANSparkJNI.c_Spark_FlattenParameterFloat(
                   parameter.getKey(), (float) parameter.getValue());
           break;
         case 4:
           flattenedString +=
               CANSparkJNI.c_Spark_FlattenParameterBool(
                   parameter.getKey(), (boolean) parameter.getValue());
           break;
       }
     }
 
     return flattenedString;
   }
 }
 