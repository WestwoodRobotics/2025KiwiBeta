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
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
 import com.revrobotics.spark.config.SparkFlexConfigAccessor;

import CustomLibs.QualityOfLife.NeoSparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.utils.SparkModels;
 
 public class NeoSparkFlex extends NeoSparkBase {
   private NeoSparkFlexExternalEncoder extEncoder;
   private final Object extEncoderLock = new Object();
   protected long sparkHandle;
   private NeoSparkBaseConfig current_config;
 
   /**
    * Accessor for SPARK parameter values. This object contains fields and methods to retrieve
    * parameters that have been applied to the device. To set parameters, see {@link SparkBaseConfig}
    * and {@link SparkBase#configure(SparkBaseConfig, SparkBase.ResetMode, SparkBase.PersistMode)}.
    *
    * <p>NOTE: This uses calls that are blocking to retrieve parameters and should be used
    * infrequently.
    */
   public final SparkFlexConfigAccessor configAccessor;
 
   /**
    * Create a new object to control a SPARK Flex motor Controller
    *
    * @param deviceId The device ID.
    * @param type The motor type connected to the controller. Brushless motor wires must be connected
    *     to their matching colors and the hall sensor must be plugged in. Brushed motors must be
    *     connected to the Red and Black terminals only.
    */
   public NeoSparkFlex(int deviceId, MotorType type) {
     super(deviceId, type, SparkModels.SparkFlex);
     current_config = new NeoSparkFlexConfig();
     configAccessor = new SparkFlexConfigAccessor(sparkHandle);
 
     if (CANSparkJNI.c_Spark_GetSparkModel(sparkHandle) != SparkModel.SparkFlex.id) {
       DriverStation.reportWarning(
           "CANSparkFlex object created for CAN ID "
               + deviceId
               + ", which is not a SPARK Flex. Some functionalities may not work.",
           true);
     }
   }

   public REVLibError configure(
     NeoSparkBaseConfig config) {
     current_config = config;
     REVLibError status = super.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
     
     return status;
   }
 
   /** ***** Extended Functions ****** */
   /**
    * Returns an object for interfacing with an external quadrature encoder
    *
    * @return An object for interfacing with an external quadrature encoder
    */
   public RelativeEncoder getExternalEncoder() {
     throwIfClosed();
     synchronized (extEncoderLock) {
       if (extEncoder == null) {
         extEncoder = new NeoSparkFlexExternalEncoder(this);
       }
       return extEncoder;
     }
   }



   public void setPIDF(double kP, double kI, double kD, double kF) {
    if(getClosedLoopController().getP() != kP) {
      current_config.closedLoop.p(kP);
    }
    if(getClosedLoopController().getI() != kI) {
      current_config.closedLoop.i(kI);
      
    }
    if(getClosedLoopController().getD() != kD) {
      current_config.closedLoop.d(kD);
      
    }
    if(getClosedLoopController().getF() != kF) {
      current_config.closedLoop.f(kF);
      
    }
    configure(current_config);
  }

  public void setOutputRange(double min, double max) {
    if(getClosedLoopController().getOutputRangeMin() != min) {
      current_config.closedLoop.minOutput(min);
    }
    if(getClosedLoopController().getOutputRangeMax() != max) {
      current_config.closedLoop.maxOutput(max);
    }
    configure(current_config);
  }

  public void setIdleMode(IdleMode mode) {
    if(getIdleMode() != mode) {
      current_config.idleMode(mode);
    }
    configure(current_config);
  }

  public IdleMode getIdleMode() {
    return current_config.getIdleMode();
  }


  public void setSmartCurrentLimit(int limit) {
    if(getSmartCurrentLimit() != limit) {
      current_config.smartCurrentLimit(limit);
    }
    configure(current_config);
  }

  public double getSmartCurrentLimit() {
    return current_config.getSmartCurrentLimit();
  }

  public void setPositionConversionFactor(double factor) {
    if(getPositionConversionFactor() != factor) {
      current_config.encoder.positionConversionFactor(factor);
    }
    configure(current_config);
  }

  public double getPositionConversionFactor() {
    return current_config.encoder.getPositionConversionFactor();
  }

  public void setVelocityConversionFactor(double factor) {
    if(getVelocityConversionFactor() != factor) {
      current_config.encoder.velocityConversionFactor(factor);
    }
    configure(current_config);
  }

  public double getVelocityConversionFactor() {
    return current_config.encoder.getVelocityConversionFactor();
  }

  public void setPositionWrappingEnabled(boolean enabled) {
    if(isPositionWrappingEnabled() != enabled) {
      current_config.closedLoop.positionWrappingEnabled(enabled);
    }
    configure(current_config);
  }

  public boolean isPositionWrappingEnabled() {
    return current_config.closedLoop.isPositionWrappingEnabled();
  }


  public void setPositionWrappingInputRange(double min, double max) {
    if(getPositionWrappingInputRangeMin() != min) {
      current_config.closedLoop.positionWrappingInputRange(min, getPositionWrappingInputRangeMax());
    }
    if(getPositionWrappingInputRangeMax() != max) {
      current_config.closedLoop.positionWrappingInputRange(getPositionWrappingInputRangeMin(), max);
    }
    configure(current_config);
  }

  public void setPositionInputRangeMin(double min) {
    if(getPositionWrappingInputRangeMin() != min) {
      current_config.closedLoop.positionWrappingInputRange(min, getPositionWrappingInputRangeMax());
    }
    configure(current_config);
  }

  public void setPositionInputRangeMax(double max) {
    if(getPositionWrappingInputRangeMax() != max) {
      current_config.closedLoop.positionWrappingInputRange(getPositionWrappingInputRangeMin(), max);
    }
    configure(current_config);
  }

  public double getPositionWrappingInputRangeMin() {
    return current_config.closedLoop.getPositionWrappingInputRangeMin();
  }

  public double getPositionWrappingInputRangeMax() {
    return current_config.closedLoop.getPositionWrappingInputRangeMax();
  }

  public NeoSparkBaseConfig getCurrentConfig() {
    return current_config;
  }
 }
 