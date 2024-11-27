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
import com.revrobotics.spark.SparkMax;
import CustomLibs.QualityOfLife.NeoSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import java.nio.ByteBuffer;
import java.util.concurrent.atomic.AtomicBoolean;

public abstract class NeoSparkLowLevel implements MotorController, AutoCloseable {
  // TODO(Noah): Deprecate and introduce correctly named constants in a more appropriate spot
  public static final int kAPIMajorVersion = CANSparkJNI.c_Spark_GetAPIMajorRevision();
  public static final int kAPIMinorVersion = CANSparkJNI.c_Spark_GetAPIMinorRevision();
  public static final int kAPIBuildVersion = CANSparkJNI.c_Spark_GetAPIBuildRevision();
  public static final int kAPIVersion = CANSparkJNI.c_Spark_GetAPIVersion();

  public enum MotorType {
    kBrushed(0),
    kBrushless(1);

    @SuppressWarnings("MemberName")
    public final int value;

    MotorType(int value) {
      this.value = value;
    }

    public static MotorType fromId(int id) {
      for (MotorType type : values()) {
        if (type.value == id) {
          return type;
        }
      }
      return null;
    }
  }

  public enum PeriodicFrame {
    kStatus0(0),
    kStatus1(1),
    kStatus2(2),
    kStatus3(3),
    kStatus4(4),
    kStatus5(5),
    kStatus6(6),
    kStatus7(7);

    @SuppressWarnings("MemberName")
    public final int value;

    PeriodicFrame(int value) {
      this.value = value;
    }

    public static PeriodicFrame fromId(int id) {
      for (PeriodicFrame type : values()) {
        if (type.value == id) {
          return type;
        }
      }
      return null;
    }
  }

  public class PeriodicStatus0 {
    public double appliedOutput;
    public short faults;
    public short stickyFaults;
    public byte lock;
    public MotorType motorType;
    public boolean isFollower;
    public boolean isInverted;
    public boolean roboRIO;
  }

  public class PeriodicStatus1 {
    public double sensorVelocity;
    public byte motorTemperature;
    public double busVoltage;
    public double outputCurrent;
  }

  public class PeriodicStatus2 {
    public double sensorPosition;
    public double iAccum;
  }

  protected enum SparkModel {
    SparkMax(0),
    SparkFlex(1),
    Unknown(255);

    final int id;

    SparkModel(int id) {
      this.id = id;
    }

    static SparkModel fromId(int id) {
      for (SparkModel model : values()) {
        if (model.id == id) return model;
      }
      return Unknown;
    }
  }

  protected final long sparkHandle;
  protected final AtomicBoolean isClosed = new AtomicBoolean(false);
  private final int deviceId;
  private String firmwareString;
  protected final MotorType motorType;
  protected final SparkModel expectedSparkModel;

  /**
   * Create a new SPARK Controller
   *
   * <p>Package-private, so that only other classes in our package can create
   *
   * @param deviceId The device ID
   * @param type The motor type connected to the controller. Brushless motors must be connected to
   *     their matching color and the hall sensor plugged in. Brushed motors must be connected to
   *     the Red and Black terminals only.
   */
  NeoSparkLowLevel(int deviceId, MotorType type, SparkModel model) {
    if (type == null) {
      throw new IllegalArgumentException("type must not be null");
    }
    this.deviceId = deviceId;
    firmwareString = "";
    motorType = type;
    expectedSparkModel = model;
    if (CANSparkJNI.c_Spark_RegisterId(deviceId) == REVLibError.kDuplicateCANId.value) {
      throw new IllegalStateException(
          "A CANSparkMax instance has already been created with this device ID: " + deviceId);
    }
    sparkHandle = CANSparkJNI.c_Spark_Create(deviceId, type.value, model.id);
  }

  /** Closes the SPARK Controller */
  @Override
  public void close() {
    boolean alreadyClosed = isClosed.getAndSet(true);
    if (alreadyClosed) {
      return;
    }
    CANSparkJNI.c_Spark_Destroy(sparkHandle);
  }

  /**
   * Get the firmware version of the SPARK.
   *
   * @return uint32_t Firmware version integer. Value is represented as 4 bytes, Major.Minor.Build
   *     H.Build L
   */
  // TODO(Noah): Add a function that returns the different version fields as an object
  public int getFirmwareVersion() {
    throwIfClosed();
    return CANSparkJNI.c_Spark_GetFirmwareVersion(sparkHandle);
  }

  /**
   * Set the control frame send period for the native CAN Send thread.
   *
   * @param periodMs The send period in milliseconds between 1ms and 100ms or set to 0 to disable
   *     periodic sends. Note this is not updated until the next call to Set() or SetReference().
   */
  public void setControlFramePeriodMs(int periodMs) {
    throwIfClosed();
    CANSparkJNI.c_Spark_SetControlFramePeriod(sparkHandle, periodMs);
  }

  /**
   * Get the firmware version of the SPARK as a string.
   *
   * @return std::string Human readable firmware version string
   */
  public String getFirmwareString() {
    throwIfClosed();
    if (firmwareString == "") {
      int version = getFirmwareVersion();
      ByteBuffer b = ByteBuffer.allocate(4);
      b.putInt(version);

      byte[] verBytes = b.array();

      StringBuilder firmwareString = new StringBuilder();
      firmwareString
          .append("v")
          .append((int) verBytes[0])
          .append(".")
          .append((int) verBytes[1])
          .append(".")
          .append((int) verBytes[2] << 8 | (int) verBytes[3]);

      this.firmwareString = firmwareString.toString();
    }
    return firmwareString;
  }

  /**
   * Get the unique serial number of the SPARK. Not currently available.
   *
   * @return byte[] Vector of bytes representig the unique serial number
   */
  public byte[] getSerialNumber() {
    throwIfClosed();
    return new byte[0];
  }

  /**
   * Get the configured Device ID of the SPARK.
   *
   * @return int device ID
   */
  public int getDeviceId() {
    throwIfClosed();
    return deviceId;
  }

  /**
   * Get the motor type setting for the SPARK
   *
   * @return MotorType Motor type setting
   */
  public MotorType getMotorType() {
    throwIfClosed();
    return motorType;
  }

  /**
   * Set the amount of time to wait for a periodic status frame before returning a timeout error.
   * This timeout will apply to all periodic status frames for the SPARK motor controller.
   *
   * <p>To prevent invalid timeout errors, the minimum timeout for a given periodic status is 2.1
   * times its period. To use the minimum timeout for all status frames, set timeoutMs to 0.
   *
   * <p>The default timeout is 500ms.
   *
   * @param timeoutMs The timeout in milliseconds
   */
  public void setPeriodicFrameTimeout(int timeoutMs) {
    throwIfClosed();
    CANSparkJNI.c_Spark_SetPeriodicFrameTimeout(sparkHandle, timeoutMs);
  }

  /**
   * Set the maximum number of times to retry an RTR CAN frame. This applies to calls such as
   * SetParameter* and GetParameter* where a request is made to the SPARK motor controller and a
   * response is expected. Anytime sending the request or receiving the response fails, it will
   * retry the request a number of times, no more than the value set by this method. If an attempt
   * succeeds, it will immediately return. The minimum number of retries is 0, where only a single
   * attempt will be made and will return regardless of success or failure.
   *
   * <p>If the CAN timeout is set to 0 with {@link SparkBase#setCANTimeout(int)}, this will have no
   * effect when getting the response from the SPARK motor controller fails. However, it will retry
   * when sending the request from the roboRIO fails.
   *
   * <p>The default maximum is 5 retries.
   *
   * @param numRetries The maximum number of retries
   */
  public void setCANMaxRetries(int numRetries) {
    throwIfClosed();
    CANSparkJNI.c_Spark_SetCANMaxRetries(sparkHandle, numRetries);
  }

  REVLibError setpointCommand(double value) {
    throwIfClosed();
    return setpointCommand(value, SparkMax.ControlType.kDutyCycle);
  }

  REVLibError setpointCommand(double value, SparkMax.ControlType ctrl) {
    throwIfClosed();
    return setpointCommand(value, ctrl, 0);
  }

  REVLibError setpointCommand(double value, SparkMax.ControlType ctrl, int pidSlot) {
    throwIfClosed();
    return setpointCommand(value, ctrl, pidSlot, 0);
  }

  REVLibError setpointCommand(
      double value, SparkMax.ControlType ctrl, int pidSlot, double arbFeedforward) {
    throwIfClosed();
    return setpointCommand(value, ctrl, pidSlot, arbFeedforward, 0);
  }

  // TODO(Noah): Deprecate all setpointCommand() methods in favor of the corresponding methods in
  // SparkClosedLoopController.
  //             Next year, we can set them all to be protected.
  REVLibError setpointCommand(
      double value, SparkMax.ControlType ctrl, int pidSlot, double arbFeedforward, int arbFFUnits) {
    throwIfClosed();
    return REVLibError.fromInt(
        CANSparkJNI.c_Spark_SetpointCommand(
            sparkHandle, (float) value, ctrl.value, pidSlot, (float) arbFeedforward, arbFFUnits));
  }

  public float getSafeFloat(float f) {
    throwIfClosed();
    if (Float.isNaN(f) || Float.isInfinite(f)) return 0;

    return f;
  }

  /** Create the sim gui Fault Manager for this Spark Device */
  public void createSimFaultManager() {
    CANSparkJNI.c_Spark_CreateSimFaultManager(sparkHandle);
  }

  protected void throwIfClosed() {
    if (isClosed.get()) {
      throw new IllegalStateException("This SPARK object has previously been closed.");
    }
  }
}
