package frc.lib;

import com.revrobotics.spark.config.SparkBaseConfig;

/** A Utility class for controlling the amount of data sent by a CAN Spark motor controller over the CAN bus.
 */
public class CANSparkUtil {
  public enum Usage {
    kAll,
    kPositionOnly,
    kVelocityOnly,
    kMinimal
  };

  /**
   * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic status
   * frame period of nonessential frames from 20ms to 500ms.
   *
   * <p>See
   * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
   * for a description of the status frames.
   *
   * @param motor The motor to adjust the status frame periods on.
   * @param usage The status frame feedack to enable. kAll is the default when a CANSparkMax is
   *     constructed.
   * @param enableFollowing Whether to enable motor following.
   */
public static void setCANSparkBusUsage(SparkBaseConfig config, Usage usage, boolean enableFollowing) {
  if (enableFollowing) {
    config.signals.appliedOutputPeriodMs(10);
  } else {
    config.signals.appliedOutputPeriodMs(500);
  }

  if (usage == Usage.kAll) {
    //sets ussage to send all the frames of data yay
    config.signals.analogVelocityPeriodMs(20);
    config.signals.analogPositionPeriodMs(20);
    config.signals.analogVoltagePeriodMs(500);
  } else if (usage == Usage.kPositionOnly) {
    //only sends the position frames every 20 ms, saves on velocity and other status
      config.signals.analogVelocityPeriodMs(1000);
    config.signals.analogPositionPeriodMs(20);
    config.signals.analogVoltagePeriodMs(1000);
  } else if (usage == Usage.kVelocityOnly) {
    //only sends the velocity every 20 ms
      config.signals.analogVelocityPeriodMs(20);
    config.signals.analogPositionPeriodMs(1000);
    config.signals.analogVoltagePeriodMs(1000);
  } else if (usage == Usage.kMinimal) {
    //sends as little data as possible to save canbus usage
      config.signals.analogVelocityPeriodMs(500);
    config.signals.analogPositionPeriodMs(500);
    config.signals.analogVoltagePeriodMs(500);

  }
}

  /**
   * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic status
   * frame period of nonessential frames from 20ms to 500ms.
   *
   * <p>See
   * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
   * for a description of the status frames.
   *
   * @param motor The motor to adjust the status frame periods on.
   * @param usage The status frame feedack to enable. kAll is the default when a CANSparkMax is
   *     constructed.
   */
  public static void setCANSparkBusUsage(SparkBaseConfig config, Usage usage) {
    setCANSparkBusUsage(config, usage, false);
  }
}