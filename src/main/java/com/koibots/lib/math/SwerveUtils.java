package com.koibots.lib.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static edu.wpi.first.math.kinematics.SwerveDriveKinematics.desaturateWheelSpeeds;

public class SwerveUtils {

  /**
   * Steps a value towards a target with a specified step size.
   * @param _current The current or starting value.  Can be positive or negative.
   * @param _target The target value the algorithm will step towards.  Can be positive or negative.
   * @param _stepsize The maximum step size that can be taken.
   * @return The new value for {@code _current} after performing the specified step towards the specified target.
   */
  public static double StepTowards(double _current, double _target, double _stepsize) {
    if (Math.abs(_current - _target) <= _stepsize) {
      return _target;
    }
    else if (_target < _current) {
      return _current - _stepsize;
    }
    else {
      return _current + _stepsize;
    }
  }

  /**
   * Steps a value (angle) towards a target (angle) taking the shortest path with a specified step size.
   * @param _current The current or starting angle (in radians).  Can lie outside the 0 to 2*PI range.
   * @param _target The target angle (in radians) the algorithm will step towards.  Can lie outside the 0 to 2*PI range.
   * @param _stepsize The maximum step size that can be taken (in radians).
   * @return The new angle (in radians) for {@code _current} after performing the specified step towards the specified target.
   * This value will always lie in the range 0 to 2*PI (exclusive).
   */
  public static double StepTowardsCircular(double _current, double _target, double _stepsize) {
    _current = wrapAngle(_current);
    _target = wrapAngle(_target);

    double stepDirection = Math.signum(_target - _current);
    double difference = Math.abs(_current - _target);

    if (difference <= _stepsize) {
      return _target;
    }
    else if (difference > Math.PI) { //does the system need to wrap over eventually?
      //handle the special case where you can reach the target in one step while also wrapping
      if (_current + 2*Math.PI - _target < _stepsize || _target + 2*Math.PI - _current < _stepsize) {
        return _target;
      }
      else {
        return wrapAngle(_current - stepDirection * _stepsize); //this will handle wrapping gracefully
      }

    }
    else {
      return _current + stepDirection * _stepsize;
    }
  }

  /**
   * Finds the (unsigned) minimum difference between two angles including calculating across 0.
   * @param _angleA An angle (in radians).
   * @param _angleB An angle (in radians).
   * @return The (unsigned) minimum difference between the two angles (in radians).
   */
  public static double AngleDifference(double _angleA, double _angleB) {
    double difference = Math.abs(_angleA - _angleB);
    return difference > Math.PI? (2 * Math.PI) - difference : difference;
  }

  /**
   * Wraps an angle until it lies within the range from 0 to 2*PI (exclusive).
   * @param _angle The angle (in radians) to wrap.  Can be positive or negative and can lie multiple wraps outside the output range.
   * @return An angle (in radians) from 0 and 2*PI (exclusive).
   */
  public static double wrapAngle(double _angle) {
    double twoPi = 2*Math.PI;

    return _angle % twoPi;
  }

  /**
   * Corrects for swerve skew in first order kinematics described
   * <a href="https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964">here</a>
   * @param speeds The speeds the robot should be set to
   * @param kinematics
   * @param periodSeconds The time in seconds between calls of this function
   * @param maxLinearSpeed The max speed the robot can move in a straight line
   * @return The corrected states to set swerve modules to
   */
  public static SwerveModuleState[] secondOrderKinematics(
          ChassisSpeeds speeds,
          SwerveDriveKinematics kinematics,
          double periodSeconds,
          double maxLinearSpeed,
          SwerveModuleState[] previousStates) {
    var setpointTwist = new Pose2d()
            .log(
                    new Pose2d(
                            speeds.vxMetersPerSecond * periodSeconds,
                            speeds.vyMetersPerSecond * periodSeconds,
                            new Rotation2d(speeds.omegaRadiansPerSecond * periodSeconds)));

    var adjustedSpeeds =
            new edu.wpi.first.math.kinematics.ChassisSpeeds(
                    setpointTwist.dx / periodSeconds,
                    setpointTwist.dy / periodSeconds,
                    setpointTwist.dtheta / periodSeconds);

    var targetModuleStates = kinematics.toSwerveModuleStates(adjustedSpeeds);

    desaturateWheelSpeeds(targetModuleStates, maxLinearSpeed);

    if (adjustedSpeeds.vxMetersPerSecond == 0.0
            && adjustedSpeeds.vyMetersPerSecond == 0.0
            && adjustedSpeeds.omegaRadiansPerSecond == 0) {
      targetModuleStates[0] = new SwerveModuleState(0, new Rotation2d()); // TODO: Replace Rotation2d with actual previous state
      targetModuleStates[1] = new SwerveModuleState(0, new Rotation2d());
      targetModuleStates[2] = new SwerveModuleState(0, new Rotation2d());
      targetModuleStates[3] = new SwerveModuleState(0, new Rotation2d());
    }

    return targetModuleStates;
  }
}