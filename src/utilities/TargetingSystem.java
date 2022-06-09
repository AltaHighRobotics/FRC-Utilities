package utilities;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A class for creating and tracking a target in 3D space. Also allows for
 * predicting where the target will be in the future.
 *
 * @author Icarus Innovated
 */
public class TargetingSystem
{
	/**
	 * The callibration data for the system. The key is the distance,
	 * the vector is the time, velocity, and angle of the shot.
	 */
	private final TargetingCalibration CALIBRATION_DATA;
	
    /**
	 * The interpolated data point for the current distance.
	 */
    private CartesianVector lerpedData;

	/**
	 * The current measured location of the target in 3D space, relative to the
	 * robot.
	 */
	public CartesianVector position = new CartesianVector(0, 0, 0);

	/**
	 * The previous average location of the target.
	 */
	private CartesianVector previousPosition = new CartesianVector(0, 0, 0);

	/**
	 * All previous locations of the target, up to the value of max memory.
	 */
	private ArrayList<CartesianVector> positions = new ArrayList<CartesianVector>();

	/**
	 * The average of the previous positions.
	 */
	private CartesianVector averagePosition = new CartesianVector(0, 0, 0);

	/**
	 * The position the target should be in, computed in place of a real position if
	 * the target is lost.
	 */
	private CartesianVector predictedPosition = new CartesianVector(0, 0, 0);

	/**
	 * The difference between the average position and the previous position.
	 */
	private CartesianVector velocity = new CartesianVector(0, 0, 0);

	/**
	 * The previous average velocity of the target.
	 */
	private CartesianVector previousVelocity = new CartesianVector(0, 0, 0);

	/**
	 * All previous velocities of the target, up to the value of max memory.
	 */
	private ArrayList<CartesianVector> velocities = new ArrayList<CartesianVector>();

	/**
	 * The average of the previous velocities.
	 */
	private CartesianVector averageVelocity = new CartesianVector(0, 0, 0);

	/**
	 * The predicted velocity, used when the target is not seen.
	 */
	private CartesianVector predictedVelocity = new CartesianVector(0, 0, 0);

	/**
	 * The difference between the average velocity and the previous velocity.
	 */
	private CartesianVector acceleration = new CartesianVector(0, 0, 0);

	/**
	 * All previous accelerations of the target, up to the value of max memory.
	 */
	private ArrayList<CartesianVector> accelerations = new ArrayList<CartesianVector>();

	/**
	 * The average of the previous accelerations.
	 */
	private CartesianVector averageAcceleration = new CartesianVector(0, 0, 0);

	/**
	 * The position the target is predicted to be in by the time the projectile hits
	 * it.
	 */
	public CartesianVector leadPosition = new CartesianVector(0, 0, 0);

	/**
	 * The distance between the camera and the target's predicted position.
	 */
	private double distanceToPredictedTarget;

	/**
	 * The speed the target is moving at, relative to the robot.
	 */
	private double speed;

	/**
	 * The minimum speed that must be reached for target leading to be computed.
	 */
	private double velocityThreshold;

	/**
	 * The time that has passed since the last time the target was seen.
	 */
	private int age = 1;

	/**
	 * The amount of time that must pass without seeing the target for the system to
	 * reset collected values.
	 */
	private int refreshThreshold;

	/**
	 * The maximum amount of previous values to remember.
	 */
	private int maxMemory;

	/**
	 * The maximum amount of time that acceleration should be used in predictions.
	 */
	private int maxAccelerationPrediction;

	/**
	 * True when the target has been seen this frame.
	 */
	private boolean updated;

	/**
	 * The azimuth angle between the robot and the target, if the robot was aligned
	 * with the field.
	 */
	private double absoluteAzimuthToTarget;

	/**
	 * The elevation angle between the robot and the target, taking camera angle
	 * into acount.
	 */
	private double absoluteElevationToTarget;

	/**
	 * Equal to the absolute azimuth, but in radians instead of degrees.
	 */
	private double azimuthToTargetRadians;

	/**
	 * Equal to the absolute elevation, but in radians instead of degrees.
	 */
	private double elevationToTargetRadians;

	/**
	 * The distance between the camera and the target.
	 */
	private double distanceToTarget;

	/**
	 * The measured position of the target, relative to the robot.
	 */
	private CartesianVector relativeTargetPosition;

	/**
	 * The measured position of the target, relative to the field.
	 */
	private CartesianVector absoluteTargetPosition;

	/**
	 * The location of the camera relative to the robot.
	 */
	private final CartesianVector CAMERA_POSITION;

	/**
	 * The angle the camera is mounted at, relative to horizontal.
	 */
	private final double CAMERA_ELEVATION_ANGLE;

	/**
	 * The location of the target (measure this on the actual field for best results).
	 */
	private final CartesianVector TARGET_POSITION;

	/**
	 * Creates a new TargetingSystem object with set parameters
	 * 
	 * @param cameraPosition            The location of the camera relative to the
	 *                                  robot
	 * @param cameraElevationAngle      The angle the camera is mounted at, relative
	 *                                  to horizontal
	 * @param targetPosition            The location of the target, measured on the
	 *                                  physical field
	 * @param calibrationData           The callibration data for the system. The key is the distance,
	 *									the value is a vector representing the time, velocity, and angle of the shot.
	 * @param velocityThreshold         The minimum speed that must be reached for
	 *                                  target leading to be computed
	 * @param refreshThreshold          The amount of time that must pass without
	 *                                  seeing the target for the system to reset
	 *                                  collected values.
	 * @param maxMemory                 The maximum amount of previous values to
	 *                                  remember
	 * @param maxAccelerationPrediction The maximum amount of time that acceleration
	 *                                  should be used in predictions
	 */
	public TargetingSystem(CartesianVector cameraPosition, double cameraElevationAngle, CartesianVector targetPosition,
			TargetingCalibration calibrationData, double velocityThreshold, int refreshThreshold,
			int maxMemory, int maxAccelerationPrediction)
	{
		this.CAMERA_POSITION = cameraPosition;
		this.CAMERA_ELEVATION_ANGLE = cameraElevationAngle;
		this.TARGET_POSITION = targetPosition;
		this.CALIBRATION_DATA = calibrationData;
		this.velocityThreshold = velocityThreshold;
		this.refreshThreshold = refreshThreshold;
		this.maxMemory = maxMemory;
		this.maxAccelerationPrediction = maxAccelerationPrediction;
		this.relativeTargetPosition = new CartesianVector(0, 0, 0);
		this.absoluteTargetPosition = new CartesianVector(0, 0, 0);
	}

	public CartesianVector getOdometryCalibration(CartesianVector robotPosition)
	{
		absoluteTargetPosition = relativeTargetPosition.getAddition(robotPosition);
		return absoluteTargetPosition.getSubtraction(TARGET_POSITION);
	}

	/**
	 * Updates the target position, velocity, and acceleration.
	 * Also updates the predicted location and lead position of the target.
	 * 
	 * @param robotYaw           The yaw of the robot (degrees).
	 * @param targetingAzimuth   The azimuth angle reported by the targeting camera (degrees).
	 * @param targetingElevation The elevation angle reported by the targeting camera (degrees).
	 */
	public CartesianVector updateTarget(double robotYaw, double targetingAzimuth,
			double targetingElevation)
	{
		if (targetingElevation != 0)
		{
			absoluteAzimuthToTarget = targetingAzimuth + robotYaw;
			azimuthToTargetRadians = -Math.toRadians(absoluteAzimuthToTarget);
	
			absoluteElevationToTarget = targetingElevation + CAMERA_ELEVATION_ANGLE;
			elevationToTargetRadians = Math.toRadians(absoluteElevationToTarget);
	
			distanceToTarget = (TARGET_POSITION.z - CAMERA_POSITION.z) / (Math.tan(elevationToTargetRadians));
			relativeTargetPosition.set(Math.cos(azimuthToTargetRadians) * distanceToTarget,
					Math.sin(azimuthToTargetRadians) * distanceToTarget);
			position.copy(relativeTargetPosition);
	
			updated = true;
		}
		if (updated)
		{
			if (age > refreshThreshold)
			{
				positions.clear();
				velocities.clear();
				accelerations.clear();
			}
			positions = manageList(positions, position, maxMemory);
			averagePosition = weightedAvgVectorList(positions);
			predictedPosition.copy(averagePosition);
			velocity = averagePosition.getSubtraction(previousPosition);
			velocity.divide(age);
			velocities = manageList(velocities, velocity, maxMemory);
			averageVelocity = weightedAvgVectorList(velocities);
			predictedVelocity.copy(averageVelocity);
			previousPosition.copy(averagePosition);
			acceleration = averageVelocity.getSubtraction(previousVelocity);
			acceleration.divide(age);
			accelerations = manageList(accelerations, acceleration, maxMemory);
			averageAcceleration = weightedAvgVectorList(accelerations);
			previousVelocity.copy(averageVelocity);
			speed = averageVelocity.magnitude3D();
			if (speed < velocityThreshold)
			{
				averageVelocity.set(0, 0, 0);
				averageAcceleration.set(0, 0, 0);
			}
			age = 0;
		}
		updated = false;
		age = age + 1;
		if (age <= maxAccelerationPrediction)
		{
			predictedVelocity.add(averageAcceleration);
		}
		predictedPosition.add(predictedVelocity);
		distanceToPredictedTarget = predictedPosition.magnitude3D();
		lerpedData = CALIBRATION_DATA.get(distanceToPredictedTarget);
		SmartDashboard.putNumber("Lead Dist", distanceToPredictedTarget);
		SmartDashboard.putNumber("Lead Time", lerpedData.x);
		leadPosition = predictedPosition.getAddition(averageVelocity.getMultiplication(lerpedData.x));
		distanceToPredictedTarget = leadPosition.magnitude3D();
		lerpedData = CALIBRATION_DATA.get(distanceToPredictedTarget);
		CartesianVector newVelocity = averageVelocity.getAddition(averageAcceleration.getMultiplication(Math.min(lerpedData.x,maxAccelerationPrediction)));
		leadPosition = predictedPosition.getAddition(newVelocity.getMultiplication(lerpedData.x));
		return lerpedData;
	}

	/**
	 * Gets the weighted average of a list of vectors
	 * 
	 * @param vectorList an ArrayList of vector objects
	 * @return A vector representing the weighted average value of the vectors in the list.
	 */
	private CartesianVector weightedAvgVectorList(ArrayList<CartesianVector> vectorList)
	{
		if (vectorList.size() >= 1)
		{
			CartesianVector total = vectorList.get(0).clone();
			total.set(0, 0, 0);
			int weight = 1;
			int totalWeight = 0;
			for (CartesianVector i : vectorList)
			{
				CartesianVector weightedVector = i.clone();
				weightedVector.multiply(weight);
				weight ++;
				totalWeight += weight;
				total.add(weightedVector);
			}
			total.divide(totalWeight);
			return total;
		}
		return new CartesianVector(0, 0, 0);
	}

	/**
	 * Adds a vector to a list of vectors, and removes the oldest vector if the list
	 * is above maxSize.
	 * 
	 * @param vectorList The list of vectors to modify.
	 * @param newItem    The new vector to add to the end of the list.
	 * @param maxSize    The maximum length the list is allowed to reach.
	 * @return The modified version of the input list.
	 */
	private ArrayList<CartesianVector> manageList(ArrayList<CartesianVector> vectorList, CartesianVector newItem,
			int maxSize)
	{
		vectorList.add(newItem);
		if (vectorList.size() > maxSize)
		{
			vectorList.remove(0);
		}
		return vectorList;
	}
}