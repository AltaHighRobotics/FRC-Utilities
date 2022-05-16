package utilities;

import java.util.ArrayList;

/**
 * A class for creating and tracking a target in 3D space. Also allows for
 * predicting where the target will be in the future.
 *
 * @author Icarus Innovated
 */
public class TargetingSystem
{
	/**
	 * The current measured location of the target in 3D space, relative to the
	 * robot.
	 */
	private CartesianVector position = new CartesianVector(0, 0, 0);

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
	private double distanceToPredictedGoal;

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
	private double azimuthToGoalRadians;

	/**
	 * Equal to the absolute elevation, but in radians instead of degrees.
	 */
	private double elevationToGoalRadians;

	/**
	 * The distance between the camera and the target.
	 */
	private double distanceToGoal;

	/**
	 * The measured position of the goal, relative to the robot.
	 */
	private CartesianVector relativeGoalPosition;

	/**
	 * The measured position of the goal, relative to the field.
	 */
	private CartesianVector absoluteGoalPosition;

	/**
	 * The location of the camera relative to the robot.
	 */
	private final CartesianVector CAMERA_POSITION;

	/**
	 * The angle the camera is mounted at, relative to horizontal.
	 */
	private final double CAMERA_ELEVATION_ANGLE;

	/**
	 * The location of the goal (measure this on the actual field for best results).
	 */
	private final CartesianVector GOAL_POSITION;

	/**
	 * Creates a new TargetingSystem object with set parameters
	 * 
	 * @param cameraPosition            The location of the camera relative to the
	 *                                  robot
	 * @param cameraElevationAngle      The angle the camera is mounted at, relative
	 *                                  to horizontal
	 * @param goalPosition              The location of the goal, measured on the
	 *                                  physical field
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
	public TargetingSystem(CartesianVector cameraPosition, double cameraElevationAngle, CartesianVector goalPosition,
			double velocityThreshold, int refreshThreshold, int maxMemory, int maxAccelerationPrediction)
	{
		this.CAMERA_POSITION = cameraPosition;
		this.CAMERA_ELEVATION_ANGLE = cameraElevationAngle;
		this.GOAL_POSITION = goalPosition;
		this.velocityThreshold = velocityThreshold;
		this.refreshThreshold = refreshThreshold;
		this.maxMemory = maxMemory;
		this.maxAccelerationPrediction = maxAccelerationPrediction;
		this.relativeGoalPosition = new CartesianVector(0, 0, 0);
		this.absoluteGoalPosition = new CartesianVector(0, 0, 0);
	}

	public void setNewTargetPosition(CartesianVector newPosition)
	{
		position.copy(newPosition);
		updated = true;
	}

	public void setNewTargetPositionFromTargetingCamera(double robotYaw, double targetingAzimuth,
			double targetingElevation)
	{
		absoluteAzimuthToTarget = targetingAzimuth + robotYaw;
		azimuthToGoalRadians = Math.toRadians(absoluteAzimuthToTarget);

		absoluteElevationToTarget = targetingElevation + CAMERA_ELEVATION_ANGLE;
		elevationToGoalRadians = Math.toRadians(absoluteElevationToTarget);

		distanceToGoal = (GOAL_POSITION.z - CAMERA_POSITION.z) / (Math.tan(elevationToGoalRadians));
		relativeGoalPosition.set(Math.cos(azimuthToGoalRadians) * distanceToGoal,
				Math.sin(azimuthToGoalRadians) * distanceToGoal);
		position.copy(relativeGoalPosition);

		updated = true;
	}

	public CartesianVector getOdometryCalibration(CartesianVector robotPosition)
	{
		absoluteGoalPosition = relativeGoalPosition.getAddition(robotPosition);
		return absoluteGoalPosition.getSubtraction(GOAL_POSITION);
	}

	/**
	 * Updates the target position, and computes the velocity and acceleration of
	 * the target.
	 * 
	 */
	public void updateTarget()
	{
		if (updated)
		{
			if (age > refreshThreshold)
			{
				positions.clear();
				velocities.clear();
				accelerations.clear();
			}
			positions = manageList(positions, position, maxMemory);
			averagePosition = avgVectorList(positions);
			predictedPosition.copy(averagePosition);
			velocity = averagePosition.getSubtraction(previousPosition);
			velocity.divide(age);
			velocities = manageList(velocities, velocity, maxMemory);
			averageVelocity = avgVectorList(velocities);
			previousPosition.copy(averagePosition);
			acceleration = averageVelocity.getSubtraction(previousVelocity);
			acceleration.divide(age);
			accelerations = manageList(accelerations, acceleration, maxMemory);
			averageAcceleration = avgVectorList(accelerations);
			previousVelocity.copy(averageVelocity);
			speed = averageVelocity.magnitude();
			if (speed < velocityThreshold)
			{
				averageVelocity.set(0, 0, 0);
				averageAcceleration.set(0, 0, 0);
			}
			age = 0;
		}
		updated = false;
		age = age + 1;
		predictedPosition.add(averageVelocity
				.getAddition(averageAcceleration.getMultiplication(Math.min(age, maxAccelerationPrediction))));
	}

	public void updateTargetLeadPosition(double timeToTarget)
	{
		leadPosition = predictedPosition.getAddition(averageVelocity.getMultiplication(timeToTarget));
	}

	/**
	 * Calculates the distance between the input location and the target location.
	 * 
	 * @param robotLocation The location of the robot on the field, relative to the
	 *                      target.
	 * @return The distance between the robot and the target, in the form of a 3D
	 *         vector.
	 */
	public double getTargetDistance(CartesianVector robotLocation)
	{
		distanceToPredictedGoal = predictedPosition.getSubtraction(robotLocation).magnitude();
		return distanceToPredictedGoal;
	}

	/**
	 * Gets the average of a list of vectors
	 * 
	 * @param vectorList an ArrayList of vector objects
	 * @return A vector representing the average value of the vectors in the list.
	 */
	private CartesianVector avgVectorList(ArrayList<CartesianVector> vectorList)
	{
		if (vectorList.size() >= 1)
		{
			CartesianVector total = vectorList.get(0).clone();
			total.set(0, 0, 0);
			for (CartesianVector i : vectorList)
			{
				total.add(i);
			}
			total.divide(vectorList.size());
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