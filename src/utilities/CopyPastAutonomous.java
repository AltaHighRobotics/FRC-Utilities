package utilities;

/**
 * A system for tracking robot position, and navigating to locations on the
 * field.
 * 
 * @author Icarus Innovated
 */
public class CopyPastAutonomous
{
	/**
	 * The computed drive power needed to reach the waypoint.
	 */
	private double drivePower;

	/**
	 * The computed steering power needed to aim at the waypoint.
	 */
	private double steeringPower;

	/**
	 * The direction the robot should attempt to face.
	 */
	private double targetHeading;

	/**
	 * The wrapped target heading, which represents the shortest path to the target.
	 */
	private double headingWrap;

	/**
	 * Controls if the robot drives in reverse or not.
	 */
	private boolean reverse;

	/**
	 * The position and rotation of the robot in 3D space.
	 */
	public CartesianVector position;

	/**
	 * The speed of the robot in 2D space.
	 */
	public CartesianVector velocity;

	/**
	 * The robot's target in 2D space.
	 */
	public CartesianVector target;

	/**
	 * The difference between the robot's target and its position.
	 */
	public CartesianVector positionError;

	/**
	 * The direction the waypoint is in from the current position.
	 */
	public double directionToWaypoint;

	/**
	 * The wrapped direction, which represents the shortest direction to steer.
	 */
	private double directionWrap;

	/**
	 * The speed the robot will attempt to drive at, based on the distance to the
	 * target.
	 */
	public double targetSpeed;

	/**
	 * The current position of the left and right encoders.
	 */
	private CartesianVector currentMotorPositions;

	/**
	 * The previous position of the left and right encoders.
	 */
	private CartesianVector previousMotorPositions;

	/**
	 * The difference between the current and previous encoder positions.
	 */
	public CartesianVector motorVelocities;

	/**
	 * The length of time that the robot has been at the target.
	 */
	public int timeAtWaypoint;

	/**
	 * The ratio between drive encoder units and full rotations of the drive motor.
	 * For example, a falcon 500 integrated encoder has 2048 steps per rotation,
	 * so for drivetrains running falcon 500s, set this to 2048.
	 */
	private final int ENCODER_UNITS_PER_ROTATION;

	/**
	 * The ratio between full rotations of the drive motor and meters travled.
	 */
	private final double METERS_PER_ROTATION;

	/**
	 * The maximum allowed distance between the robot and the target.
	 */
	private final double MAX_WAYPOINT_ERROR;

	/**
	 * A PID controller for computing the steering power.
	 */
	private final ConfigurablePID drivetrainHeadingPID;

	/**
	 * A PID controller for computing the drive power.
	 */
	private final ConfigurablePID drivetrainSpeedPID;

	/**
	 * Creates a new CopyPastAutonomous with the set ratio constants and PID configurations.
	 * 
	 * @param encoderUnitsPerRotation The ratio between drive encoder units and full rotations of the drive motor.
	 * @param metersPerRotation       The ratio between full rotations of the drive motor and meters travled.
	 * @param headingPIDConfig        A PIDConfiguration with settings for the heading controller.
	 * @param speedPIDConfig          A PIDConfiguration with settings for the speed controller.
	 */
	public CopyPastAutonomous(double maxError, int encoderUnitsPerRotation, double metersPerRotation, PIDConfiguration headingPIDConfig, PIDConfiguration speedPIDConfig)
	{
		this.position = new CartesianVector(0, 0, 0, 0, 0, 0);
		this.velocity = new CartesianVector(0, 0);
		this.target = new CartesianVector(0, 0);
		this.positionError = new CartesianVector(0, 0);

		this.currentMotorPositions = new CartesianVector(0, 0);
		this.previousMotorPositions = new CartesianVector(0, 0);
		this.motorVelocities = new CartesianVector(0, 0);

		this.ENCODER_UNITS_PER_ROTATION = encoderUnitsPerRotation;
		this.METERS_PER_ROTATION = metersPerRotation;
		this.MAX_WAYPOINT_ERROR = maxError;

		this.drivetrainHeadingPID = new ConfigurablePID(headingPIDConfig);
		this.drivetrainSpeedPID = new ConfigurablePID(speedPIDConfig);
	}

	/**
	 * Updates where the robot believes itself to be. This function should be called
	 * before any others, and also resets power levels as a safety.
	 * 
	 * @param leftMotorEncoderPos  The raw encoder value of the left side of the
	 *                             robot.
	 * @param rightMotorEncoderPos The raw encoder value of the right side of the
	 *                             robot.
	 * @param pitch                The pitch angle of the robot, in degrees, as
	 *                             measured by an IMU.
	 * @param roll                 The roll angle of the robot, in degrees, as
	 *                             measured by an IMU.
	 * @param yaw                  The compass heading of the robot, in degrees, as
	 *                             measured by an IMU.
	 */
	public void updateRobotPositon(double leftMotorEncoderPos, double rightMotorEncoderPos, double pitch, double roll, double yaw)
	{
		drivePower = 0;
		steeringPower = 0;
		position.a = Math.toRadians(pitch);
		position.b = Math.toRadians(roll);
		double newHeading = Math.toRadians(yaw);
		double headingRate = newHeading - position.c;
		double predictedHeading = newHeading + headingRate;
		position.c = Math.toRadians(newHeading);

		currentMotorPositions.set(leftMotorEncoderPos / ENCODER_UNITS_PER_ROTATION,
				rightMotorEncoderPos / ENCODER_UNITS_PER_ROTATION);
		motorVelocities = currentMotorPositions.getSubtraction(previousMotorPositions);
		previousMotorPositions.copy(currentMotorPositions);
		motorVelocities.multiply(METERS_PER_ROTATION);
		motorVelocities.average();

		velocity.set((Math.cos(predictedHeading) * motorVelocities.average), (-Math.sin(predictedHeading) * motorVelocities.average));
		position.add(velocity);
	}

	/**
	 * Updates the steering and throttle of the robot to go to
	 * the input waypoint.
	 * 
	 * @param waypoint The location on the field to go to.
	 * @return how long the robot has been at the waypoint.
	 */
	public int goToWaypoint(final CartesianVector waypoint)
	{
		positionError = waypoint.getSubtraction(position);
		if (hasReachedWaypoint())
		{
			timeAtWaypoint ++;
		} else
		{
			timeAtWaypoint = 0;
			directionToWaypoint = -positionError.direction2D();
			directionWrap = (((directionToWaypoint - position.c - Math.PI) % (Math.PI * 2)) + Math.PI);

			targetSpeed = Math.pow(Math.cos(directionWrap),7) * Math.sqrt(positionError.magnitude2D());
			drivePower = drivetrainSpeedPID.runPID(targetSpeed, motorVelocities.average);

			targetHeading = directionToWaypoint;
			if (reverse)
			{
				targetHeading += Math.PI;
			}

			headingWrap = (((position.c - targetHeading - Math.PI) % (Math.PI * 2)) + Math.PI);
			steeringPower = drivetrainHeadingPID.runPID(0, headingWrap);
		}
		return timeAtWaypoint;
	}

	/**
	 * Updates the steering and throttle of the robot to go
	 * to the 'to' waypoint, while staying on the line between
	 * the 'from' and 'to' points.
	 * 
	 * @param fromWaypoint The location on the field to go from.
	 * @param toWaypoint The location on the field to go to.
	 * @return how long the robot has been at the waypoint.
	 */
	public int followPathBetweenWaypoints(CartesianVector fromWaypoint, CartesianVector toWaypoint)
	{
		positionError = toWaypoint.getSubtraction(position);
		CartesianVector extendedWaypoint = fromWaypoint.getSubtraction(toWaypoint);
		extendedWaypoint.normalize();
		extendedWaypoint.multiply(positionError.magnitude2D()/1.1);
		extendedWaypoint.add(toWaypoint);
		return goToWaypoint(extendedWaypoint);
	}

	/**
	 * Updates the steering power to point at a waypoint.
	 * 
	 * @param waypoint The location on the field to point at.
	 * @return how long the robot has been pointing at the waypoint.
	 */
	public int pointAtWaypoint(final CartesianVector waypoint)
	{
		positionError = waypoint.getSubtraction(position);
		targetHeading = -positionError.direction2D();
		headingWrap = (((position.c - targetHeading - Math.PI) % (Math.PI * 2)) + Math.PI);
		steeringPower = drivetrainHeadingPID.runPID(0, headingWrap);
		if (steeringPower < 0.1)
		{
			timeAtWaypoint ++;
		} else
		{
			timeAtWaypoint = 0;
		}
		return timeAtWaypoint;
	}

	/**
	 * Gets the desired steering power of the robot, to reach the target waypoint,
	 * as of the last update.
	 * 
	 * @return The power level, from -1 to 1, that the robot needs the steering to
	 *         be set to.
	 */
	public double getSteeringPower()
	{
		return steeringPower;
	}

	/**
	 * Gets the desired drive power of the robot, to reach the target waypoint, as
	 * of the last update.
	 * 
	 * @return The power level, from -1 to 1, that the robot needs the drive to be
	 *         set to.
	 */
	public double getDrivePower()
	{
		return drivePower;
	}

	/**
	 * Sets the robots current position to a new value, erasing any previous
	 * position tracking.
	 * 
	 * @param newPosition A 6D vector object, containing the desired x, y, z,
	 * 					  a, b, and c values to set the position to.
	 */
	public void setPosition(CartesianVector newPosition)
	{
		position.copy(newPosition);
	}

	/**
	 * Gets the currently tracked position of the robot. This is updated whenever
	 * updatePosition() or setPosition() are called.
	 * 
	 * @return The tracked position of the robot. If updatePosition() has been
	 *         running, this will reflect the robots position, in inches.
	 */
	public CartesianVector getPosition()
	{
		return position.clone();
	}

	/**
	 * Sets the reverse setting of the robot. This value controls if the robot will
	 * go to waypoints while driving backwards or forwards.
	 * 
	 * @param driveReverse A boolean for if the robot will drive in reverse. When
	 *                     true, it will drive backwards. When false, it will drive
	 *                     forwards.
	 */
	public void setReverse(boolean driveReverse)
	{
		reverse = driveReverse;
	}

	/**
	 * Sets the robots current target, which is used by
	 * updateTargetHeadingAndSpeed().
	 * 
	 * @param waypoint A 2D vector object, containing the desired x and y values to
	 *                 set the target waypoint to.
	 */
	public void setWaypoint(final CartesianVector waypoint)
	{
		target.copy(waypoint);
	}

	/**
	 * A simple check for if the robot is within the configured MAX_WAYPOINT_ERROR
	 * of the currently set waypoint.
	 * 
	 * @return True if the robot position is near the waypoint.
	 */
	public boolean hasReachedWaypoint()
	{
		return Math.abs(positionError.magnitude2D()) < MAX_WAYPOINT_ERROR;
	}

	/**
	 * A simple check for if the robot is within the configured MAX_WAYPOINT_ERROR
	 * of the input point.
	 * 
	 * @param point A 2D vector containing the x and y values to check against.
	 * @return True if the robot position is near the input point.
	 */
	public boolean isAtPoint(CartesianVector point)
	{
		return Math.abs(position.getSubtraction(point).magnitude2D()) < MAX_WAYPOINT_ERROR;
	}

}
