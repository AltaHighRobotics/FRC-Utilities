package utilities;

import java.util.ArrayList;

/** A class for creating and tracking a target in 3D space. Also allows for predicting where the target will be in the future.
 * 
 *  @author Ian
 *  @author Tim
 *  @author Icarus Innovated
 */
public class TargetingSystem
{
    private CartesianVector position = new CartesianVector(0, 0, 0);
    private CartesianVector previousPosition = new CartesianVector(0, 0, 0);
    private ArrayList<CartesianVector> positions = new ArrayList<CartesianVector>();
    private CartesianVector averagePosition = new CartesianVector(0, 0, 0);
    private CartesianVector predictedPosition = new CartesianVector(0, 0, 0);
    private CartesianVector velocity = new CartesianVector(0, 0, 0);
    private CartesianVector previousVelocity = new CartesianVector(0, 0, 0);
    private ArrayList<CartesianVector> velocities = new ArrayList<CartesianVector>();
    private CartesianVector averageVelocity = new CartesianVector(0, 0, 0);
    private CartesianVector acceleration = new CartesianVector(0, 0, 0);
    private ArrayList<CartesianVector> accelerations = new ArrayList<CartesianVector>();
    private CartesianVector averageAcceleration = new CartesianVector(0, 0, 0);
    public CartesianVector leadPosition = new CartesianVector(0, 0, 0);
    private double distance;
    private double speed;
    private double velocityThreshold;
    private int age = 1;
    private int refreshThreshold;
    private int maxMemory;
    private int maxAccelerationPrediction;
    private boolean updated;

    private double CAMERA_HEIGHT;
    private double CAMERA_ELEVATION_ANGLE;
    private double GOAL_HEIGHT;
    private double absoluteAzimuthToTarget;
    private double absoluteElevationToTarget;
    private double azimuthToGoalRadians;
    private double elevationToGoalRadians;
    private double distanceToGoal;
    private CartesianVector relativeGoalPosition;
    private CartesianVector absoluteGoalPosition;
    private CartesianVector realGoalPosition;
  
    public TargetingSystem(double velocityThreshold, int refreshThreshold, int maxMemory, int maxAccelerationPrediction)
    {
        this.velocityThreshold = velocityThreshold;
        this.refreshThreshold = refreshThreshold;
        this.maxMemory = maxMemory;
        this.maxAccelerationPrediction = maxAccelerationPrediction;
        this.relativeGoalPosition = new CartesianVector(0, 0, 0);
        this.absoluteGoalPosition = new CartesianVector(0, 0, 0);
        this.realGoalPosition = new CartesianVector(0, 0, 0);
    }

    public void setNewTargetPosition(CartesianVector newPosition)
    {
        position.copy(newPosition);
        updated = true;
    }
    
    public void setNewTargetPositionFromTargetingCamera(double robotYaw, double targetingAzimuth, double targetingElevation)
    {
        absoluteAzimuthToTarget = targetingAzimuth + robotYaw;
        azimuthToGoalRadians = Math.toRadians(absoluteAzimuthToTarget);

        absoluteElevationToTarget = targetingElevation + CAMERA_ELEVATION_ANGLE;
        elevationToGoalRadians = Math.toRadians(absoluteElevationToTarget);

        distanceToGoal = (GOAL_HEIGHT-CAMERA_HEIGHT)/(Math.tan(elevationToGoalRadians));
        relativeGoalPosition.set(Math.cos(azimuthToGoalRadians)*distanceToGoal,Math.sin(azimuthToGoalRadians)*distanceToGoal);
        position.copy(relativeGoalPosition);
        
        updated = true;
    }

    public CartesianVector getOdometryCalibration(CartesianVector robotPosition)
    {
        absoluteGoalPosition = relativeGoalPosition.getAddition(robotPosition);
        return absoluteGoalPosition.getSubtraction(realGoalPosition);
    }

    /** Updates the target position, and computes the velocity and acceleration of the target.
     * 
     */
    public void updateTarget()
    {
        if(updated)
        {
            if(age > refreshThreshold)
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
            if(speed < velocityThreshold)
            {
                averageVelocity.set(0,0,0);
                averageAcceleration.set(0,0,0);
            }
            age = 0;
        }
        updated = false;
        age = age + 1;
        predictedPosition.add(averageVelocity.getAddition(averageAcceleration.getMultiplication(Math.min(age, maxAccelerationPrediction))));
    }

    public void updateTargetLeadPosition(double timeToTarget)
    {
        leadPosition = predictedPosition.getAddition(averageVelocity.getMultiplication(timeToTarget));
    }

    /** Calculates the distance between the input location and the target location.
     * 
     * @param robotLocation The location of the robot on the field, relative to the target.
     * @return The distance between the robot and the target, in the form of a 3D vector.
     */
    public double getTargetDistance(CartesianVector robotLocation)
    {
        distance = predictedPosition.getSubtraction(robotLocation).magnitude();
        return distance;
    }

    /** Gets the average of a list of vectors
     * 
     * @param vectorList an ArrayList of vector objects
     * @return A vector representing the average value of the vectors in the list.
     */
    private CartesianVector avgVectorList(ArrayList<CartesianVector> vectorList)
    {
        if(vectorList.size() >= 1) {
            CartesianVector total = vectorList.get(0).clone();
            total.set(0, 0, 0);
            for(CartesianVector i : vectorList) {
                total.add(i);
            }
            total.divide(vectorList.size());
            return total;
        }
        return new CartesianVector(0, 0, 0);
    }

    /** Adds a vector to a list of vectors, and removes the oldest vector if the list is above maxSize.
     * 
     * @param vectorList The list of vectors to modify.
     * @param newItem The new vector to add to the end of the list.
     * @param maxSize The maximum length the list is allowed to reach.
     * @return The modified version of the input list.
     */
    private ArrayList<CartesianVector> manageList(ArrayList<CartesianVector> vectorList, CartesianVector newItem, int maxSize)
    {
        vectorList.add(newItem);
        if(vectorList.size() > maxSize) {
            vectorList.remove(0);
        }
        return vectorList;
    }
}