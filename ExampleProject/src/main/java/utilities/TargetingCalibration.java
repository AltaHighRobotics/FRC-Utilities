package utilities;

/**
 * A class for creating a calibration table for a shooter.
 *
 * @author Icarus Innovated
 */
public class TargetingCalibration
{
    /**
	 * The callibration data for the system. x is distance,
     * y is time, z is velocity, and a is angle.
	 */
	private final CartesianVector[] CALIBRATION_DATA;

    /**
	 * The interpolated data point for the current distance.
	 */
    private CartesianVector lerpedPoint;

    /**
     * The smallest distance that was given in the input calibration.
     */
    private final double minCalibratedDist;

    /**
     * The largest distance that was given in the input calibration.
     */
    private final double maxCalibratedDist;

    /**
     * The data point directly below the target distance.
     */
    private CartesianVector lowerData;

    /**
     * The data point directly above the target distance.
     */
    private CartesianVector upperData;

    /**
     * Creates a calibration table with the given data.
     * 
     * @param CALIBRATION_DATA A CartesianVector[] where x is distance,
     *                         y is time, z is velocity, and a is angle.
     */
    public TargetingCalibration(CartesianVector[] CALIBRATION_DATA)
    {
        this.lowerData = CALIBRATION_DATA[0];
        this.minCalibratedDist = this.lowerData.x;
        this.lerpedPoint = this.lowerData;
        this.upperData = CALIBRATION_DATA[CALIBRATION_DATA.length - 1];
        this.maxCalibratedDist = this.upperData.x;
        this.CALIBRATION_DATA = CALIBRATION_DATA;
    }
    
    /**
	 * Interpolates between calibration points by a distance.
	 * 
	 * @param distance The distance to create an interpolated point at.
	 */
    public CartesianVector get(double distance)
    {
        if (distance > maxCalibratedDist || distance < minCalibratedDist)
        {
            return lerpedPoint;
        }
        double lowerDist = 0;
        double upperDist = 1000;
        for (CartesianVector dataPoint : CALIBRATION_DATA)
        {
            double dist = dataPoint.x;
            if (dist == distance)
            {
                return dataPoint;
            }
            if (dist < distance && dist > lowerDist)
            {
                lowerDist = dist;
                lowerData.copy(dataPoint);
            }
            if (dist > distance)
            {
                upperDist = dist;
                upperData.copy(dataPoint);
                break;
            }
        }
        double distRange = upperDist - lowerDist;
        double distPercent = (distance - lowerDist) / distRange;
		lerpedPoint = upperData.getSubtraction(lowerData);
		lerpedPoint.multiply(distPercent);
        lerpedPoint.add(lowerData);
        return lerpedPoint;
    }
}
