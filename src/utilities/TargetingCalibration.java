package utilities;

import java.util.HashMap;

public class TargetingCalibration
{
    /**
	 * The callibration data for the system. The key is the distance,
	 * the vector is the time, velocity, and angle of the shot.
	 */
	private final HashMap<Double, CartesianVector> CALIBRATION_DATA;

    /**
	 * The interpolated data point for the current distance.
	 */
    private CartesianVector lerpedPoint;

    private double lowerDist;
    private double upperDist;
    private double distRange;
    private double distPercent;
    private CartesianVector lowerData;
    
    public TargetingCalibration()
    {
        this.CALIBRATION_DATA = new HashMap<Double, CartesianVector>();
    }

    public TargetingCalibration(HashMap<Double, CartesianVector> CALIBRATION_DATA)
    {
        this.CALIBRATION_DATA = CALIBRATION_DATA;
        upperDist = 0;
        lowerDist = 99999;
        for (double dist : this.CALIBRATION_DATA.keySet())
        {
            if (dist < lowerDist) {lowerDist = dist;}
            if (dist > upperDist) {upperDist = dist;}
        }
        CartesianVector maxData = this.CALIBRATION_DATA.get(upperDist);
        this.CALIBRATION_DATA.put(99999.0, maxData);
        CartesianVector minData = this.CALIBRATION_DATA.get(lowerDist);
        this.CALIBRATION_DATA.put(0.0, minData);
    }

    public void append(double distance, CartesianVector data)
    {
        CALIBRATION_DATA.put(distance, data);
    }
    
    /**
	 * Interpolates between calibration points by a distance.
	 * 
	 * @param distance The distance to create an interpolated point at.
	 */
    public CartesianVector get(double distance)
    {
        if (CALIBRATION_DATA.containsKey(distance))
        {
            return CALIBRATION_DATA.get(distance);
        }
        lowerDist = 0;
        upperDist = 999999;
        for (double dist : CALIBRATION_DATA.keySet())
        {
            if (dist < distance && dist > lowerDist)
            {
                lowerDist = dist;
            }
            if (dist > distance && dist < upperDist)
            {
                upperDist = dist;
            }
        }
        distRange = upperDist - lowerDist;
        distPercent = (distance - lowerDist) / distRange;
        lowerData = CALIBRATION_DATA.get(lowerDist);
		lerpedPoint = CALIBRATION_DATA.get(upperDist).getSubtraction(lowerData);
		lerpedPoint.multiply(distPercent);
        lerpedPoint.add(lowerData);
        return lerpedPoint;
    }
}
