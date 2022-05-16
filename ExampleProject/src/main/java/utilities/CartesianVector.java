package utilities;

/**
 * A class for creating and working with 2D and 3D vectors Each instance
 * contains an x, y, and optionally a z value, along with various functions for
 * performing standard math operations on itself.
 * 
 * @author Icarus Innovated
 */
public class CartesianVector
{
	/**
	 * The x component of the vector
	 */
	public double x;

	/**
	 * The y component of the vector
	 */
	public double y;

	/**
	 * The z component of the vector. If a vector is created with only two
	 * parameters, this value will remain 0
	 */
	public double z;

	/**
	 * The direction of the vector, in radians. This is 0 unless the direction
	 * function is called.
	 */
	public double direction;

	/**
	 * The magnitude of the vector. This is 0 unless the magnitude function is
	 * called.
	 */
	public double magnitude;

	/**
	 * The average value of the components of the vector. This is 0 unless the
	 * average function is called.
	 */
	public double average;

	/**
	 * If the vector is created with 3 values, this will be true. Otherwise, it will
	 * be false.
	 */
	public final boolean is3D;

	/**
	 * Makes a 3D vector object with the specified values.
	 * 
	 * @param initialX A double representing the initial x value.
	 * @param initialY A double representing the initial y value.
	 * @param initialZ A double representing the initial z value.
	 */
	public CartesianVector(double initialX, double initialY, double initialZ)
	{
		this.x = initialX;
		this.y = initialY;
		this.z = initialZ;
		this.is3D = true;
	}

	/**
	 * Makes a 2D vector object with the specified values.
	 * 
	 * @param initialX A double representing the initial x value.
	 * @param initialY A double representing the initial y value.
	 */
	public CartesianVector(double initialX, double initialY)
	{
		x = initialX;
		y = initialY;
		z = 0;
		is3D = false;
	}

	/**
	 * Returns a vector object with the specified values and forces it to match the
	 * given type.
	 * 
	 * @param force3D  If this is false, the input z value will be discarded.
	 * @param initialX A double representing the initial x value.
	 * @param initialY A double representing the initial y value.
	 * @param initialZ A double representing the initial z value.
	 * @return A vector with the same values as the inputs, and with the same type
	 *         as the input (2D or 3D).
	 */
	private CartesianVector vectorType(boolean force3D, double initialX, double initialY, double initialZ)
	{
		if (force3D)
		{
			return new CartesianVector(initialX, initialY, initialZ);
		}
		return new CartesianVector(initialX, initialY);
	}

	/**
	 * Copies the value of an existing vector into the parent vector.
	 * 
	 * @param a The vector to be copied.
	 */
	public void copy(CartesianVector a)
	{
		x = a.x;
		y = a.y;
		if (is3D)
		{
			z = a.z;
		}
	}

	/**
	 * Sets an existing vector to the specified values.
	 * 
	 * @param newX A double representing the new x value.
	 * @param newY A double representing the new y value.
	 * @param newZ A double representing the new z value.
	 */
	public void set(double newX, double newY, double newZ)
	{
		x = newX;
		y = newY;
		z = newZ;
	}

	/**
	 * Sets an existing vector to the specified values.
	 * 
	 * @param newX A double representing the new x value.
	 * @param newY A double representing the new y value.
	 */
	public void set(double newX, double newY)
	{
		x = newX;
		y = newY;
	}

	/**
	 * Creates a clone of the vector
	 * 
	 * @return An exact copy of the initial vector.
	 */
	public CartesianVector clone()
	{
		return vectorType(is3D, x, y, z);
	}

	/**
	 * Gets the result of adding another vector to the parent vector. This function
	 * does NOT modify the parent vector.
	 * 
	 * @param a The vector to add to the parent.
	 * @return A new vector containing the result of the addition. The result will
	 *         be the same type as the parent (2D or 3D).
	 */
	public CartesianVector getAddition(CartesianVector a)
	{
		return vectorType(is3D, x + a.x, y + a.y, z + a.z);
	}

	/**
	 * Gets the result of subtracting another vector from the parent vector. This
	 * function does NOT modify the parent vector.
	 * 
	 * @param a The vector to subtract from the parent.
	 * @return A new vector containing the result of the subtraction. The result
	 *         will be the same type as the parent (2D or 3D).
	 */
	public CartesianVector getSubtraction(CartesianVector a)
	{
		return vectorType(is3D, x - a.x, y - a.y, z - a.z);
	}

	/**
	 * Gets the result of multiplying the parent vector by a number. This function
	 * does NOT modify the parent vector.
	 * 
	 * @param scalar A double that will be used to multiply each component of the
	 *               parent vector
	 * @return A new vector3D containing the result of the multiplication. The
	 *         result will be the same type as the parent (2D or 3D).
	 */
	public CartesianVector getMultiplication(double scalar)
	{
		return vectorType(is3D, x * scalar, y * scalar, z * scalar);
	}

	/**
	 * Gets the result of dividing the parent vector by a number. This function does
	 * NOT modify the parent vector.
	 * 
	 * @param scalar A double that will be used to divide each component of the
	 *               parent vector.
	 * @return A new vector3D containing the result of the division. The result will
	 *         be the same type as the parent (2D or 3D).
	 */
	public CartesianVector getDivision(double scalar)
	{
		return vectorType(is3D, x / scalar, y / scalar, z / scalar);
	}

	/**
	 * Gets the result of normalizing the parent vector to a length of 1. This
	 * function does NOT modify the parent vector. Calling this function also
	 * updates the parent vector's magnitude.
	 * 
	 * @return A new vector3D with the same direction as the parent, but with a
	 *         length of 1. The result will be the same type as the parent (2D or
	 *         3D).
	 */
	public CartesianVector getNormalization()
	{
		return getDivision(magnitude());
	}

	/**
	 * Adds another vector to the parent vector. This function DOES modify the
	 * parent vector.
	 * 
	 * @param a The vector to add to the parent.
	 */
	public void add(CartesianVector a)
	{
		x += a.x;
		y += a.y;
		if (is3D)
		{
			z += a.z;
		}
	}

	/**
	 * Subtracts another vector to the parent vector. This function DOES modify the
	 * parent vector.
	 * 
	 * @param a The vector to subtract from the parent.
	 */
	public void subtract(CartesianVector a)
	{
		x -= a.x;
		y -= a.y;
		if (is3D)
		{
			z -= a.z;
		}
	}

	/**
	 * Multiplies the parent vector by a number. This function DOES modify the
	 * parent vector.
	 * 
	 * @param scalar A double that will be used to multiply each component of the
	 *               parent vector.
	 */
	public void multiply(double scalar)
	{
		x *= scalar;
		y *= scalar;
		if (is3D)
		{
			z *= scalar;
		}
	}

	/**
	 * Divides the parent vector by a number. This function DOES modify the parent
	 * vector.
	 * 
	 * @param scalar A double that will be used to divide each component of the
	 *               parent vector.
	 */
	public void divide(double scalar)
	{
		x /= scalar;
		y /= scalar;
		if (is3D)
		{
			z /= scalar;
		}
	}

	/**
	 * Normalizes the parent vector to a length of 1. This function DOES modify the
	 * parent vector. Calling this function also updates the parent vector's
	 * magnitude.
	 */
	public void normalize()
	{
		divide(magnitude());
	}

	/**
	 * Calculates magnitude of the parent vector.
	 * 
	 * @return A double representing the magnitude of the parent vector. This value
	 *         is also stored in the parent to avoid extra calculations when
	 *         possible.
	 */
	public double magnitude()
	{
		magnitude = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
		return magnitude;
	}

	/**
	 * Calculates the direction of the parent vector.
	 * 
	 * @return A double representing the direction of the parent vector in radians.
	 *         This value is also stored in the parent to avoid extra calculations
	 *         when possible.
	 */
	public double direction()
	{
		direction = Math.atan2(y, x);
		return direction;
	}

	/**
	 * Calculates average of the parent vector's components.
	 * 
	 * @return A double representing the average of the parent vector's components.
	 *         This is only useful in a few cases, so ensure this is the right
	 *         function. This value is also stored in the parent to avoid extra
	 *         calculations when possible.
	 */
	public double average()
	{
		average = x + y + z;
		if (is3D)
		{
			average = average / 3;
		} else
		{
			average = average / 2;
		}
		return average;
	}
}
