package utilities;

/**
 * A class for creating and working with 2D and 3D vectors Each instance
 * contains an x, y, and optionally a z value, along with various functions for
 * performing standard math operations on itself.
 * 
 * @author Ian
 * @author Icarus Innovated
 */
public class vector
{
	public double x;
	public double y;
	public double z;
	public double direction;
	public double magnitude;
	public double average;
	public final boolean is3D;

	/**
	 * Makes a 3D vector object with the specified values.
	 * 
	 * @param x A double representing the initial x value.
	 * @param y A double representing the initial y value.
	 * @param z A double representing the initial z value.
	 */
	public vector(double x, double y, double z)
	{
		this.x = x;
		this.y = y;
		this.z = z;
		this.is3D = true;
	}

	/**
	 * Makes a 2D vector object with the specified values.
	 * 
	 * @param x A double representing the initial x value.
	 * @param y A double representing the initial y value.
	 */
	public vector(double x, double y)
	{
		this.x = x;
		this.y = y;
		this.z = 0;
		this.is3D = false;
	}

	private vector vectorType(boolean is3D, double x, double y, double z)
	{
		if (is3D)
		{
			return new vector(x, y, z);
		}
		return new vector(x, y);
	}

	/**
	 * Copies the value of an existing vector into the parent vector.
	 * 
	 * @param a The vector to be copied.
	 */
	public void copy(vector a)
	{
		this.x = a.x;
		this.y = a.y;
		if (this.is3D)
		{
			this.z = a.z;
		}
	}

	/**
	 * Sets an existing vector to the specified values.
	 * 
	 * @param x A double representing the new x value.
	 * @param y A double representing the new y value.
	 * @param z A double representing the new z value.
	 */
	public void set(double x, double y, double z)
	{
		this.x = x;
		this.y = y;
		this.z = z;
	}

	/**
	 * Sets an existing vector to the specified values.
	 * 
	 * @param x A double representing the new x value.
	 * @param y A double representing the new y value.
	 */
	public void set(double x, double y)
	{
		this.x = x;
		this.y = y;
	}

	/**
	 * Creates a clone of the vector
	 * 
	 * @return An exact copy of the initial vector.
	 */
	public vector clone()
	{
		return this.vectorType(this.is3D, this.x, this.y, this.z);
	}

	/**
	 * Gets the result of adding another vector to the parent vector. This function
	 * does NOT modify the parent vector.
	 * 
	 * @param a The vector to add to the parent.
	 * @return A new vector containing the result of the addition. The result will
	 *         be the same type as the parent (2D or 3D).
	 */
	public vector getAddition(vector a)
	{
		return this.vectorType(this.is3D, this.x + a.x, this.y + a.y, this.z + a.z);
	}

	/**
	 * Gets the result of subtracting another vector from the parent vector. This
	 * function does NOT modify the parent vector.
	 * 
	 * @param a The vector to subtract from the parent.
	 * @return A new vector containing the result of the subtraction. The result
	 *         will be the same type as the parent (2D or 3D).
	 */
	public vector getSubtraction(vector a)
	{
		return this.vectorType(this.is3D, this.x - a.x, this.y - a.y, this.z - a.z);
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
	public vector getMultiplication(double scalar)
	{
		return this.vectorType(this.is3D, this.x * scalar, this.y * scalar, this.z * scalar);
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
	public vector getDivision(double scalar)
	{
		return this.vectorType(this.is3D, this.x / scalar, this.y / scalar, this.z / scalar);
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
	public vector getNormalization()
	{
		return this.getDivision(this.magnitude());
	}

	/**
	 * Adds another vector to the parent vector. This function DOES modify the
	 * parent vector.
	 * 
	 * @param a The vector to add to the parent.
	 */
	public void add(vector a)
	{
		this.x += a.x;
		this.y += a.y;
		if (this.is3D)
		{
			this.z += a.z;
		}
	}

	/**
	 * Subtracts another vector to the parent vector. This function DOES modify the
	 * parent vector.
	 * 
	 * @param a The vector to subtract from the parent.
	 */
	public void subtract(vector a)
	{
		this.x -= a.x;
		this.y -= a.y;
		if (this.is3D)
		{
			this.z -= a.z;
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
		this.x *= scalar;
		this.y *= scalar;
		if (this.is3D)
		{
			this.z *= scalar;
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
		this.x /= scalar;
		this.y /= scalar;
		if (this.is3D)
		{
			this.z /= scalar;
		}
	}

	/**
	 * Normalizes the parent vector to a length of 1. This function DOES modify the
	 * parent vector. Calling this function also updates the parent vector's
	 * magnitude.
	 */
	public void normalize()
	{
		this.divide(this.magnitude());
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
		this.magnitude = Math.sqrt(Math.pow(this.x, 2) + Math.pow(this.y, 2) + Math.pow(this.z, 2));
		return this.magnitude;
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
		this.average = this.x + this.y + this.z;
		if (this.is3D)
		{
			this.average = this.average / 3;
		} else
		{
			this.average = this.average / 2;
		}
		return this.average;
	}
}
