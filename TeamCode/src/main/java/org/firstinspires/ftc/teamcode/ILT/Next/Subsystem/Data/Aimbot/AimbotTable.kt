package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Aimbot

/**
 * Lookup table for hood position and flywheel velocity based on distance.
 * Values are tuned empirically during testing.
 */
object AimbotTable {

    /**
     * Distance-indexed lookup table.
     * Each entry: distance (inches) -> Pair(hoodPosition, flywheelVelocity)
     */
    private val lookupTable: Map<Int, Pair<Double, Double>> = mapOf(
        12 to Pair(0.81, 835.0),
        24 to Pair(0.93, 862.0),
        36 to Pair(0.71, 844.0),
        48 to Pair(0.60, 848.0),
        60 to Pair(0.62, 908.0),
        72 to Pair(0.65, 1025.0),
        84 to Pair(0.70, 1165.0),
        96 to Pair(0.70, 1230.0),
        108 to Pair(0.42, 1070.0),
        120 to Pair(0.43, 1112.0),
        132 to Pair(0.44, 1150.0),
        144 to Pair(0.45, 1250.0)
    )

    // Sorted distances for interpolation
    private val distances = lookupTable.keys.sorted()

    /**
     * Get aim values for a given distance.
     * Returns Pair(hoodPosition, flywheelVelocity) or null if out of range.
     *
     * Uses linear interpolation between known values.
     */
    fun getValues(distance: Double): Pair<Double, Double>? {
        if (distance < distances.first() || distance > distances.last()) {
            return null
        }

        // Find surrounding distances for interpolation
        var lowerDist = distances.first()
        var upperDist = distances.last()

        for (d in distances) {
            if (d <= distance) lowerDist = d
            if (d >= distance) {
                upperDist = d
                break
            }
        }

        // Exact match
        if (lowerDist == upperDist) {
            return lookupTable[lowerDist]
        }

        // Interpolate
        val lowerVals = lookupTable[lowerDist] ?: return null
        val upperVals = lookupTable[upperDist] ?: return null

        val t = (distance - lowerDist) / (upperDist - lowerDist)

        val hood = lowerVals.first + t * (upperVals.first - lowerVals.first)
        val velocity = lowerVals.second + t * (upperVals.second - lowerVals.second)

        return Pair(hood, velocity)
    }

    /**
     * Get hood position for distance (with offset applied)
     */
    fun getHoodPosition(distance: Double, offset: Double = 0.06): Double? {
        return getValues(distance)?.first?.plus(offset)?.coerceIn(0.0, 1.0)
    }

    /**
     * Get flywheel velocity for distance (with offset applied)
     */
    fun getFlywheelVelocity(distance: Double, offset: Double = 100.0): Double? {
        return getValues(distance)?.second?.plus(offset)
    }

    /**
     * Snap to nearest valid distance (multiples of 12)
     */
    fun snapToValidDistance(distance: Int): Int {
        val snapped = (distance / 12) * 12
        return snapped.coerceIn(12, 144)
    }

    /**
     * Get all valid distances for UI/debugging
     */
    fun getValidDistances(): List<Int> = distances
}