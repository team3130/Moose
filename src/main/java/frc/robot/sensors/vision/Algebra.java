package frc.robot.sensors.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.ejml.simple.SimpleMatrix;

class Algebra {

    /**
     * Shortcut helper to build 3-D vectors from three values.
     * @param x
     * @param y
     * @param z
     * @return 3x1 Matrix witch is the vector
     */
    public static Matrix<N3,N1> buildVector(double x, double y, double z) {
        return new VecBuilder<>(Nat.N3()).fill(x,y,z);
    }

    /**
     * Calculate the rotation matrix based on a rotation vector
     * using the Rodrigues formula
     * https://en.wikipedia.org/wiki/Rodrigues'_rotation_formula
     * @param rvec the rotation vector, its magnitude is the angle of the rotation
     * @return 3x3 Matrix that can be used to rotate vectors around rvec
     */
    public static Matrix<N3,N3> Rodrigues(Matrix<N3,N1> rvec) {
        if (rvec.getNumRows() != 3 || rvec.getNumCols() != 1) {
            throw new IllegalArgumentException(
                "Rotation vector must be 3-dimensional but nRows = "
                 + rvec.getNumRows() + ", nCols = " + rvec.getNumCols());
        }
        // Rotation angle is simply the length of the r-vector
        final double theta = rvec.normF();

        // A zero rotation vector is an edge case scenario that can crash
        // our further computations but in reality is a trivial solution. So don't crash.
        if (theta == 0.0) return new Matrix<>(SimpleMatrix.identity(3));

        Matrix<N3,N1> k = rvec.times(1/theta);
        Matrix<N3,N3> K = new MatBuilder<N3,N3>(Nat.N3(),Nat.N3()).fill(
            0.0,        -k.get(2,0),  k.get(1,0),
            k.get(2,0),  0.0,        -k.get(0,0),
           -k.get(1,0),  k.get(0,0),  0.0
        );

        Matrix<N3,N3> K2 = K.times(K);
        Matrix<N3,N3> R = new Matrix<>(
            SimpleMatrix.identity(3)
            .plus(    Math.sin(theta),  K.getStorage())
            .plus(1 - Math.cos(theta), K2.getStorage())
        );

        return R;
    }

}