package org.firstinspires.ftc.teamcode.code.utility;

public class QuarticSolver {

    private static final double EPS = 1e-9;
    private static final int MAX_ITER = 1000;

    public static Double lowestPositiveRealSolution(
        double a, double b, double c, double d) {

        // Solve a t^4 + b t^2 + c t + d = 0
        Complex[] roots = solveQuartic(a, 0.0, b, c, d);

        Double smallest = null;

        for (Complex root : roots) {
            if (Math.abs(root.imag) < 1e-6) {
                double real = root.real;
                if (real > 0) {
                    if (smallest == null || real < smallest) {
                        smallest = real;
                    }
                }
            }
        }

        if (smallest == null) return null;

        return smallest;
    }

    // ---- Quartic via Durand–Kerner ----

    private static Complex[] solveQuartic(double a4, double a3,
                                          double a2, double a1, double a0) {

        // Normalize polynomial
        a3 /= a4;
        a2 /= a4;
        a1 /= a4;
        a0 /= a4;

        Complex[] roots = new Complex[4];
        roots[0] = new Complex(1, 0);
        roots[1] = new Complex(0, 1);
        roots[2] = new Complex(-1, 0);
        roots[3] = new Complex(0, -1);

        for (int iter = 0; iter < MAX_ITER; iter++) {
            boolean converged = true;

            for (int i = 0; i < 4; i++) {

                Complex numerator = evaluate(roots[i], a3, a2, a1, a0);
                Complex denominator = Complex.ONE;

                for (int j = 0; j < 4; j++) {
                    if (i != j) {
                        denominator = denominator.multiply(
                            roots[i].subtract(roots[j]));
                    }
                }

                Complex delta = numerator.divide(denominator);
                roots[i] = roots[i].subtract(delta);

                if (delta.abs() > EPS) {
                    converged = false;
                }
            }

            if (converged) break;
        }

        return roots;
    }

    private static Complex evaluate(Complex x,
                                    double a3, double a2,
                                    double a1, double a0) {

        // x^4 + a3 x^3 + a2 x^2 + a1 x + a0
        return x.pow(4)
            .add(x.pow(3).scale(a3))
            .add(x.pow(2).scale(a2))
            .add(x.scale(a1))
            .add(new Complex(a0, 0));
    }

    // ---- Minimal Complex class ----

    private static class Complex {
        final double real;
        final double imag;

        static final Complex ONE = new Complex(1, 0);

        Complex(double r, double i) {
            this.real = r;
            this.imag = i;
        }

        Complex add(Complex o) {
            return new Complex(real + o.real, imag + o.imag);
        }

        Complex subtract(Complex o) {
            return new Complex(real - o.real, imag - o.imag);
        }

        Complex multiply(Complex o) {
            return new Complex(
                real*o.real - imag*o.imag,
                real*o.imag + imag*o.real
            );
        }

        Complex divide(Complex o) {
            double denom = o.real*o.real + o.imag*o.imag;
            return new Complex(
                (real*o.real + imag*o.imag)/denom,
                (imag*o.real - real*o.imag)/denom
            );
        }

        Complex scale(double s) {
            return new Complex(real*s, imag*s);
        }

        Complex pow(int n) {
            Complex result = new Complex(1, 0);
            for (int i = 0; i < n; i++)
                result = result.multiply(this);
            return result;
        }

        double abs() {
            return Math.hypot(real, imag);
        }
    }
}