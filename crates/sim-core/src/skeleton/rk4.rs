//! Classic fixed-step 4th-order Runge–Kutta for small fixed-size state vectors.

/// Advance `x` by one step `dt` of dx/dt = f(x). `f` writes the derivative into its 2nd argument.
pub fn rk4_step<const N: usize>(x: &mut [f64; N], dt: f64, f: impl Fn(&[f64; N], &mut [f64; N])) {
    let mut k1 = [0.0; N];
    let mut k2 = [0.0; N];
    let mut k3 = [0.0; N];
    let mut k4 = [0.0; N];
    let mut tmp = [0.0; N];

    f(x, &mut k1);
    for i in 0..N {
        tmp[i] = x[i] + 0.5 * dt * k1[i];
    }
    f(&tmp, &mut k2);
    for i in 0..N {
        tmp[i] = x[i] + 0.5 * dt * k2[i];
    }
    f(&tmp, &mut k3);
    for i in 0..N {
        tmp[i] = x[i] + dt * k3[i];
    }
    f(&tmp, &mut k4);
    for i in 0..N {
        x[i] += dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// dx/dt = -x has the exact solution e^{-t}; RK4 at dt = 0.01 is accurate to ~1e-10.
    #[test]
    fn exponential_decay_matches_exact_solution() {
        let mut x = [1.0];
        let dt = 0.01;
        for _ in 0..100 {
            rk4_step(&mut x, dt, |x, dx| dx[0] = -x[0]);
        }
        let exact = (-1.0f64).exp();
        // RK4 global error ~ C·dt⁴ ≈ 1e-8·C with C ≪ 1 here; 1e-9 abs + 1e-9 rel.
        assert!(
            (x[0] - exact).abs() <= 1e-9 + 1e-9 * exact,
            "x = {}, exact = {exact}",
            x[0]
        );
    }
}
