# Kinematics for the PRR used in Calgames 2025.

Important classes here:

* `ElevatorArmWristKinematics` relates joint configuration to cartesian end-effector pose and vice-versa.
* `AnalyticalJacobian` relates joint velocities to cartesian velocities and vice-versa, and the same for acceleration.

The math, in brief:

The mechanism position is:

```math
\begin{equation}
x = f(q) =
\begin{bmatrix}
q_1 + l_2c_2 + l_3c_{23} \\
l_2s_2 + l_3c_{23} \\
q_2 + q_3
\end{bmatrix}
\end{equation}
```

where $q_i$ are joint configurations, $l_i$ are link lengths,
the tuple, $x$, represents the end-effector pose: $(x, y, \theta)$
and $c_2$ etc are $cos(q_2)$.

To find the velocity, differentiate position and apply the chain rule:

```math
\begin{equation}
\dot{x} = \dot{f}\dot{q}
\end{equation}
```

The derivative function, $\dot{f}$ is also called the _Jacobian_ matrix:

```math
\begin{equation}
J =
\begin{bmatrix}
1 & -l_2s_2 - l_3s_{23} & -l_3s_{23}\\
0 & l_2c_2 + l_3c_{23} & l_3c_{23}\\
0 & 1 & 1
\end{bmatrix}
\end{equation}
```

So the function for cartesian velocity, $\dot{x}$, can be written:

```math
\begin{equation}
\dot{x} = J \dot{q}
\end{equation}
```

And the inverse is

```math
\begin{equation}
\dot{q} = J^{-1}\dot{x}
\end{equation}
```

To find the acceleration, differentiate again and apply the product rule:

```math
\begin{equation}
\ddot{x} = \dot{J}\dot{q} + J\ddot{q}
\end{equation}
```

Computing the time derivative of the Jacobian is simple enough, remembering
to apply the chain rule

```math
\begin{equation}
\dot{J} =
\begin{bmatrix}
0 & -l_2c_2\dot{q_2} - l_3c_{23}(\dot{q_2}+\dot{q_3}) & -l_3c_{23}(\dot{q_2}+\dot{q_3}) \\
0 & -l_2s_2\dot{q_2} - l_3s_{23}(\dot{q_2}+\dot{q_3}) & -l_3s_{23}(\dot{q_2}+\dot{q_3}) \\
0 & 0 & 0
\end{bmatrix}
\end{equation}
```

To find the joint accelerations from the cartesian requires only the inverse of
the Jacobian itself, not the inverse of the derivative:

```math
\begin{equation}
\ddot{q} = J^{-1}(\ddot{x} - \dot{J}\dot{q})
\end{equation}
```

Or

```math
\begin{equation}
\ddot{q} = J^{-1}(\ddot{x} - \dot{J}J^{-1}\dot{x})
\end{equation}
```
