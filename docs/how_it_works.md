# How it works

A short tour of the core ideas. [Read this for a great intro to Gaussian Belief Propagation.](https://gaussianbp.github.io)

> These instruction pages are a work in progress and will keep being updated — check back for more.

## Factor graphs, solved with Gaussian Belief Propagation (GBP)

An optimisation problem is expressed as a set of **Variables** (the unknowns) related to one
another by quadratic cost functions called **Factors**. Together the Variables and Factors form a
**Factor Graph**, which we solve with **Gaussian Belief Propagation (GBP)**: an iterative
message-passing algorithm where each node refines its estimate from messages sent by its neighbours.

GBP is local — a node only talks to the factors/variables it is directly connected to, which is what
lets the same machinery run distributed across many robots.

Start by creating a graph. Variables and factors are added into a **layer**, a self-contained
sub-problem (more on layers [below](#how-its-organised)); a simple graph has just one:

```cpp
FactorGraph fg;                                   // the graph
int layer_id = fg.addLayer("my problem")->lid_;   // a layer to hold the nodes
```

You then add variables and factors into the layer (next two sections) and solve with GBP. Both
`addVariable` and `addFactor` create the node, assign it a `Key` (see [below](#how-its-organised)),
and return a handle to it.

## Variables

A **Variable** is an unknown we want to estimate. It can live on any Lie group — `R^N`, `SO(2)`,
`SE(2)`, `SO(3)`, `SE(3)` (`R^N` is the ordinary Euclidean case). The group operations come from the
[`manif` library](https://artivis.github.io/manif/), wrapped behind our own `LieGroup` type so the
GBP code stays group-agnostic.

Its **belief** is a Gaussian: a mean `X_0` and a covariance. The covariance lives in the **tangent
space** at `X_0` (for the Euclidean case this is just the usual covariance). We store the
**precision** (inverse covariance) `lambda` rather than the covariance itself.

You create a variable with a prior mean and, optionally, a prior covariance:

```cpp
//        addVariable(layer, prior_mean, sigma_prior_list)
auto var = fg.addVariable<Variable>(
               layer_id,
               manif::SO2d(0.5),         // The prior mean is a point on the manifold (here we use the SO(2) group)
               Eigen::VectorXd{{0.1}}    // per-dof prior sigma: where each diagonal element in the covariance is sigma^2
);
```

- `prior_mean` is a `LieGroup` — it carries both *which group* the variable lives on and its initial
  value. For a Euclidean variable just pass an `Eigen::VectorXd` (e.g. `LieGroup(Eigen::VectorXd{{x, y}})`).
- `sigma_prior_list` is the per-dof prior standard deviation; the prior precision is
  `diag(sigma_prior_list)^-2`. Leaving it empty sets the prior precision to **zero** — genuinely no
  prior, so the variable is determined entirely by its factors. Such a variable has a singular
  (non-invertible) belief until enough factors constrain it; if it might be under-constrained, give it
  a **weak prior** (a large sigma) instead to keep its belief well-posed.

The prior is built into the variable — there is no separate "prior factor" node.

The belief is available at any time:

```cpp
var->belief_.mu             // the mean: a LieGroup element
var->belief_.mu.coeffs()    // its minimal coordinates ([x, y, theta] for SE2, etc.)
var->belief_.lambda         // the precision (covariance = belief_.lambda.inverse())
```

## Factors

A **Factor** relates two or more variables `X1, X2, …` through a Gaussian **measurement model**, with
cost

```
|| h(X1, X2, …) - z ||^2_Sigma
```

where `h(·)` is the measurement function, `z` the observed measurement, and `Sigma` the measurement
covariance (the cost is the squared Mahalanobis norm under `Sigma`).

For example a **smoothness factor** asks its two variables to have the same value:

```
h(X1, X2) = X1 - X2        z = 0
```

so the cost is minimised when `X1 == X2`. On a Lie group `X1 - X2` is the manifold difference (the
tangent vector between the two elements).

Add a factor into the layer with `addFactor`, passing the keys of the variables it connects followed
by the factor's own constructor arguments:

```cpp
//                              layer    connected variables   group  z      sigma
fg.addFactor<SmoothnessFactor>(layer_id, {a->key_, b->key_}, "R2", zero2, sigma);
```

- the second argument is the list of variable `Key`s the factor connects (here two);
- the rest go to the factor's constructor. For a `SmoothnessFactor` that is the group name, the
  observation `z`, and the per-dof measurement sigma. The **group name** is the space the connected
  variables live on: `"SO2"`, `"SE2"`, `"SO3"`, `"SE3"`, or `"R<n>"` for `R^n` (e.g. `"R2"`).

## Creating a custom factor

A factor is a subclass of `Factor` that implements `computeResidualAndJacobian(X)`, returning the
residual `h(X) - z` and its Jacobian `dh/dX`. The constructor forwards the group name, observation
`z`, and measurement sigma to the base `Factor`.

```cpp
class MyFactor : public Factor {
public:
    MyFactor(Key key, std::vector<Key> vars, float sigma)
        // base Factor args: (key, connected vars, group name, observation z, per-dof sigma)
        : Factor{key, vars, "R2", Eigen::VectorXd::Zero(2), Eigen::VectorXd::Constant(2, sigma)} {}

    std::pair<Eigen::VectorXd, Eigen::MatrixXd>
    computeResidualAndJacobian(const std::vector<LieGroup>& X) override {
        // X[k] is connected variable k's state. X[k].coeffs() are its coordinates.
        Eigen::VectorXd r = /* residual h(X) - z_ */;   // z_ is the base-class observation
        Eigen::MatrixXd J = /* Jacobian dh/dX */;       // rows = measurement dof, cols = total variable dofs
        return { r, J };
    }
};
```

Then add it like any other factor:

```cpp
fg.addFactor<MyFactor>(layer_id, {a->key_, b->key_}, sigma);
```

Notes:

- If you don't have a closed-form Jacobian, use `jacobianFirstOrder(X, h)`, where `h` returns just the
  residual. It perturbs each variable on the manifold and finite-differences. A common pattern is a
  separate `computeResidual(X)` helper that both `computeResidualAndJacobian` and the finite-difference
  path call (see the planning factors in [`Pathplanning.cpp`](../src/CustomFactorGraphLayers/Pathplanning.cpp)).
- `X[k]` is a `LieGroup`. Useful operations: `coeffs()` (minimal coordinates), `ominus(other, J_self, J_other)`
  (the manifold difference `X[k] - other` with analytic Jacobians as out-parameters), and `operator*`,
  `inverse()`, `Ad()`, `log()`.
- Override `skipFactor()` to return `true` when the factor should contribute nothing in some state
  (the inter-robot collision factor uses this beyond a safety distance).

`SmoothnessFactor` is the simplest worked example: its residual is `X[1] - X[0]`, and its exact
Jacobian comes straight from `ominus`'s out-parameters. See [`Factor.cpp`](../src/gbp/Factor.cpp).

## Solving the graph

One GBP *iteration* updates every factor's outgoing messages, then every variable's belief from its
incoming messages. Iterating lets information propagate along the graph:

```cpp
fg.optimiseGBP(5);   // run 5 GBP iterations
```

## How it's organised

Variables and Factors live inside a **FactorGraphLayer**, and one or more layers make up a
**FactorGraph**:

```
{ Variable, Factor }  ⊂  FactorGraphLayer  ⊂  FactorGraph  ⊂  FactorGraphGroup
```

- **FactorGraphLayer** — the variables and factors of a single sub-problem.
- **FactorGraph** — a stack of layers. Optimising the whole problem often means optimising several
  distinct sub-problems together (e.g. *path planning* and *consensus* as separate layers) while
  sharing information between them.
- A **Robot** is a wrapper around a FactorGraph (it derives from `FactorGraph` and adds the
  robot-specific state and behaviour).
- **FactorGraphGroup** — many Robots optimised together. Each robot solves its own graph locally, and
  the group routes the messages exchanged *between* robots (inter-robot collision-avoidance or
  consensus factors) each sweep. There is no central solver — coordination emerges from these local
  message exchanges.

Every Variable and Factor is identified by a **`Key`** encoding *where it lives*: the graph id (which
robot), the layer id, a node id, and whether it is a variable or a factor. This is what lets a factor
on one robot refer to a variable on another. A Key prints as `[R_<graph>|L_<layer>|V_<id>]` (or
`F_<id>` for a factor) — for example `[R_0|L_1|V_2]` is variable 2 in layer 1 of robot 0.

## See also

- [`src/examples/gbp-two-layer-factorgraph.cpp`](../src/examples/gbp-two-layer-factorgraph.cpp) — minimal standalone smoothing example.
- [`src/examples/gbp-1d-line-fitting.cpp`](../src/examples/gbp-1d-line-fitting.cpp) — interactive line fitting with robust kernels.
- [`inc/CustomFactorGraphLayers/ExampleLayer.h`](../inc/CustomFactorGraphLayers/ExampleLayer.h) — a copy-me template for a custom layer, factor, and variable.
- [A micro Lie theory for state estimation in robotics](https://arxiv.org/abs/1812.01537) — a great introduction to Lie groups.
