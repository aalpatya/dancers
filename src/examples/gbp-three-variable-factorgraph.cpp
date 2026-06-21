/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
// Smallest possible GBP chain, showing the built-in variable prior on a Lie group.
//
// Three SO(2) rotations joined by smoothness factors. The two ends carry a prior built straight into
// the variable (its constructor takes a prior mean + sigma); the middle one is free. GBP makes the
// chain interpolate between the pinned ends, settling at 0 / 45 / 90 deg:
//
//        [prior 0deg]                                     [prior 90deg]
//             |                                                 |
//            (a)-------[smooth]-------(b)-------[smooth]-------(c)
//
// The point: no separate prior factor is needed - the prior lives in the variable, and updateBelief()
// applies it on the manifold before folding in the messages.
//
// Build & run:   make -C build gbp-three-variable-factorgraph  &&  ./build/examples/gbp-three-variable-factorgraph
/**************************************************************************************/
#include <gbp/FactorGraph.h>
#include <gbp/Variable.h>
#include <gbp/Factor.h>
#include <manif/SO2.h>

int main(){
    std::cout.precision(3);
    FactorGraph fg(0);                          // Create a factor graph with graph_id = 0
    int lid = fg.addLayer("SO2 chain")->lid_;   // Add a factor graph layer, the layer id is lid

    Eigen::VectorXd strong_sigma = Eigen::VectorXd::Constant(1, 1e-5);  // prior sigma (small = strong_sigma anchor)
    Eigen::VectorXd weak_sigma   = Eigen::VectorXd::Constant(1, 0.5);   // smoothness sigma
    
    // Each variable's group value IS its prior mean / initial estimate. The ends also pass a sigma (a
    // built-in prior); the middle passes none (free to move).
    std::vector<Key> added_variables{
        fg.addVariable<Variable>(lid, manif::SO2d(0.0),    strong_sigma)->key_,
        fg.addVariable<Variable>(lid, manif::SO2d(0.0))->key_,
        fg.addVariable<Variable>(lid, manif::SO2d(M_PI_2), strong_sigma)->key_
    };
    int N = added_variables.size();
    // The factor residual = h(X1, X2) - z = X1 - X2 - z, and the cost function to minimise is the squared residual. 
    // So z is the ideal difference between the two variables
    Eigen::VectorXd z{{0.}};        
    for (int i=0; i<N-1; i++){
        fg.addFactor<SmoothnessFactor>(lid, {added_variables[i], added_variables[i+1]}, "SO2", z, weak_sigma);
    }

    print("Pre GBP beliefs:");
    for (auto& [k, v] : fg.layer(lid).variables_){
        print(" ", k, ": μ =", RAD2DEG * v->belief_.mu.coeffs(), "°, σ =", RAD2DEG * sqrt(v->belief_.lambda.inverse()(0,0)), "°");
    }
    
    // Perform 2 iterations of GBP
    fg.optimiseGBP(2);
    
    print("Post GBP beliefs:");
    for (auto& [k, v] : fg.layer(lid).variables_){
        print(" ", k, ": μ =", RAD2DEG * v->belief_.mu.coeffs(), "°, σ =", RAD2DEG * sqrt(v->belief_.lambda.inverse()(0,0)), "°");
    };
    return 0;
}
