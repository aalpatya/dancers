/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
// Minimal, self-contained example of building a problem with the GBP library.
//
// We build ONE FactorGraph containing TWO layers and solve them with Gaussian Belief Propagation:
//
//     Layer "lid_euclidean":  variables live in R^3   (a point  [x, y, z])
//     Layer "lid_SE2":        variables live in SE2   (a pose   [x, y, theta])
//
// In each layer the variables are connected in a chain by smoothness factors (each pulls its two
// variables to be equal). The two ends of each chain are pinned by priors, so GBP makes the chain
// interpolate smoothly between the pinned ends:
//
//        [prior]                                                         [prior]
//           |                                                               |
//          (x0)--[smooth]--(x1)--[smooth]--(x2)--[smooth]--(x3)--[smooth]--(x4)
//
//   legend:   (xN) a variable    [smooth] a binary smoothness factor    [prior] a unary anchor
//
// HOW TO BUILD YOUR OWN PROBLEM (the whole recipe is in main() below):
//   1. FactorGraph fg;                              // the graph
//   2. int lid = fg.addLayer("name")->lid_;  // add as many layers as you like
//   3. fg.addVariable<VarType>(lid, ...ctor args...); // variables  (a Key is assigned for you)
//   4. fg.addFactor<FactorType>(lid, {keys}, ...);    // factors connecting those variables
//   5. fg.optimiseGBP(n) to run n GBP sweeps until converged; read each variable's belief_.mu.
// To add a CUSTOM variable or factor, subclass Variable / Factor (see inc/gbp + the ExampleLayer).
//
// Build & run:   make -C build gbp-two-layer-factorgraph  &&  ./build/examples/gbp-two-layer-factorgraph
/**************************************************************************************/
#include <iostream>
#include <sstream>
#include <vector>

#include <gbp/FactorGraph.h>
#include <gbp/Variable.h>
#include <gbp/Factor.h>
#include <manif/SE2.h>

int main(){
    std::cout.precision(3);   // print everything to 3 significant figures
    FactorGraph fg(/*id=*/0);
    // Two layers. A layer is just a container of variables + factors; its id is its position.
    int lid_euclidean = fg.addLayer("Euclidean")->lid_;   // R^3 points
    int lid_SE2       = fg.addLayer("SE2")->lid_;          // SE2 poses

    const int N = 5;                                  // variables per chain
    Eigen::VectorXd strong_sigma = Eigen::VectorXd::Constant(3, 1e-3);   // prior sigma (small = strong anchor)
    Eigen::VectorXd weak_sigma = Eigen::VectorXd::Constant(3, 0.5);    // smoothness sigma
    Eigen::VectorXd zero_sigma = Eigen::VectorXd::Constant(3, 0.);    // smoothness sigma
    Eigen::VectorXd zero3vec = Eigen::VectorXd::Zero(3);            // smoothness "should be equal" target
    
    print("Pre GBP beliefs:");
    // ---- Layer 1: a chain of R^3 points, ends pinned to (0,0,0) and (8,4,2). ----
    // Each node takes the group it lives in as its first argument. For R^n that is any Eigen vector
    // of dimension n (here zero3vec, size 3); it converts to a LieGroup implicitly.
    {
        Eigen::VectorXd start{{0., 0., 0.}};    // Prior mean of first variable
        Eigen::VectorXd end{{8., 4., 2.}};      // Prior mean of last variable
        print(fg.layer(lid_euclidean).name_ + " layer - priors: first", start.transpose(), "last", end.transpose());

        // Create variables, assigning strong prior covariances (through sigma) to the first and last variable
        // The first and last variables are therefore 'pinned' to their priors.
        // The intermediate variables are created with zero sigma.
        std::vector<Key> k; // A list of keys that we will populate with our newly created variables
        for (int i = 0; i < N; i++){
            Eigen::VectorXd mu    = (i==0) ? start : (i==N-1) ? end : zero3vec;   // R^3 group value = the mean
            Eigen::VectorXd sigma = (i==0 || i==N-1) ? strong_sigma : zero_sigma;
            k.push_back(fg.addVariable<Variable>(lid_euclidean, mu, sigma)->key_);
        }
        // Add smoothness factors between consecutive variables: we need their keys to do this.
        // We also pass in the Lie Group type. In this layer it's just Euclidean R3.
        for (int i = 0; i < N-1; i++)
        fg.addFactor<SmoothnessFactor>(lid_euclidean, {k[i], k[i+1]}, "R3", zero3vec, weak_sigma);
    }
    
    // ---- Layer 2: a chain of SE2 poses, ends pinned to (0,0,0deg) and (5,5,90deg). ----
    // The group here is SE2; a manif::SE2d value passes straight in (it converts to a LieGroup).
    {
        manif::SE2d start(0., 0., 0.);
        manif::SE2d end(5., 5., M_PI_2);
        manif::SE2d middle = manif::SE2d::Identity();
        print(fg.layer(lid_SE2).name_ + " layer - priors: first", start.translation().transpose(), start.angle(), "last", end.translation().transpose(), end.angle());
        // Create variables, assigning strong prior covariances (through sigma) to the first and last variable
        // The first and last variables are therefore 'pinned' to their priors.
        // The intermediate variables are created with zero sigma.
        std::vector<Key> k; // A list of keys that we will populate with our newly created variables
        for (int i = 0; i < N; i++){
            manif::SE2d mu    = (i==0) ? start : (i==N-1) ? end : middle;
            Eigen::VectorXd sigma = (i==0 || i==N-1) ? strong_sigma : Eigen::VectorXd();
            k.push_back(fg.addVariable<Variable>(lid_SE2, manif::SE2d(mu), sigma)->key_);
        }
        // Add smoothness factors between consecutive variables: we need their keys to do this.
        // We also pass in the Lie Group type. In this layer it's SE(2) group.
        for (int i = 0; i < N-1; i++)
            fg.addFactor<SmoothnessFactor>(lid_SE2, {k[i], k[i+1]}, "SE2", zero3vec, weak_sigma);
    }
    print("");
    // ---- Solve: run 5 GBP sweeps over the whole graph (all layers). ----
    fg.optimiseGBP(5);
    
    print("Post GBP beliefs:");
    // ---- Read out each variable's converged belief mean (belief_.mu). The chains interpolate the ends. ----
    for (int L : fg.layers_){
        print(fg.layer(L).name_ + ":");
        for (auto& [key, var] : fg.layer(L).variables_){
            print(key,": μ =" , var->belief_.mu.coeffs().transpose());
        }
        print("");
    }

    return 0;
}
