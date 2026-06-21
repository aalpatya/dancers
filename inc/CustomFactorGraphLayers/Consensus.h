/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#pragma once
#include <gbp/FactorGraphLayer.h>
#include <CustomFactorGraphLayers/DrawableLayer.h>

// This layer uses the built-in Variable and SmoothnessFactor from gbp/. For a
// factor or variable specific to a layer, subclass Factor/Variable (see Pathplanning.h).

/******************************************************************************************************/
// Consensus layer: the formation/decision variable(s) with a prior, sliding-window smoothness factors,
// and inter-robot smoothness factors used for consensus.
/******************************************************************************************************/
class ConsensusLayer : public FactorGraphLayer, public DrawableLayer {
public:
    // Layer-private tuning, read straight from this layer's config param bag.
    float SIGMA_CONSENSUS_FACTOR_SMOOTHNESS, SIGMA_CONSENSUS_VAR_PRIOR, SIGMA_CONSENSUS_VAR_SLIDING_WINDOW;
    int   NUM_CONSENSUS_VAR_SLIDING_WINDOW, SLIDING_WINDOW_TIMESTEP_FREQ;
    // The Lie group the consensus state lives in, from this layer's config (discrete -> R1;
    // continuous -> the SPACE param).
    LieGroup lieGroup_;

    ConsensusLayer(int graph_id, int lid, const LayerConfig& cfg = {}) : FactorGraphLayer(graph_id, lid, cfg) {
        SIGMA_CONSENSUS_FACTOR_SMOOTHNESS  = cfg.params.numf("SIGMA_FACTOR_SMOOTHNESS",      0.05f);
        SIGMA_CONSENSUS_VAR_PRIOR          = cfg.params.numf("SIGMA_VAR_PRIOR",              1.0f);
        SIGMA_CONSENSUS_VAR_SLIDING_WINDOW = cfg.params.numf("SIGMA_VAR_SLIDING_WINDOW",     0.1f);
        NUM_CONSENSUS_VAR_SLIDING_WINDOW   = cfg.params.numi("NUM_VAR_SLIDING_WINDOW",       1);
        SLIDING_WINDOW_TIMESTEP_FREQ       = cfg.params.numi("SLIDING_WINDOW_TIMESTEP_FREQ", 1);
        lieGroup_ = (cfg.params.str("TYPE", "continuous")=="discrete")
                        ? makeLieGroup("R1") : makeLieGroup(cfg.params.str("SPACE", "SE2"));
    }
    void initialiseLayerNodes(FactorGraph& owner) override;
    void createInterrobotFactors(FactorGraph& owner, int other_id) override;
    void preGBPUpdateNodes(FactorGraph& owner, uint32_t clock) override;   // advances the sliding window
    void postGBPUpdateNodes(FactorGraph& owner) override;
    void draw(Robot* robot, Simulator* sim, Color& col) override;
};

// Map a continuous value in [0,1) to one of n_bins discrete decisions, and back (discrete consensus).
inline int quantise(double X_continuous, int n_bins) {
    double X_clipped = std::max(std::min(X_continuous, 0.9999), 0.);
    return (int)round(X_clipped * n_bins);
}
inline double quantiseInv(int X_discrete, int n_bins) {
    return X_discrete / (double)n_bins;
}

inline int angle2decision(double anglerads, double num_decisions){
    double rounded = std::floor(anglerads / (1.*M_PI/num_decisions));
    rounded = fmod(fmod(rounded, num_decisions) + num_decisions, num_decisions);
    return (int)rounded;
}
inline double decision2angle(int decision_integer, double num_decisions){
    return decision_integer * (1.*M_PI / num_decisions);        
}
inline double normalisedDecision2angle(double normalised_decision){
    return normalised_decision * 1.*M_PI;
}