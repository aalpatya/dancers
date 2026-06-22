/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#pragma once
#include <gbp/GBPCore.h>
#include <gbp/Factor.h>
#include <gbp/Variable.h>
#include <functional>
#include <memory>
#include <map>
#include <vector>
#include <string>
#include <cctype>
#include <stdexcept>

// Forward decl to break the include cycle (FactorGraph includes this header). Hooks take it by reference.
class FactorGraph;

// ParamBag is a typed key/value bag for one layer's params, filled from the layer's `params:` block in
// the config and read by the layer. LayerConfig is one configured layer instance.
struct ParamBag {
    std::map<std::string, double>      nums;   // numbers and bools (bools as 0/1)
    std::map<std::string, std::string> strs;

    double num(const std::string& k, double d) const { auto it = nums.find(k); return it!=nums.end() ? it->second : d; }
    float  numf(const std::string& k, float d) const { return (float)num(k, (double)d); }
    int    numi(const std::string& k, int d)   const { return (int)num(k, (double)d); }
    std::string str(const std::string& k, const std::string& d) const { auto it = strs.find(k); return it!=strs.end() ? it->second : d; }
};

// One configured layer instance. `name` is the user's label, `type` selects the layer class
// (mapped in makeLayer(), src/Robot.cpp), `params` is its bag.
struct LayerConfig {
    std::string name;
    std::string type;
    ParamBag    params;
};

// The configured layer stack. A layer's id is its index here, the same for every graph. Populated
// once from the config by Globals.cpp.
inline std::vector<LayerConfig>& layerConfigs(){
    static std::vector<LayerConfig> cfgs;
    return cfgs;
}
// Index of the first configured layer of the given type ("planning"/"consensus"/...) or name, or -1.
inline int layerIdOfType(std::string type){
    for (auto& c : type) c = (char)std::tolower((unsigned char)c);
    for (int i=0; i<(int)layerConfigs().size(); i++) if (layerConfigs()[i].type==type) return i;
    return -1;
}

// One layer of a FactorGraph: owns its variables_/factors_, takes part in GBP message passing
// (factorIteration/variableIteration), and builds/updates its nodes through the lifecycle hooks below.
// The hooks default to no-ops, so a plain FactorGraphLayer works as a hand-filled node container; a
// custom layer subclasses it and overrides the hooks it needs.
//
// Each hook receives the owning FactorGraph (the layer lives in owner.stack_). The gbp library is
// application-agnostic; an app stack subclass (e.g. Robot) recovers itself with
//     Robot* robot = static_cast<Robot*>(&owner);
// Drawing lives on the app-side DrawableLayer interface, since it needs the rendering context.
class FactorGraphLayer {
public:
    int graph_id_;                                          // id of the graph this layer belongs to
    int lid_;                                               // this layer's id (its index in the stack)
    std::string name_;                                      // human-readable label (from cfg)
    LayerConfig cfg_;                                       // read params via cfg_.params
    std::map<Key, std::shared_ptr<Variable>> variables_{};
    std::map<Key, std::shared_ptr<Factor>> factors_{};
    std::map<std::pair<Key, Key>, Message>* p_inbox_  = nullptr;   // wired by the owning FactorGraph
    std::map<int, std::map<std::pair<Key, Key>, Message>>* p_outbox_ = nullptr;

    FactorGraphLayer(int graph_id, int lid = -1, const LayerConfig& cfg = {})
        : graph_id_(graph_id), lid_(lid), name_(cfg.name), cfg_(cfg) {}
    virtual ~FactorGraphLayer() = default;

    // ---- GBP ----
    // A factor iteration recomputes a factor's outgoing messages; a variable iteration updates a
    // variable's belief and its outgoing messages. The no-arg overloads sweep every node (synchronous
    // GBP); the single-key overloads do just one node, the building blocks for asynchronous GBP (update
    // one node, or one local neighbourhood, at a time). See iterateFactor/iterateVariable for the body.
    void factorIteration(){ for (auto& [f_key, fac] : factors_) iterateFactor(f_key, fac); }
    void variableIteration(){ for (auto& [v_key, var] : variables_) iterateVariable(v_key, var); }
    void factorIteration(const Key& f_key){ if (auto it = factors_.find(f_key); it != factors_.end()) iterateFactor(it->first, it->second); }
    void variableIteration(const Key& v_key){ if (auto it = variables_.find(v_key); it != variables_.end()) iterateVariable(it->first, it->second); }
    // Run num_iters GBP sweeps over just this layer (one factor then one variable iteration per sweep).
    void optimiseGBP(int num_iters){
        for (int i=0; i<num_iters; i++){
            factorIteration();
            variableIteration();
        }
    }
    // First factor shared by v1 and v2, or {-1,-1} if none.
    Key findCommonFactor(std::shared_ptr<Variable> v1, std::shared_ptr<Variable> v2){
        for (auto fk1 : v1->connected_f_keys_)
            for (auto fk2 : v2->connected_f_keys_)
                if (fk1==fk2) return fk1;
        return Key{-1, -1};
    }

    // ---- Node builders. The FactorGraph's addVariable/addFactor assign ids then call these; you can
    // also call them directly with a Key you built. Add a node to this layer (registering a factor with
    // its connected variables that live here) and return it for chaining. ----
    std::shared_ptr<Variable> addVariable(std::shared_ptr<Variable> v){ variables_[v->key_] = v; return v; }
    std::shared_ptr<Factor>   addFactor(std::shared_ptr<Factor> f){
        factors_[f->key_] = f;
        for (auto& vk : f->connected_v_keys_)
            if (auto it = variables_.find(vk); it != variables_.end()) it->second->addFactor(f->key_);
        return f;
    }

    // ---- Lifecycle hooks: override the ones your layer needs; defaults are no-ops. ----
    virtual void initialiseLayerNodes(FactorGraph& /*owner*/) {}                 // build nodes (once)
    virtual void createInterrobotFactors(FactorGraph& /*owner*/, int /*other_id*/) {} // cross-graph factors
    virtual void preGBPUpdateNodes(FactorGraph& /*owner*/, uint32_t /*clock*/) {}  // each step, before GBP
    virtual void postGBPUpdateNodes(FactorGraph& /*owner*/) {}                    // each step, after GBP

private:
    // One factor's update: gather its variables' messages (local from their outboxes, external from the
    // shared inbox), recompute the factor, then copy any external-bound messages to the outbox.
    void iterateFactor(const Key& f_key, const std::shared_ptr<Factor>& fac){
        for (auto vk : fac->connected_v_keys_){
            if (vk.graph_id_ != f_key.graph_id_){
                if (p_inbox_->count(std::make_pair(f_key, vk))) fac->inbox_[vk] = p_inbox_->at(std::make_pair(f_key, vk));
                // No external message yet (e.g. factor just created): a zero-information message in the
                // variable's group, so it contributes nothing until the real one arrives.
                else fac->inbox_[vk] = Message(fac->groupForKey(vk));
            } else {
                fac->inbox_[vk] = variables_.at(vk)->outbox_.at(f_key);
            }
        }
        fac->updateFactor();
        for (auto vk : fac->connected_v_keys_)
            if (vk.graph_id_ != f_key.graph_id_)
                (*p_outbox_)[vk.graph_id_][std::make_pair(vk, f_key)] = fac->outbox_.at(vk);
    }
    // One variable's update: gather its factors' messages (local from their outboxes, external from the
    // shared inbox), update the belief, then copy any external-bound messages to the outbox.
    void iterateVariable(const Key& v_key, const std::shared_ptr<Variable>& var){
        for (auto fk : var->connected_f_keys_){
            if (fk.graph_id_ != v_key.graph_id_){
                if (p_inbox_->count(std::make_pair(v_key, fk))) var->inbox_[fk] = p_inbox_->at(std::make_pair(v_key, fk));
                else print("Error: no message in inbox found at variableIteration");
            } else {
                var->inbox_[fk] = factors_.at(fk)->outbox_.at(v_key);
            }
        }
        var->updateBelief();
        for (auto fk : var->connected_f_keys_)
            if (fk.graph_id_ != v_key.graph_id_)
                (*p_outbox_)[fk.graph_id_][std::make_pair(fk, v_key)] = var->outbox_.at(fk);
    }
};

// Concrete layers live in inc|src/CustomFactorGraphLayers/. makeLayer() in src/Robot.cpp maps the config's
// `type:` string to a class; add yours there. See ExampleLayer.h for a copy-me template.
