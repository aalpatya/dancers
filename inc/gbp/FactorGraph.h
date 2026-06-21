/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#pragma once
#include <memory>
#include <map>

#include <Utils.h>
#include <gbp/GBPCore.h>
#include <gbp/Factor.h>
#include <gbp/Variable.h>
#include <gbp/FactorGraphLayer.h>

#include <Eigen/Dense>
#include <Eigen/Core>

/******************************************************************************************************/
// A stack of FactorGraphLayers, each a GBP sub-problem with its own variables and factors. Layers
// share the graph's inter-graph mailboxes, so a factor in one graph can reach a variable in another
// (e.g. inter-robot factors). The application's per-entity object (e.g. Robot) derives from
// FactorGraph; many of them optimise together in a FactorGraphGroup.
//   Group  >  FactorGraph  >  FactorGraphLayer  >  { Variable, Factor }
/******************************************************************************************************/
class FactorGraph {
    public:
    int graph_id_;                                          // id of this graph
    std::vector<int> layers_;                           // layer ids, in stack order
    std::map<int, std::shared_ptr<FactorGraphLayer>> stack_ {};
    std::map<int, int32_t> next_vids_{};               // next variable node-id per layer
    std::map<int, int32_t> next_fids_{};               // next factor node-id per layer
    std::map<std::pair<Key, Key>, Message> inbox_{};        // [to_key, from_key], msg
    std::map<int, std::map<std::pair<Key, Key>, Message>> outbox_{};  // [to_key, from_key], msg
    std::map<std::pair<Key, Key>, Message> tempmailbox_{};  // [to_key, from_key], msg

    bool interrobot_comms_active_ = true;                   // whether this graph talks to other graphs

    FactorGraph(int graph_id = 0) : graph_id_(graph_id) {}
    // Adopt pre-built layers (e.g. assembled by the Simulator from the configured stack), keyed by lid_.
    FactorGraph(int graph_id, std::vector<std::shared_ptr<FactorGraphLayer>> layers) : graph_id_(graph_id) {
        for (auto& lyr : layers) adopt(lyr);
    }
    virtual ~FactorGraph() = default;

    bool hasLayer(int layer) const { return stack_.count(layer)>0; }

    // ---- Building the stack ---------------------------------------------------------------------
    // Adopt one layer: wire up its mailbox pointers and register it under its id, in stack order.
    void adopt(std::shared_ptr<FactorGraphLayer> lyr){
        lyr->p_inbox_  = &inbox_;
        lyr->p_outbox_ = &outbox_;
        stack_[lyr->lid_] = lyr;
        layers_.push_back(lyr->lid_);
        next_vids_[lyr->lid_]; next_fids_[lyr->lid_];   // default-construct the counters to 0
    }
    // Push a new layer of type LayerT onto the top of the stack (its id is its position); returns it.
    template<class LayerT = FactorGraphLayer>
    std::shared_ptr<LayerT> addLayer(const std::string& name = "", LayerConfig cfg = {}){
        if (!name.empty()) cfg.name = name;
        auto lyr = std::make_shared<LayerT>(graph_id_, (int)layers_.size(), cfg);
        adopt(lyr);
        return lyr;
    }
    FactorGraphLayer& layer(int id){ return *stack_.at(id); }
    FactorGraphLayer& layer(const std::string& name){
        for (auto& [id, l] : stack_) if (l->name_==name) return *l;
        throw std::out_of_range("FactorGraph::layer: no layer named '" + name + "'");
    }

    // ---- Adding nodes (id assigned automatically from this graph's per-layer counter) -----------
    template<class VarT, class... Args>
    std::shared_ptr<VarT> addVariable(int layer, Args&&... args){
        Key k(graph_id_, next_vids_[layer]++, layer, Node::VAR);
        auto v = std::make_shared<VarT>(k, std::forward<Args>(args)...);
        stack_.at(layer)->addVariable(v);
        return v;
    }
    template<class FacT, class... Args>
    std::shared_ptr<FacT> addFactor(int layer, std::vector<Key> connected, Args&&... args){
        Key k(graph_id_, next_fids_[layer]++, layer, Node::FAC);
        auto f = std::make_shared<FacT>(k, connected, std::forward<Args>(args)...);
        stack_.at(layer)->addFactor(f);
        return f;
    }

    // ---- Layer lifecycle: the graph drives each layer through its hooks, passing itself as the owner.
    // Concrete layers recover their application type (e.g. Robot) from the owner; see FactorGraphLayer.h.
    void initialiseLayers(){                    // build every layer's nodes (once, at construction)
        for (auto& [lyr, fg] : stack_) fg->initialiseLayerNodes(*this);
    }
    void preGBP(uint32_t clock){                // per-timestep, before the GBP iterations
        for (auto& [lyr, fg] : stack_) fg->preGBPUpdateNodes(*this, clock);
    }
    void postGBP(){                             // per-timestep, after the GBP iterations
        for (auto& [lyr, fg] : stack_) fg->postGBPUpdateNodes(*this);
    }
    void makeInterrobotFactors(int other_id){   // build this graph's inter-graph factors to other_id
        for (auto& [lyr, fg] : stack_) fg->createInterrobotFactors(*this, other_id);
    }
    // Drop every factor in this stack that connects to other_id, and unregister this stack's variables
    // from that robot's external factors.
    void deleteInterrobotFactors(int other_id){ // remove all factors connecting this graph to other_id
        for (auto layer : layers_){
            std::vector<Key> facs_to_delete{};
            for (auto& [f_key, fac] : stack_[layer]->factors_){
                if (fac->other_rid_ != other_id) continue;
                facs_to_delete.push_back(f_key);
            }
            for (auto f_key : facs_to_delete) deleteFactor(f_key);

            for (auto& [v_key, var] : stack_[layer]->variables_){
                std::vector<Key> fkeys = var->connected_f_keys_;   // copy: deleteFactor() erases from it
                for (auto fk : fkeys){
                    if (fk.graph_id_ == other_id){
                        inbox_.erase(std::make_pair(v_key, fk));   // drop its inbox entry
                        var->deleteFactor(fk);
                    }
                }
            }
        }
    }

    // Run GBP on this graph: num_iters sweeps of (update all factors, then all variables) over every
    // layer. The single-graph counterpart to FactorGraphGroup::optimiseGBP(), which also exchanges
    // inter-graph messages each sweep.
    void optimiseGBP(int num_iters){
        for (int i=0; i<num_iters; i++){
            for (auto& [lid, layer] : stack_) layer->factorIteration();
            for (auto& [lid, layer] : stack_) layer->variableIteration();
        }
    }

    // Access the i'th variable within a layer (negative i counts from the end, modular):
    //   auto variable = graph.getVar(layer, i);
    std::shared_ptr<Variable>& getVar(const int& layer, const int& i){
        // find() + check rather than operator[], which would silently insert a null layer and then
        // segfault on deref.
        auto sit = stack_.find(layer);
        if (sit==stack_.end() || !sit->second){
            print("ERROR! getVar: layer is not present in this graph's stack");
            throw std::out_of_range("getVar: requested a layer that is not present in this graph's stack");
        }
        auto& fg = sit->second;
        int n = fg->variables_.size();
        int search_vid = ((n + i) % n + n) % n;
        auto it = fg->variables_.begin();
        std::advance(it, search_vid);
        return it->second;
    }

    bool varExists(const Key& v_key){ return stack_.at(v_key.lid_)->variables_.count(v_key); }
    bool facExists(const Key& f_key){ return stack_.at(f_key.lid_)->factors_.count(f_key); }

    // Access the variable by a specific key
    std::shared_ptr<Variable>& getVar(const Key& v_key){
        if (v_key.node_type_!=Node::VAR){
            print("ERROR! Asked for Var, but is a Fac");
        }
        auto& variables = stack_.at(v_key.lid_)->variables_;
        auto it = variables.find(v_key);   // one lookup on the common path
        if (it==variables.end()){
            print("ERROR! Variable does not exist");
            return variables.at(v_key);    // throws out_of_range
        }
        return it->second;
    }
    // Access the factor by a specific key
    std::shared_ptr<Factor>& getFac(const Key& f_key){
        if (f_key.node_type_!=Node::FAC){
            print("ERROR! Asked for Fac, but is a Var");
        }
        auto& factors = stack_.at(f_key.lid_)->factors_;
        auto it = factors.find(f_key);     // one lookup on the common path
        if (it==factors.end()){
            print("ERROR! Factor does not exist");
            return factors.at(f_key);      // throws out_of_range
        }
        return it->second;
    }

    void deleteFactor(Key fkey){
        auto fac = getFac(fkey);
        for (auto vk : fac->connected_v_keys_){
            if (vk.graph_id_!=fkey.graph_id_){                  // If factor is connected to an external variable:
                inbox_.erase(std::make_pair(fkey, vk));         // remove its entry from the graph inbox
                outbox_[vk.graph_id_][std::make_pair(vk, fkey)] = Message(MSG_TYPE::DELETION); // indicates deleted fkey
            } else {
                // Unregister the factor with each of its connected variables in the same graph
                getVar(vk)->deleteFactor(fkey);
            }
        }
        stack_.at(fkey.lid_)->factors_.erase(fkey);    // Delete the factor from the same graph as itself
    }

    void deleteVariable(Key vkey){
        auto var = getVar(vkey);
        std::vector<Key> facs_to_delete{};
        for (auto fk : var->connected_f_keys_){
            if (fk.graph_id_!=vkey.graph_id_){                  // If variable is connected to an external factor:
                inbox_.erase(std::make_pair(vkey, fk));         // remove its entry from the graph inbox
                outbox_[fk.graph_id_][std::make_pair(fk, vkey)] = Message(MSG_TYPE::DELETION); // Indicates deleted vkey
            } else {
                facs_to_delete.push_back(fk);   // local factor: deleted below (also unregisters its variables)
            }
        }
        for (auto fk : facs_to_delete) deleteFactor(fk);
        stack_.at(vkey.lid_)->variables_.erase(vkey);      // Delete the variable from the graph
    }
};

/******************************************************************************************************/
// A set of FactorGraphs optimised together (e.g. the simulator's robots). References the caller's map,
// it does not own it. Runs the multi-graph GBP loop: route inter-graph messages
// (outbox->tempmailbox->inbox) and sweep every graph's layers. Templated on the graph type so callers
// can keep concrete handles (FactorGraphGroup<Robot>) while the group itself only uses the base.
/******************************************************************************************************/
template <class GraphT = FactorGraph>
class FactorGraphGroup {
public:
    std::map<int, std::shared_ptr<GraphT>>& graphs_;   // the participating graphs, keyed by id
    std::vector<int>& layers_;                     // layer ids to iterate, in order
    uint32_t clock_ = 0;                               // timestep counter (kept in sync by the owner)

    FactorGraphGroup(std::map<int, std::shared_ptr<GraphT>>& graphs, std::vector<int>& layers)
        : graphs_(graphs), layers_(layers) {}

    // One factor (resp. variable) update sweep across every graph, per layer, in parallel over graphs.
    // Snapshot into a vector first so the parallel loop can index directly (walking the map would be O(n^2)).
    void factorIteration(){
        std::vector<FactorGraph*> g; g.reserve(graphs_.size());
        for (auto& [id, gr] : graphs_) g.push_back(gr.get());
        for (auto layer : layers_){
#pragma omp parallel for
            for (int i=0; i<(int)g.size(); i++) g[i]->stack_[layer]->factorIteration();
        }
    }
    void variableIteration(){
        std::vector<FactorGraph*> g; g.reserve(graphs_.size());
        for (auto& [id, gr] : graphs_) g.push_back(gr.get());
        for (auto layer : layers_){
#pragma omp parallel for
            for (int i=0; i<(int)g.size(); i++) g[i]->stack_[layer]->variableIteration();
        }
    }

    // Route inter-graph messages: each graph's outbox -> recipient's tempmailbox, then drain every
    // tempmailbox into the graph's inbox, registering or deleting external factors and variables as told.
    void exchangeMessages(){
        for (auto& [id, gr] : graphs_){
            if (!gr->interrobot_comms_active_) continue;
            for (auto& [to_id, content] : gr->outbox_){
                auto rit = graphs_.find(to_id);
                if (rit==graphs_.end()) continue;
                for (auto& [keypair, msg] : content) rit->second->tempmailbox_[keypair] = msg;
            }
            gr->outbox_.clear();
        }
        for (auto& [id, gr] : graphs_){
            for (auto& [keypair, msg] : gr->tempmailbox_){
                if (keypair.first.node_type_==Node::VAR){            // factor -> variable message
                    auto& [vk, fk] = keypair;
                    if (!gr->varExists(vk)) continue;
                    if (msg.isDeleteInstruction()){ gr->getVar(vk)->deleteFactor(fk); continue; }
                    if (!gr->inbox_.count(keypair)) gr->getVar(vk)->addFactor(fk);   // register new external factor
                } else {                                             // variable -> factor message
                    auto& [fk, vk] = keypair;
                    if (!gr->facExists(fk)) continue;
                    if (msg.isDeleteInstruction()){ gr->deleteFactor(fk); continue; }
                }
                gr->inbox_[keypair] = msg;
            }
            gr->tempmailbox_.clear();
        }
    }

    // Full GBP across all graphs: num_iters_external rounds of (exchange external messages, then 2 internal
    // sweeps). The single-graph equivalent is FactorGraph::optimiseGBP().
    void optimiseGBP(int num_iters_external){
        int num_iters_internal = 2;
        for (int i=0; i<num_iters_external; i++){
            exchangeMessages();
            for (int ii=0; ii<num_iters_internal; ii++){
                factorIteration();
                variableIteration();
            }
        }
    }

    // Per-timestep layer hooks across every graph, around the GBP iterations. preGBP runs sequentially
    // (layers may add or remove nodes here) and uses the group's clock_, which the owner keeps in sync.
    // postGBP only reads back, so it runs in parallel.
    void preGBP(){ for (auto& [id, gr] : graphs_) gr->preGBP(clock_); }
    void postGBP(){
        std::vector<FactorGraph*> g; g.reserve(graphs_.size());
        for (auto& [id, gr] : graphs_) g.push_back(gr.get());
#pragma omp parallel for
        for (int i=0; i<(int)g.size(); i++) g[i]->postGBP();
    }
};
