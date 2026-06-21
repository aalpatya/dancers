/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#pragma once 
#include <memory>
#include <map>
#include <vector>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <gbp/LieGroups.h>   // LieGroup wrapper (gbp depends on this, not on the app's Globals.h)
#include <type_traits>
/*************************************************************************************************************/
// This file contains the core algorithm of Gaussian Belief Propagation
/*************************************************************************************************************/

// Configuration the core reads (set once at startup, see main.cpp).
namespace gbp {
struct Config {
    float  damping = 0.0f;          // GBP message damping in [0,1) (0 = none)
    double robust_threshold = 1.0;  // Mahalanobis distance at which a robust kernel kicks in (all factors)
    // relinearise_threshold is a Geometric tangent-distance threshold for relinearising a factor's lin point
    // (0 = relinearise every sweep).
    // This is an absolute distance in the group's tangent space, so the sensible value is space-dependent:
    // metres for R^n, and a metre/radian mix for SE2/SE3 (ideally per-dof; see Factor::updateLinearisationPoint).
    double relinearise_threshold = 0.0;
};
inline Config& config(){ static Config c; return c; }
}
enum class Node {None, FAC, VAR};

// A factor-graph layer is identified by an integer id = its index in the configured layer stack,
// the same across all robots so cross-robot factor keys line up. A layer id of -1 means "unset".

/******************************************************************/
// This data structure is used to represent both variables and factors
// it includes the id of the robot that the variable/factor belongs to, as well as its own id
// The given operators allow for searching and comparisons using the Key structure
/******************************************************************/
class Key {
    public:
    int32_t graph_id_;
    int lid_;
    int32_t node_id_;
    Node node_type_;
    bool valid_;

    Key(){
        graph_id_ = -1;
        node_id_ = -1;
        lid_ = -1;
        node_type_ = Node::None;
        valid_ = false;
    }

    Key(int graph_id, int node_id, int lid=-1, Node node_type=Node::None, bool valid=true)
        : graph_id_(graph_id), lid_(lid), node_id_(node_id), node_type_(node_type), valid_(valid) {}

    friend bool operator== (const Key &key1, const Key &key2) {
        return (key1.graph_id_ == key2.graph_id_) && (key1.node_id_ == key2.node_id_) && (key1.lid_ == key2.lid_) && (key1.node_type_ == key2.node_type_) ;
    }
    friend bool operator!= (const Key &key1, const Key &key2) {
        return !((key1.graph_id_ == key2.graph_id_) && (key1.node_id_ == key2.node_id_) && (key1.lid_ == key2.lid_) && (key1.node_type_ == key2.node_type_));
    }

    friend bool operator< (const Key &key1, const Key &key2) {
        return (key1.node_type_ < key2.node_type_) ||
                ((key1.node_type_ == key2.node_type_) && (key1.graph_id_ < key2.graph_id_)) ||
                (((key1.node_type_ == key2.node_type_) && (key1.graph_id_ == key2.graph_id_)) && (key1.lid_ < key2.lid_)) ||
                ((((key1.node_type_ == key2.node_type_) && (key1.graph_id_ == key2.graph_id_)) && (key1.lid_ == key2.lid_)) && (key1.node_id_ < key2.node_id_));
    }

    friend std::ostream& operator<< (std::ostream& stream, const Key& key) {
        stream << "[R." << key.graph_id_ << "|" << "L." << key.lid_ << "|" << ((key.node_type_==Node::VAR)?"V.":"F.") << key.node_id_ << "]";
        return stream;
    }
};

// A message is either a normal GBP message (GBP) or an instruction to delete the connected node
// (DELETION), used when factors/variables are removed at runtime.
enum class MSG_TYPE { DELETION, GBP };

/******************************************************************/
// Data structure for a message that is passed in GBP. Following the thesis (Lie-group GBP), a message
// is a Gaussian given by a point estimate and a precision matrix defined in the tangent space at that
// point: {mu, lambda}. The point estimate mu is stored directly as a LieGroup element (not minimal
// coordinates), so the GBP maths uses it without fromCoeffs/coeffs round-trips. The information vector
// eta is meaningless on a manifold, so it is neither stored nor transmitted.
/******************************************************************/
class Message {
    public:
    MSG_TYPE type;            // GBP, or DELETION (a request to delete the connected node)
    LieGroup mu;              // mean / point estimate (a group element)
    Eigen::MatrixXd lambda;   // precision (inverse covariance), in the tangent space at mu

    // A bare message: GBP by default, or Message(DELETION). mu is UNSET; also the default ctor the
    // Mailbox map needs (always overwritten before use).
    Message(MSG_TYPE type_in = MSG_TYPE::GBP) : type(type_in) {}

    // A zero message for a group: identity mu (built from zero coordinates) with a dof x dof zero precision.
    Message(const LieGroup& group)
        : type(MSG_TYPE::GBP),
          mu(LieGroup(group.type(), Eigen::VectorXd::Zero(group.dof()))),
          lambda(Eigen::MatrixXd::Zero(group.dof(), group.dof())) {}

    // A message from an explicit mean (group element) and precision.
    Message(const LieGroup& mu_in, const Eigen::MatrixXd& lambda_in)
        : type(MSG_TYPE::GBP), mu(mu_in), lambda(lambda_in) {}

    // Checks if the Message is valid (if the precision matrix is not invertible, it's not valid)
    bool isValid(){
        return lambda.fullPivLu().isInvertible();
    }

    bool isDeleteInstruction(){
        return (type == MSG_TYPE::DELETION);
    }

    friend std::ostream& operator<< (std::ostream& stream, const Message& msg) {
        stream << msg.mu.coeffs().transpose() << "|" << msg.lambda(0,0);
        return stream;
    }
};

// This is the data structure representing a mailbox of Messages, that can be accessed by a Key.
using Mailbox = std::map<Key, Message>;

// The LieGroup wrapper and quantise helpers live in gbp/LieGroups.h (included above).