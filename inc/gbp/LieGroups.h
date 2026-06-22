/**************************************************************************************/
// Copyright (c) 2026 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#pragma once
#include <string>
#include <stdexcept>
#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <cmath>
#include <variant>
#include <Eigen/Dense>
#include <manif/SE2.h>
#include <manif/SO2.h>
#include <manif/SE3.h>
#include <manif/SO3.h>

/******************************************************************************************************/
// The set of Lie groups a variable can live in. UNSET is the empty/default state (a default-constructed
// LieGroup, overwritten before use). RN is the Euclidean group R^n: its dof is the length of its minimal
// coordinates, the others are fixed (SO2=1, SE2=3, SO3=3, SE3=6).
/******************************************************************************************************/
enum class LieGroupType { UNSET, RN, SO2, SE2, SO3, SE3 };

/******************************************************************************************************/
// Holds one element of whichever group a variable lives in, behind a single runtime type. A group is
// identified by its LieGroupType; an element is the group's minimal coordinates (SE2 -> [x,y,theta],
// SO3 -> log, SE3 -> [x,y,z, log], SO2 -> [theta], RN -> the vector itself). The two-piece public view:
//
//     LieGroup X(LieGroupType::SO2, coeffs);   // build from a type + minimal coordinates
//     LieGroupType t = X.type();               // which group
//     Eigen::VectorXd c = X.coeffs();          // its minimal coordinates
//     LieGroup Xnew(t, c);                      // reconstruct an identical element
//
// Internally the element is stored as the concrete manif::<> group (or an Eigen::VectorXd for R^n),
// behind a std::variant, so storing the materialised element amortises the exp/log across the many
// retractions/compositions per GBP sweep. dof()/isEuclidean() are cached at construction.
//
// The Lie backend is the manif library (https://github.com/artivis/manif). manif exposes analytic
// Jacobians as operation out-parameters (rplus/rminus/compose/inverse(J...)), surfaced below by the
// methods taking output Jacobian references - this is how factors get exact (not finite-difference)
// Jacobians. The GBP math in Variable/Factor only calls the methods below, so it stays group-agnostic.
//
// To add a group: add a LieGroupType, a variant alternative, a branch in each visitor / makeElement /
// coeffs, and a makeLieGroup case. The GBP algorithm itself stays group-agnostic.
/******************************************************************************************************/
class LieGroup {
public:
    // std::monostate is the UNSET element (a default-constructed, not-yet-assigned LieGroup).
    using Element = std::variant<std::monostate, manif::SE2d, manif::SO2d, manif::SE3d, manif::SO3d, Eigen::VectorXd>;

    // dof of a group; for RN, pass the length of its minimal coordinates.
    static int dofOf(LieGroupType t, int rn_size = 0) {
        switch (t) {
            case LieGroupType::SO2: return 1;
            case LieGroupType::SE2: return 3;
            case LieGroupType::SO3: return 3;
            case LieGroupType::SE3: return 6;
            case LieGroupType::RN:  default: return rn_size;
        }
    }

    // --- Constructors -------------------------------------------------------------------------------
    // Default: UNSET (the empty group). Every default-constructed member is overwritten before use;
    // calling a group operation on an UNSET element asserts.
    LieGroup() : g_(std::monostate{}), dof_(0), euclidean_(false) {}

    // The canonical constructor: a group named by its type, from minimal coordinates.
    LieGroup(LieGroupType t, const Eigen::VectorXd& c)
        : g_(makeElement(t, c)), dof_(dofOf(t, (int)c.size())), euclidean_(t == LieGroupType::RN) {}

    // R^n convenience: build from an Eigen vector/expression. manif's groups aren't MatrixBase, so they
    // don't match here.
    template <class Derived>
    LieGroup(const Eigen::MatrixBase<Derived>& g)
        : g_(std::in_place_type<Eigen::VectorXd>, Eigen::VectorXd(g)),
          dof_((int)g.size()), euclidean_(true) {}

    // Direct-from-element constructors. Used internally to wrap an already computed manif element with no
    // exp/log round-trip; meta is set inline (no std::visit).
    LieGroup(const manif::SE2d& g) : g_(std::in_place_type<manif::SE2d>, g), dof_(3), euclidean_(false) {}
    LieGroup(const manif::SO2d& g) : g_(std::in_place_type<manif::SO2d>, g), dof_(1), euclidean_(false) {}
    LieGroup(const manif::SE3d& g) : g_(std::in_place_type<manif::SE3d>, g), dof_(6), euclidean_(false) {}
    LieGroup(const manif::SO3d& g) : g_(std::in_place_type<manif::SO3d>, g), dof_(3), euclidean_(false) {}

    // --- Inspection ---------------------------------------------------------------------------------
    LieGroupType type() const {
        return std::visit([](auto const& x) -> LieGroupType {
            using T = std::decay_t<decltype(x)>;
            if constexpr (std::is_same_v<T, manif::SE2d>)      return LieGroupType::SE2;
            else if constexpr (std::is_same_v<T, manif::SO2d>) return LieGroupType::SO2;
            else if constexpr (std::is_same_v<T, manif::SE3d>) return LieGroupType::SE3;
            else if constexpr (std::is_same_v<T, manif::SO3d>) return LieGroupType::SO3;
            else if constexpr (std::is_same_v<T, Eigen::VectorXd>) return LieGroupType::RN;
            else                                                return LieGroupType::UNSET;
        }, g_);
    }
    int  dof() const { return dof_; }
    bool isEuclidean() const { return euclidean_; }

    // The group's config-string name: "SO2"/"SE2"/"SO3"/"SE3", or "R<dof>" for R^n (inverse of
    // makeLieGroup). Used to construct factors by name.
    std::string name() const {
        switch (type()) {
            case LieGroupType::SO2: return "SO2";
            case LieGroupType::SE2: return "SE2";
            case LieGroupType::SO3: return "SO3";
            case LieGroupType::SE3: return "SE3";
            case LieGroupType::RN:  return "R" + std::to_string(dof_);
            default:                return "UNSET";
        }
    }

    // This element's minimal coordinates (the user-facing parameterisation).
    Eigen::VectorXd coeffs() const {
        return std::visit([](auto const& x) -> Eigen::VectorXd {
            using T = std::decay_t<decltype(x)>;
            if constexpr (std::is_same_v<T, manif::SE2d>) { Eigen::Vector3d c; c << x.x(), x.y(), x.angle(); return c; }
            else if constexpr (std::is_same_v<T, manif::SO2d>) return Eigen::Vector<double,1>{x.angle()};
            else if constexpr (std::is_same_v<T, manif::SO3d>) return x.log().coeffs();                       // so3 tangent (3)
            else if constexpr (std::is_same_v<T, manif::SE3d>) { Eigen::Matrix<double,6,1> c; c << x.translation(), x.asSO3().log().coeffs(); return c; }
            else if constexpr (std::is_same_v<T, Eigen::VectorXd>) return x;                                  // R^n
            else return Eigen::VectorXd();                                                                    // UNSET -> empty
        }, g_);
    }

    // Build another element of THIS group from minimal coordinates (inverse of coeffs()).
    LieGroup fromCoeffs(const Eigen::VectorXd& v) const { return LieGroup(type(), v); }

    // Retraction: this (+) tau (right-plus on the manifold). Returns the new element.
    LieGroup operator+(const Eigen::VectorXd& tau) const {
        return std::visit([&](auto const& x) -> LieGroup {
            using T = std::decay_t<decltype(x)>;
            if constexpr (std::is_same_v<T, std::monostate>) { assert(false && "operator+ on UNSET LieGroup"); return LieGroup(); }
            else if constexpr (std::is_same_v<T, Eigen::VectorXd>) return LieGroup(Eigen::VectorXd(x + tau));
            else { typename T::Tangent t(tau); return LieGroup(T(x + t)); }   // x (+) t = right-plus
        }, g_);
    }
    // Inverse retraction: this (-) rhs (right-minus), returning the tangent that maps rhs to this.
    Eigen::VectorXd operator-(const LieGroup& rhs) const {
        return std::visit([&](auto const& x) -> Eigen::VectorXd {
            using T = std::decay_t<decltype(x)>;
            if constexpr (std::is_same_v<T, std::monostate>) { assert(false && "operator- on UNSET LieGroup"); return Eigen::VectorXd(); }
            else if constexpr (std::is_same_v<T, Eigen::VectorXd>) return Eigen::VectorXd(x - std::get<T>(rhs.g_));
            else return (x - std::get<T>(rhs.g_)).coeffs();   // right-minus, tangent as an Eigen vector
        }, g_);
    }

    // Right Jacobian of exp and its inverse, for a tangent vector (identity for Euclidean groups).
    Eigen::MatrixXd drExp(const Eigen::VectorXd& tau) const {
        return std::visit([&](auto const& x) -> Eigen::MatrixXd {
            using T = std::decay_t<decltype(x)>;
            if constexpr (std::is_same_v<T, std::monostate>) { assert(false && "drExp on UNSET LieGroup"); return Eigen::MatrixXd(); }
            else if constexpr (std::is_same_v<T, Eigen::VectorXd>) return Eigen::MatrixXd::Identity(tau.size(), tau.size());
            else { typename T::Tangent t(tau); return t.rjac(); }
        }, g_);
    }
    Eigen::MatrixXd drExpInv(const Eigen::VectorXd& tau) const {
        return std::visit([&](auto const& x) -> Eigen::MatrixXd {
            using T = std::decay_t<decltype(x)>;
            if constexpr (std::is_same_v<T, std::monostate>) { assert(false && "drExpInv on UNSET LieGroup"); return Eigen::MatrixXd(); }
            else if constexpr (std::is_same_v<T, Eigen::VectorXd>) return Eigen::MatrixXd::Identity(tau.size(), tau.size());
            else { typename T::Tangent t(tau); return t.rjacinv(); }
        }, g_);
    }

    // --- group operations (this and rhs must be the same group) ---------------------------------------
    // Group composition this * rhs (for R^n the group op is vector addition).
    LieGroup operator*(const LieGroup& rhs) const {
        return std::visit([&](auto const& x) -> LieGroup {
            using T = std::decay_t<decltype(x)>;
            if constexpr (std::is_same_v<T, std::monostate>) { assert(false && "operator* on UNSET LieGroup"); return LieGroup(); }
            else if constexpr (std::is_same_v<T, Eigen::VectorXd>) return LieGroup(Eigen::VectorXd(x + std::get<T>(rhs.g_)));
            else return LieGroup(T(x * std::get<T>(rhs.g_)));
        }, g_);
    }
    // Group inverse (for R^n, negation).
    LieGroup inverse() const {
        return std::visit([&](auto const& x) -> LieGroup {
            using T = std::decay_t<decltype(x)>;
            if constexpr (std::is_same_v<T, std::monostate>) { assert(false && "inverse on UNSET LieGroup"); return LieGroup(); }
            else if constexpr (std::is_same_v<T, Eigen::VectorXd>) return LieGroup(Eigen::VectorXd(-x));
            else return LieGroup(T(x.inverse()));
        }, g_);
    }
    // Relative element this^-1 * rhs (the transform taking this to rhs). For R^n this is rhs - this.
    LieGroup between(const LieGroup& rhs) const { return inverse() * rhs; }

    // Group inverse, also returning J_self = d(out)/d(this) (for chaining Jacobians, manif style).
    LieGroup inverse(Eigen::MatrixXd& J_self) const {
        return std::visit([&](auto const& x) -> LieGroup {
            using T = std::decay_t<decltype(x)>;
            if constexpr (std::is_same_v<T, std::monostate>) { assert(false && "inverse on UNSET LieGroup"); return LieGroup(); }
            else if constexpr (std::is_same_v<T, Eigen::VectorXd>) {
                J_self = -Eigen::MatrixXd::Identity(x.size(), x.size());
                return LieGroup(Eigen::VectorXd(-x));
            } else {
                Eigen::Matrix<double, T::DoF, T::DoF> J;
                auto y = x.inverse(J); J_self = J;
                return LieGroup(T(y));
            }
        }, g_);
    }
    // this^-1 * rhs, also returning J_self = d(out)/d(this) and J_rhs = d(out)/d(rhs).
    LieGroup between(const LieGroup& rhs, Eigen::MatrixXd& J_self, Eigen::MatrixXd& J_rhs) const {
        return std::visit([&](auto const& x) -> LieGroup {
            using T = std::decay_t<decltype(x)>;
            if constexpr (std::is_same_v<T, std::monostate>) { assert(false && "between on UNSET LieGroup"); return LieGroup(); }
            else if constexpr (std::is_same_v<T, Eigen::VectorXd>) {
                int n = (int)x.size(); J_self = -Eigen::MatrixXd::Identity(n, n); J_rhs = Eigen::MatrixXd::Identity(n, n);
                return LieGroup(Eigen::VectorXd(std::get<T>(rhs.g_) - x));
            } else {
                Eigen::Matrix<double, T::DoF, T::DoF> Js, Jr;
                auto y = x.between(std::get<T>(rhs.g_), Js, Jr); J_self = Js; J_rhs = Jr;
                return LieGroup(T(y));
            }
        }, g_);
    }

    // Adjoint matrix Ad (dof x dof); identity for the abelian R^n.
    Eigen::MatrixXd Ad() const {
        return std::visit([&](auto const& x) -> Eigen::MatrixXd {
            using T = std::decay_t<decltype(x)>;
            if constexpr (std::is_same_v<T, std::monostate>) { assert(false && "Ad on UNSET LieGroup"); return Eigen::MatrixXd(); }
            else if constexpr (std::is_same_v<T, Eigen::VectorXd>) return Eigen::MatrixXd::Identity(dof_, dof_);
            else return x.adj();
        }, g_);
    }

    // Lie-algebra logarithm: the tangent vector tau with exp(tau) == this. NOTE this is the Lie tangent
    // (e.g. SE2 -> [vx,vy,omega]), which differs from coeffs() (the minimal chart, e.g. SE2 -> [x,y,theta]).
    // For R^n the two coincide (the vector itself).
    Eigen::VectorXd log() const {
        return std::visit([&](auto const& x) -> Eigen::VectorXd {
            using T = std::decay_t<decltype(x)>;
            if constexpr (std::is_same_v<T, std::monostate>) { assert(false && "log on UNSET LieGroup"); return Eigen::VectorXd(); }
            else if constexpr (std::is_same_v<T, Eigen::VectorXd>) return x;
            else return x.log().coeffs();
        }, g_);
    }
    // Lie-algebra exponential: the element exp(tau) in group `t` (inverse of log()).
    static LieGroup exp(LieGroupType t, const Eigen::VectorXd& tau) {
        return LieGroup(t, Eigen::VectorXd::Zero(dofOf(t, (int)tau.size()))) + tau;   // identity (+) tau
    }

    // --- analytic-Jacobian operations (manif out-parameters) ------------------------------------------
    // Right-minus this (-) rhs, also returning J_self = d(tau)/d(this) and J_rhs = d(tau)/d(rhs).
    Eigen::VectorXd ominus(const LieGroup& rhs, Eigen::MatrixXd& J_self, Eigen::MatrixXd& J_rhs) const {
        return std::visit([&](auto const& x) -> Eigen::VectorXd {
            using T = std::decay_t<decltype(x)>;
            if constexpr (std::is_same_v<T, std::monostate>) { assert(false && "ominus on UNSET LieGroup"); return Eigen::VectorXd(); }
            else if constexpr (std::is_same_v<T, Eigen::VectorXd>) {
                int n = (int)x.size(); J_self = Eigen::MatrixXd::Identity(n, n); J_rhs = -Eigen::MatrixXd::Identity(n, n);
                return Eigen::VectorXd(x - std::get<T>(rhs.g_));
            } else {
                using Jac = Eigen::Matrix<double, T::DoF, T::DoF>;
                Jac js, jr;
                auto t = x.rminus(std::get<T>(rhs.g_), js, jr);
                J_self = js; J_rhs = jr;
                return t.coeffs();
            }
        }, g_);
    }
    // Right-plus this (+) tau, also returning J_self = d(out)/d(this) and J_tau = d(out)/d(tau).
    LieGroup oplus(const Eigen::VectorXd& tau, Eigen::MatrixXd& J_self, Eigen::MatrixXd& J_tau) const {
        return std::visit([&](auto const& x) -> LieGroup {
            using T = std::decay_t<decltype(x)>;
            if constexpr (std::is_same_v<T, std::monostate>) { assert(false && "oplus on UNSET LieGroup"); return LieGroup(); }
            else if constexpr (std::is_same_v<T, Eigen::VectorXd>) {
                int n = (int)x.size(); J_self = Eigen::MatrixXd::Identity(n, n); J_tau = Eigen::MatrixXd::Identity(n, n);
                return LieGroup(Eigen::VectorXd(x + tau));
            } else {
                using Jac = Eigen::Matrix<double, T::DoF, T::DoF>;
                Jac js, jt;
                auto y = x.rplus(typename T::Tangent(tau), js, jt);
                J_self = js; J_tau = jt;
                return LieGroup(T(y));
            }
        }, g_);
    }

    // Act on a point: rigid transform for SE2/SE3, rotation for SO2/SO3, translation for R^n. Also
    // returns the analytic Jacobians J_self = d(out)/d(this) and J_point = d(out)/d(p). The point has the
    // group's space dimension (2 for SE2/SO2, 3 for SE3/SO3, n for R^n).
    Eigen::VectorXd act(const Eigen::VectorXd& p, Eigen::MatrixXd& J_self, Eigen::MatrixXd& J_point) const {
        return std::visit([&](auto const& x) -> Eigen::VectorXd {
            using T = std::decay_t<decltype(x)>;
            if constexpr (std::is_same_v<T, std::monostate>) { assert(false && "act on UNSET LieGroup"); return Eigen::VectorXd(); }
            else if constexpr (std::is_same_v<T, Eigen::VectorXd>) {
                int n = (int)x.size(); J_self = Eigen::MatrixXd::Identity(n, n); J_point = Eigen::MatrixXd::Identity(n, n);
                return Eigen::VectorXd(x + p);
            } else {
                constexpr int Dm = T::Dim, Df = T::DoF;
                typename T::Vector pt = p.head(Dm);
                Eigen::Matrix<double, Dm, Df> Jm;
                Eigen::Matrix<double, Dm, Dm> Jv;
                typename T::Vector y = x.act(pt, Jm, Jv);
                J_self = Jm; J_point = Jv;
                return Eigen::VectorXd(y);
            }
        }, g_);
    }

    // --- typed accessors (assert if the group has no such part) ---------------------------------------
    // Translation part: SE2 -> [x,y], SE3 -> [x,y,z], R^n -> the vector itself.
    Eigen::VectorXd translation() const {
        return std::visit([&](auto const& x) -> Eigen::VectorXd {
            using T = std::decay_t<decltype(x)>;
            if constexpr (std::is_same_v<T, manif::SE2d>)      return x.translation();
            else if constexpr (std::is_same_v<T, manif::SE3d>) return x.translation();
            else if constexpr (std::is_same_v<T, Eigen::VectorXd>) return x;
            else { assert(false && "translation() on a group with no translation part"); return Eigen::VectorXd(); }
        }, g_);
    }
    // Planar heading angle (radians): SO2 and SE2 only.
    double angle() const {
        return std::visit([&](auto const& x) -> double {
            using T = std::decay_t<decltype(x)>;
            if constexpr (std::is_same_v<T, manif::SO2d>)      return x.angle();
            else if constexpr (std::is_same_v<T, manif::SE2d>) return x.angle();
            else { assert(false && "angle() is only defined for SO2/SE2"); return 0.0; }
        }, g_);
    }

private:
    // Build the stored element for a (type, coeffs) pair.
    static Element makeElement(LieGroupType t, const Eigen::VectorXd& v) {
        // Cheap guard: coeffs must match the group's dof (compiled out in release builds).
        assert((int)v.size() == dofOf(t, (int)v.size()) && "LieGroup(type, coeffs): coeffs size != group dof");
        switch (t) {
            case LieGroupType::SO2:
                return Element(std::in_place_type<manif::SO2d>, manif::SO2d(v(0)));
            case LieGroupType::SE2:
                return Element(std::in_place_type<manif::SE2d>, manif::SE2d(v(0), v(1), v(2)));
            case LieGroupType::SO3:
                return Element(std::in_place_type<manif::SO3d>, manif::SO3Tangentd(Eigen::Vector3d(v.head<3>())).exp());
            case LieGroupType::SE3:
                return Element(std::in_place_type<manif::SE3d>, manif::SE3d(Eigen::Vector3d(v.head<3>()), manif::SO3Tangentd(Eigen::Vector3d(v.tail<3>())).exp()));
            case LieGroupType::UNSET:
                return Element(std::in_place_type<std::monostate>);
            case LieGroupType::RN:
            default:
                return Element(std::in_place_type<Eigen::VectorXd>, Eigen::VectorXd(v));
        }
    }

    Element g_;
    int  dof_;
    bool euclidean_;
};

// Map a Lie-group name to its enum type: "RN"/"R<n>" -> RN, else SO2/SE2/SO3/SE3.
inline LieGroupType lieGroupTypeFromString(const std::string& s) {
    if (s == "SO2") return LieGroupType::SO2;
    if (s == "SE2") return LieGroupType::SE2;
    if (s == "SO3") return LieGroupType::SO3;
    if (s == "SE3") return LieGroupType::SE3;
    if (!s.empty() && s[0] == 'R') return LieGroupType::RN;
    throw std::runtime_error("Unknown Lie group: " + s);
}

// Build the identity element of the group named in a config string: "SE2", "SO2", "SE3", "SO3", or
// "R<n>" (any n >= 1) for the Euclidean group R^n.
inline LieGroup makeLieGroup(const std::string& s) {
    if (s.size() >= 2 && s[0] == 'R') {
        int n = std::atoi(s.c_str() + 1);
        if (n >= 1) return LieGroup(LieGroupType::RN, Eigen::VectorXd::Zero(n));
        throw std::runtime_error("Unknown Lie group: " + s);
    }
    LieGroupType t = lieGroupTypeFromString(s);
    return LieGroup(t, Eigen::VectorXd::Zero(LieGroup::dofOf(t)));
}
