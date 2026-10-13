// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "physics/PhysicsWorld.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <unordered_map>

#include "physics/Integrator.hpp"

namespace sim::physics {
namespace {

void hashMix(std::uint64_t& hash, double value) {
    std::uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(double));
    hash ^= bits;
    hash *= 1099511628211ULL;
}

// LDL^T factorisation of a small symmetric positive (semi-)definite matrix,
// stored dense row-major but processed within its envelope: row i only spans
// columns [first_i, i], where first_i is its first non-zero. Fill-in of the
// factor never leaves the envelope, so a chain whose joints are created in
// order (a block-tridiagonal matrix) factors in O(n) instead of O(n^3).
//
// A pivot that collapses below a relative threshold belongs to a redundant row
// (e.g. two joints locking the same degree of freedom); its inverse is set to
// zero so that row simply receives no impulse instead of an unbounded one.
class EnvelopeLdlt {
public:
    void factor(std::vector<Real> matrix, std::size_t n) {
        n_ = n;
        // After factoring, the envelope of the strict lower triangle holds
        // S_ij = L_ij * d_j and inverseDiagonal_ holds 1 / d_j (0 if redundant).
        lower_ = std::move(matrix);
        first_.assign(n, 0);
        for (std::size_t i = 0; i < n; ++i) {
            std::size_t j = 0;
            while (j < i && lower_[i * n + j] == 0.0) {
                ++j;
            }
            first_[i] = j;
        }

        inverseDiagonal_.assign(n, 0.0);
        for (std::size_t j = 0; j < n; ++j) {
            const Real original = lower_[j * n + j];
            Real pivot = original;
            for (std::size_t k = first_[j]; k < j; ++k) {
                const Real scaled = lower_[j * n + k];
                pivot -= scaled * scaled * inverseDiagonal_[k];
            }
            const bool usable = original > 0.0 && pivot > 1e-12 * original;
            inverseDiagonal_[j] = usable ? 1.0 / pivot : 0.0;
            for (std::size_t i = j + 1; i < n; ++i) {
                if (first_[i] > j) {
                    continue;
                }
                Real value = lower_[i * n + j];
                for (std::size_t k = std::max(first_[i], first_[j]); k < j; ++k) {
                    value -= lower_[i * n + k] * lower_[j * n + k] * inverseDiagonal_[k];
                }
                lower_[i * n + j] = usable ? value : 0.0;
            }
        }
    }

    // Solves A x = b in place (x holds b on entry).
    void solve(std::vector<Real>& x) const {
        const std::size_t n = n_;
        for (std::size_t i = 0; i < n; ++i) {
            Real value = x[i];
            for (std::size_t k = first_[i]; k < i; ++k) {
                value -= lower_[i * n + k] * inverseDiagonal_[k] * x[k];
            }
            x[i] = value;
        }
        for (std::size_t i = 0; i < n; ++i) {
            x[i] *= inverseDiagonal_[i];
        }
        // Back substitution column by column: once x_k is final, remove its
        // contribution from the rows inside column k's envelope.
        for (std::size_t k = n; k-- > 0;) {
            for (std::size_t i = first_[k]; i < k; ++i) {
                x[i] -= lower_[k * n + i] * inverseDiagonal_[i] * x[k];
            }
        }
    }

private:
    std::size_t n_ = 0;
    std::vector<Real> lower_;
    std::vector<Real> inverseDiagonal_;
    std::vector<std::size_t> first_;
};

// A row's Jacobian together with M^-1 J^T, for the dynamic bodies only.
struct WeightedRow {
    const RigidBody* bodyA = nullptr;
    const RigidBody* bodyB = nullptr;
    Vec3 linearA{0.0};
    Vec3 angularA{0.0};
    Vec3 linearB{0.0};
    Vec3 angularB{0.0};
    Vec3 weightedLinearA{0.0};
    Vec3 weightedAngularA{0.0};
    Vec3 weightedLinearB{0.0};
    Vec3 weightedAngularB{0.0};
};

Real coupling(const Vec3& linear, const Vec3& angular, const Vec3& weightedLinear, const Vec3& weightedAngular) {
    return glm::dot(linear, weightedLinear) + glm::dot(angular, weightedAngular);
}

// Dense Delassus matrix  K_ij = J_i M^-1 J_j^T  (row-major). Entries of rows
// that share no dynamic body are exactly zero, which the envelope factor uses.
std::vector<Real> delassusMatrix(const std::vector<ConstraintRow*>& rows) {
    const std::size_t n = rows.size();
    std::vector<WeightedRow> weighted(n);
    for (std::size_t i = 0; i < n; ++i) {
        const ConstraintRow& row = *rows[i];
        WeightedRow& w = weighted[i];
        if (row.bodyA != nullptr && row.bodyA->isDynamic()) {
            w.bodyA = row.bodyA;
            w.linearA = row.linearA;
            w.angularA = row.angularA;
            w.weightedLinearA = row.bodyA->inverseMass * row.linearA;
            w.weightedAngularA = row.bodyA->inverseInertiaWorld() * row.angularA;
        }
        if (row.bodyB != nullptr && row.bodyB->isDynamic()) {
            w.bodyB = row.bodyB;
            w.linearB = row.linearB;
            w.angularB = row.angularB;
            w.weightedLinearB = row.bodyB->inverseMass * row.linearB;
            w.weightedAngularB = row.bodyB->inverseInertiaWorld() * row.angularB;
        }
    }

    std::vector<Real> matrix(n * n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        const WeightedRow& ri = weighted[i];
        for (std::size_t j = 0; j <= i; ++j) {
            const WeightedRow& rj = weighted[j];
            Real value = 0.0;
            if (ri.bodyA != nullptr) {
                if (ri.bodyA == rj.bodyA) {
                    value += coupling(ri.linearA, ri.angularA, rj.weightedLinearA, rj.weightedAngularA);
                }
                if (ri.bodyA == rj.bodyB) {
                    value += coupling(ri.linearA, ri.angularA, rj.weightedLinearB, rj.weightedAngularB);
                }
            }
            if (ri.bodyB != nullptr) {
                if (ri.bodyB == rj.bodyA) {
                    value += coupling(ri.linearB, ri.angularB, rj.weightedLinearA, rj.weightedAngularA);
                }
                if (ri.bodyB == rj.bodyB) {
                    value += coupling(ri.linearB, ri.angularB, rj.weightedLinearB, rj.weightedAngularB);
                }
            }
            matrix[i * n + j] = value;
            matrix[j * n + i] = value;
        }
    }
    return matrix;
}

Real rowVelocity(const ConstraintRow& row) {
    Real cdot = 0.0;
    if (row.bodyA != nullptr) {
        cdot += glm::dot(row.linearA, row.bodyA->linearVelocity) + glm::dot(row.angularA, row.bodyA->angularVelocity);
    }
    if (row.bodyB != nullptr) {
        cdot += glm::dot(row.linearB, row.bodyB->linearVelocity) + glm::dot(row.angularB, row.bodyB->angularVelocity);
    }
    return cdot;
}

// Moves a body by the position-level counterpart of an impulse,
// dx = M^-1 J^T lambda, applying the rotation through the exponential map.
void applyPositionImpulse(RigidBody* body, const Vec3& linear, const Vec3& angular, Real lambda) {
    if (body == nullptr || !body->isDynamic() || lambda == 0.0) {
        return;
    }
    body->position += body->inverseMass * linear * lambda;
    const Vec3 rotation = body->inverseInertiaWorld() * angular * lambda;
    body->orientation =
        quaternionNormalize(quaternionMultiply(quaternionFromRotationVector(rotation), body->orientation));
}

// Gathers the rigid bilateral rows of a group of constraints.
void collectProjectedRows(const std::vector<Constraint*>& constraints, std::vector<ConstraintRow*>& rows) {
    rows.clear();
    for (Constraint* constraint : constraints) {
        for (ConstraintRow& row : constraint->rows) {
            if (row.project && row.isBilateral()) {
                rows.push_back(&row);
            }
        }
    }
}

Real maxPositionError(const std::vector<ConstraintRow*>& rows) {
    Real maxError = 0.0;
    for (const ConstraintRow* row : rows) {
        maxError = std::max(maxError, std::abs(row->positionError));
    }
    return maxError;
}

// A transient constraint built from a contact manifold. Each contact point
// becomes a single one-sided row: the relative velocity along the contact normal
// may not become negative (the shapes may not approach). The impulse is clamped
// to [0, +inf) so the response only pushes apart, and a Baumgarte term removes
// penetration at the velocity level. The accumulated normal impulses are kept in
// the world between steps for warm starting.
class ContactConstraint : public Constraint {
public:
    void prepare(Real) override {}
    const char* typeName() const override { return "contact"; }

    void configure(RigidBody* a, RigidBody* b, const ContactManifold& manifold, Real baumgarte, Real slop, Real dt) {
        bodyA = a;
        bodyB = b;
        rows.assign(static_cast<std::size_t>(manifold.count), ConstraintRow{});
        for (int p = 0; p < manifold.count; ++p) {
            const ContactPoint& point = manifold.points[p];
            const Vec3 normal = point.normal;
            const Vec3 offsetA = point.position - a->position;
            const Vec3 offsetB = point.position - b->position;

            ConstraintRow& row = rows[static_cast<std::size_t>(p)];
            row.bodyA = a;
            row.bodyB = b;
            // Cdot = dot(normal, (vB + wB x rB) - (vA + wA x rA)).
            row.linearA = -normal;
            row.angularA = -glm::cross(offsetA, normal);
            row.linearB = normal;
            row.angularB = glm::cross(offsetB, normal);
            row.velocityTarget = 0.0;
            row.softness = 0.0;
            row.project = false;
            row.lower = 0.0;
            row.upper = kInfinity;

            const Real penetration = std::max(0.0, point.penetration - slop);
            row.bias = dt > 0.0 ? -baumgarte * penetration / dt : 0.0;

            const Real mass = constraintMass(row);
            row.effectiveMass = mass > 0.0 ? 1.0 / mass : 0.0;
        }
    }

    void setImpulses(const Real* normal, int count) {
        const int limit = std::min(count, static_cast<int>(rows.size()));
        for (int p = 0; p < limit; ++p) {
            rows[static_cast<std::size_t>(p)].impulse = normal[p];
        }
    }

    void getImpulses(Real* normal, int& count) const {
        count = static_cast<int>(rows.size());
        for (int p = 0; p < count; ++p) {
            normal[p] = rows[static_cast<std::size_t>(p)].impulse;
        }
    }
};

// One Newton projection step: solves K dLambda = -C over `rows` (C clamped to
// maxCorrection) and moves the bodies by M^-1 J^T dLambda. The velocities are
// left untouched, so the projection cannot inject kinetic energy.
void projectRows(const std::vector<ConstraintRow*>& rows, Real maxCorrection) {
    if (rows.empty()) {
        return;
    }
    EnvelopeLdlt ldlt;
    ldlt.factor(delassusMatrix(rows), rows.size());
    std::vector<Real> lambda(rows.size());
    for (std::size_t i = 0; i < rows.size(); ++i) {
        lambda[i] = -std::clamp(rows[i]->positionError, -maxCorrection, maxCorrection);
    }
    ldlt.solve(lambda);
    for (std::size_t i = 0; i < rows.size(); ++i) {
        const ConstraintRow& row = *rows[i];
        applyPositionImpulse(row.bodyA, row.linearA, row.angularA, lambda[i]);
        applyPositionImpulse(row.bodyB, row.linearB, row.angularB, lambda[i]);
    }
    for (const ConstraintRow* row : rows) {
        row->bodyA->updateCache();
        row->bodyB->updateCache();
    }
}

} // namespace

RigidBody& PhysicsWorld::createBody() {
    bodies_.emplace_back();
    RigidBody& created = bodies_.back();
    created.updateCache();
    return created;
}

void PhysicsWorld::step(Real dt) {
    for (RigidBody& body : bodies_) {
        body.updateCache();
        body.clearAccumulators();
    }

    for (auto& element : forceElements_) {
        element->apply();
    }

    for (RigidBody& body : bodies_) {
        integrator::integrateVelocities(body, gravity, dt);
    }

    for (auto& constraint : constraints_) {
        constraint->prepare(dt);
    }
    buildContacts(dt);

    std::vector<Constraint*> active;
    active.reserve(constraints_.size() + contacts_.size());
    for (auto& constraint : constraints_) {
        active.push_back(constraint.get());
    }
    for (auto& constraint : contacts_) {
        active.push_back(constraint.get());
    }
    for (Constraint* constraint : active) {
        constraint->warmStart();
    }

    const std::vector<std::vector<Constraint*>> islands = buildIslands(active);
    solveVelocities(islands);

    for (RigidBody& body : bodies_) {
        integrator::integratePositions(body, dt);
    }

    projectPositions(islands, dt);
    storeContactImpulses();

    time_ += static_cast<double>(dt);
}

void PhysicsWorld::buildContacts(Real dt) {
    contacts_.clear();
    contactKeys_.clear();

    const std::vector<ContactManifold> manifolds = detectCollisions(*this);
    contacts_.reserve(manifolds.size());
    contactKeys_.reserve(manifolds.size());
    for (const ContactManifold& manifold : manifolds) {
        auto constraint = std::make_unique<ContactConstraint>();
        constraint->configure(&bodies_[manifold.bodyA], &bodies_[manifold.bodyB], manifold, contactBaumgarte,
                              contactSlop, dt);
        const std::uint64_t key =
            (static_cast<std::uint64_t>(manifold.bodyA) << 32) | static_cast<std::uint64_t>(manifold.bodyB);
        const auto found = contactCache_.find(key);
        if (found != contactCache_.end() && found->second.count == manifold.count) {
            constraint->setImpulses(found->second.normal, manifold.count);
        }
        contacts_.push_back(std::move(constraint));
        contactKeys_.push_back(key);
    }
}

void PhysicsWorld::storeContactImpulses() {
    contactCache_.clear();
    for (std::size_t i = 0; i < contacts_.size(); ++i) {
        ContactImpulseCache cache;
        static_cast<ContactConstraint*>(contacts_[i].get())->getImpulses(cache.normal, cache.count);
        contactCache_.emplace(contactKeys_[i], cache);
    }
}

std::vector<std::vector<Constraint*>> PhysicsWorld::buildIslands(const std::vector<Constraint*>& all) const {
    // Union-find over constraint indices: two constraints share an island when
    // they touch the same dynamic body. Static and kinematic bodies have
    // infinite mass and do not couple their joints. The smaller index always
    // becomes the root and islands keep the creation order of their
    // constraints, so the solve order is deterministic.
    const std::size_t count = all.size();
    std::vector<std::size_t> parent(count);
    for (std::size_t i = 0; i < count; ++i) {
        parent[i] = i;
    }
    const auto find = [&parent](std::size_t i) {
        while (parent[i] != i) {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        return i;
    };

    std::unordered_map<const RigidBody*, std::size_t> owner;
    for (std::size_t i = 0; i < count; ++i) {
        const Constraint& constraint = *all[i];
        for (const RigidBody* body : {constraint.bodyA, constraint.bodyB}) {
            if (body == nullptr || !body->isDynamic()) {
                continue;
            }
            const auto found = owner.find(body);
            if (found == owner.end()) {
                owner.emplace(body, i);
                continue;
            }
            const std::size_t rootA = find(found->second);
            const std::size_t rootB = find(i);
            if (rootA != rootB) {
                parent[std::max(rootA, rootB)] = std::min(rootA, rootB);
            }
        }
    }

    std::vector<std::vector<Constraint*>> islands;
    std::vector<std::size_t> islandOfRoot(count, count);
    for (std::size_t i = 0; i < count; ++i) {
        const std::size_t root = find(i);
        if (islandOfRoot[root] == count) {
            islandOfRoot[root] = islands.size();
            islands.emplace_back();
        }
        islands[islandOfRoot[root]].push_back(all[i]);
    }
    return islands;
}

void PhysicsWorld::solveVelocities(const std::vector<std::vector<Constraint*>>& islands) {
    const std::size_t maxDirectRows = static_cast<std::size_t>(std::max(directSolverMaxRows, 0));
    std::vector<ConstraintRow*> block;
    std::vector<ConstraintRow*> bounded;
    std::vector<Real> delta;
    for (const std::vector<Constraint*>& island : islands) {
        block.clear();
        bounded.clear();
        for (Constraint* constraint : island) {
            for (ConstraintRow& row : constraint->rows) {
                if (row.isBilateral() && row.effectiveMass > 0.0) {
                    block.push_back(&row);
                } else {
                    bounded.push_back(&row);
                }
            }
        }

        if (block.empty() || block.size() > maxDirectRows) {
            // Plain projected Gauss-Seidel (sequential impulses).
            for (int iteration = 0; iteration < velocityIterations; ++iteration) {
                for (Constraint* constraint : island) {
                    constraint->solveVelocity();
                }
            }
            continue;
        }

        // Block Gauss-Seidel: the bilateral rows form one block solved exactly,
        //   (K + Gamma) dLambda = -(J v - target + bias + Gamma lambda),
        // which is the matrix form of solveRow(); bounded rows (motors, limits)
        // are then relaxed one by one. Without bounded rows a single pass is
        // already the exact solution of the step.
        std::vector<Real> matrix = delassusMatrix(block);
        for (std::size_t i = 0; i < block.size(); ++i) {
            matrix[i * block.size() + i] += block[i]->softness;
        }
        EnvelopeLdlt ldlt;
        ldlt.factor(std::move(matrix), block.size());

        delta.resize(block.size());
        const int passes = bounded.empty() ? 1 : std::max(velocityIterations, 1);
        for (int iteration = 0; iteration < passes; ++iteration) {
            for (std::size_t i = 0; i < block.size(); ++i) {
                const ConstraintRow& row = *block[i];
                delta[i] = -(rowVelocity(row) - row.velocityTarget + row.bias + row.softness * row.impulse);
            }
            ldlt.solve(delta);
            for (std::size_t i = 0; i < block.size(); ++i) {
                block[i]->impulse += delta[i];
                applyRowImpulse(*block[i], delta[i]);
            }
            for (ConstraintRow* row : bounded) {
                solveRow(*row);
            }
        }
    }
}

void PhysicsWorld::projectPositions(const std::vector<std::vector<Constraint*>>& islands, Real dt) {
    if (positionIterations <= 0) {
        return;
    }

    const std::size_t maxDirectRows = static_cast<std::size_t>(std::max(directSolverMaxRows, 0));
    std::vector<Constraint*> rigid;
    std::vector<Constraint*> single(1);
    std::vector<ConstraintRow*> rows;
    for (const std::vector<Constraint*>& island : islands) {
        rigid.clear();
        std::size_t rowCount = 0;
        for (Constraint* constraint : island) {
            if (constraint->hasProjectedRows()) {
                rigid.push_back(constraint);
                rowCount += constraint->rows.size();
            }
        }
        if (rigid.empty()) {
            continue;
        }

        for (int iteration = 0; iteration < positionIterations; ++iteration) {
            if (rowCount <= maxDirectRows) {
                // Newton step on the whole island: re-linearise every rigid row
                // at the current pose and solve K dLambda = -C at once.
                for (Constraint* constraint : rigid) {
                    constraint->prepare(dt);
                }
                collectProjectedRows(rigid, rows);
                if (maxPositionError(rows) <= positionTolerance) {
                    break;
                }
                projectRows(rows, maxPositionCorrection);
                continue;
            }

            // Non-linear Gauss-Seidel fallback for big islands: one exact
            // block per joint, swept in creation order.
            Real sweepError = 0.0;
            for (Constraint* constraint : rigid) {
                constraint->prepare(dt);
                single[0] = constraint;
                collectProjectedRows(single, rows);
                sweepError = std::max(sweepError, maxPositionError(rows));
                projectRows(rows, maxPositionCorrection);
            }
            if (sweepError <= positionTolerance) {
                break;
            }
        }
    }
}

Real PhysicsWorld::advance(Real elapsed) {
    accumulator_ += elapsed;
    while (accumulator_ >= fixedStep) {
        step(fixedStep);
        accumulator_ -= fixedStep;
    }
    return accumulator_;
}

std::uint64_t PhysicsWorld::stateHash() const {
    std::uint64_t hash = 1469598103934665603ULL;
    for (const RigidBody& body : bodies_) {
        const double values[13] = {body.position.x,       body.position.y,        body.position.z,
                                   body.orientation.w,    body.orientation.x,     body.orientation.y,
                                   body.orientation.z,    body.linearVelocity.x,  body.linearVelocity.y,
                                   body.linearVelocity.z, body.angularVelocity.x, body.angularVelocity.y,
                                   body.angularVelocity.z};
        for (double value : values) {
            hashMix(hash, value);
        }
    }
    hashMix(hash, time_);
    return hash;
}

WorldSnapshot PhysicsWorld::saveSnapshot() const {
    WorldSnapshot snapshot;
    snapshot.time = time_;
    snapshot.bodies.reserve(bodies_.size());
    for (const RigidBody& body : bodies_) {
        BodyState state;
        state.position = body.position;
        state.orientation = body.orientation;
        state.linearVelocity = body.linearVelocity;
        state.angularVelocity = body.angularVelocity;
        snapshot.bodies.push_back(state);
    }
    snapshot.impulses.reserve(constraints_.size());
    for (const auto& constraint : constraints_) {
        std::vector<Real> impulses;
        impulses.reserve(constraint->rows.size());
        for (const ConstraintRow& row : constraint->rows) {
            impulses.push_back(row.impulse);
        }
        snapshot.impulses.push_back(std::move(impulses));
    }
    return snapshot;
}

void PhysicsWorld::restoreSnapshot(const WorldSnapshot& snapshot) {
    const std::size_t count = std::min(bodies_.size(), snapshot.bodies.size());
    for (std::size_t i = 0; i < count; ++i) {
        RigidBody& body = bodies_[i];
        body.position = snapshot.bodies[i].position;
        body.orientation = snapshot.bodies[i].orientation;
        body.linearVelocity = snapshot.bodies[i].linearVelocity;
        body.angularVelocity = snapshot.bodies[i].angularVelocity;
        body.updateCache();
    }
    const std::size_t constraintCount = std::min(constraints_.size(), snapshot.impulses.size());
    for (std::size_t i = 0; i < constraintCount; ++i) {
        Constraint& constraint = *constraints_[i];
        const std::vector<Real>& impulses = snapshot.impulses[i];
        const std::size_t rowCount = std::min(constraint.rows.size(), impulses.size());
        for (std::size_t j = 0; j < rowCount; ++j) {
            constraint.rows[j].impulse = impulses[j];
        }
    }
    time_ = snapshot.time;
    accumulator_ = 0.0;
}

} // namespace sim::physics
