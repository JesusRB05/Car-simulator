// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Car Simulator contributors
#pragma once

#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <toml++/toml.hpp>

namespace sim::vfs {
class VirtualFileSystem;
}

namespace sim::modding {

// Which change a patch entry makes to its target key. See docs/MOD_FORMAT.md
// section 5.6.
enum class PatchOp {
    Set,      // replace the value (a number or a string)
    Add,      // add a number to the current value
    Multiply, // scale the current value by a number
    Remove,   // delete the key
};

// The value of a patch entry. `set` accepts a number or a string; `add` and
// `multiply` require a number; `remove` takes none.
struct PatchValue {
    enum class Kind { None, Number, String };
    Kind kind = Kind::None;
    double number = 0.0;
    std::string text;
};

// One change to a dotted key inside the target descriptor.
struct PatchEntry {
    std::string path; // dotted key, e.g. "suspension.spring_stiffness_front"
    PatchOp op = PatchOp::Set;
    PatchValue value;
};

// A targeted edit of another mod's descriptor: the anti-clobber primitive. The
// target's own file is never copied or overwritten.
struct Patch {
    std::string modId;  // the mod that ships it (empty when unknown)
    std::string source; // virtual path, for messages
    std::string target; // content reference, e.g. "base.core/voyager"
    std::vector<PatchEntry> entries;
};

// One texture a skin substitutes into the car's model. `slot` is
// `<material>.<channel>` or just `<channel>`; the channels the renderer
// understands are `base_color` (aliases `body`, `basecolor`, `albedo`) and
// `emissive` (alias `emission`).
struct SkinTexture {
    std::string slot;
    std::string file; // virtual path, resolved against the descriptor's folder
};

// A cosmetic skin for a car of any mod. Addressed as `<car-mod>/<car>/<skin>`;
// the folder mirrors the reference so skins from several mods compose
// additively, and a skin never copies the car.
struct Skin {
    std::string modId;  // the mod that ships it
    std::string source; // virtual path, for messages
    std::string ref;    // "<car-mod>/<car>/<skin>"
    std::string car;    // "<car-mod>/<car>"
    std::string name;
    std::string description;
    std::vector<double> bodyColor;     // optional, three or four numbers in [0, 1]
    std::vector<SkinTexture> textures; // the maps the skin replaces
};

// A part that fills a slot of a car of any mod. Addressed as
// `<car-mod>/<car>/<slot>/<part>`.
struct Part {
    std::string modId;  // the mod that ships it
    std::string source; // virtual path, for messages
    std::string ref;    // "<car-mod>/<car>/<slot>/<part>"
    std::string car;    // "<car-mod>/<car>"
    std::string slot;
    std::string name;
    // The slot parameters the part sets, a numeric key -> value map.
    std::vector<std::pair<std::string, double>> values;
};

// Two active mods fighting over the same thing. Reported with the mods in load
// order (last one wins today) so the manager can explain it and offer a
// reorder. (Distinct from modding::Conflict in Manifest.hpp, which is what one
// manifest *declares* it cannot coexist with.)
struct CompositionConflict {
    std::string kind;    // "patch" | "skin" | "part" | "override"
    std::string subject; // the thing the mods fight over
    std::vector<std::string> mods;
    std::string message; // human-readable, for the manager
};

// Everything a mod adds without copying: patches, skins and parts. Built by
// loadComposition() and consumed by the content loaders and the manager.
struct Composition {
    std::vector<Patch> patches;
    std::vector<Skin> skins;
    std::vector<Part> parts;
    std::vector<CompositionConflict> conflicts;
    std::vector<std::string> errors;

    // The patches that target `ref`, in application order.
    std::vector<const Patch*> patchesFor(std::string_view ref) const;
};

// Resolves a virtual path to the mod that ships it (see the app's ContentOwner
// at the call site). May be empty.
using ContentOwnerFn = std::function<std::string(std::string_view)>;

// Scans `patches/`, `skins/` and `parts/` through the VFS and parses every
// descriptor. A malformed file is recorded in `errors` (the others still load);
// two files fighting over the same thing become a `Conflict`.
Composition loadComposition(const vfs::VirtualFileSystem& vfs, const ContentOwnerFn& owner = {});

// Parses one patch descriptor (the text of a `patches/*.toml`). Returns
// std::nullopt and fills `error` when it is malformed. Shared by the loader and
// the creator tools (`sim mod validate`).
std::optional<Patch> parsePatchText(std::string_view text, std::string_view source, std::string_view modId,
                                    std::string& error);

// Applies the entries of `patches` to `table`, in order. On failure the
// offending entry is recorded in `errors` and the remaining patches still
// apply, so one bad patch never disables the rest.
void applyPatches(const std::vector<const Patch*>& patches, toml::table& table, std::vector<std::string>* errors);

// Records a conflict for every `(target, path)` two mods patch and every file
// two mods declare in `overrides`. `overrides` is (mod id, path) pairs.
void detectConflicts(Composition& composition, const std::vector<std::pair<std::string, std::string>>& overrides);

} // namespace sim::modding
