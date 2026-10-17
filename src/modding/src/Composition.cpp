// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Car Simulator contributors
#include "modding/Composition.hpp"

#include <algorithm>
#include <cctype>
#include <map>
#include <set>

#include "modding/Manifest.hpp"
#include "modding/Package.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace sim::modding {
namespace {

bool isKeyChar(char c) {
    const unsigned char uc = static_cast<unsigned char>(c);
    return std::isalnum(uc) != 0 || c == '_';
}

std::string readString(const toml::table& table, std::string_view key) {
    if (const toml::node* node = table.get(key)) {
        if (auto value = node->value<std::string>()) {
            return *value;
        }
    }
    return {};
}

std::vector<double> readNumberArray(const toml::table& table, std::string_view key) {
    std::vector<double> result;
    const toml::node* node = table.get(key);
    if (node == nullptr) {
        return result;
    }
    if (const toml::array* array = node->as_array()) {
        for (const toml::node& element : *array) {
            if (auto value = element.value<double>()) {
                result.push_back(*value);
            }
        }
    }
    return result;
}

std::vector<std::string> listSorted(const vfs::VirtualFileSystem& vfs, const std::string& directory) {
    std::vector<std::string> entries = vfs.list(directory);
    std::sort(entries.begin(), entries.end());
    return entries;
}

// A dotted patch path: identifiers separated by '.'. No array indices: a whole
// array is replaced with `set`.
bool splitPath(std::string_view path, std::vector<std::string>& keys) {
    std::size_t start = 0;
    while (start <= path.size()) {
        const std::size_t dot = path.find('.', start);
        const std::string_view key =
            dot == std::string_view::npos ? path.substr(start) : path.substr(start, dot - start);
        if (key.empty() || !std::all_of(key.begin(), key.end(), isKeyChar)) {
            return false;
        }
        keys.emplace_back(key);
        if (dot == std::string_view::npos) {
            break;
        }
        start = dot + 1;
    }
    return !keys.empty();
}

bool parseOp(std::string_view text, PatchOp& op) {
    if (text == "set") {
        op = PatchOp::Set;
    } else if (text == "add") {
        op = PatchOp::Add;
    } else if (text == "multiply") {
        op = PatchOp::Multiply;
    } else if (text == "remove") {
        op = PatchOp::Remove;
    } else {
        return false;
    }
    return true;
}

const char* opName(PatchOp op) {
    switch (op) {
    case PatchOp::Set:
        return "set";
    case PatchOp::Add:
        return "add";
    case PatchOp::Multiply:
        return "multiply";
    case PatchOp::Remove:
        return "remove";
    }
    return "?";
}

// Parses one declaration under `[[patch]]`.
bool parseEntry(const toml::table& table, PatchEntry& entry, std::string& error) {
    entry.path = readString(table, "path");
    std::vector<std::string> keys;
    if (!splitPath(entry.path, keys)) {
        error = "invalid 'path' '" + entry.path + "' (a dotted key, e.g. 'aero.lift_coefficient')";
        return false;
    }

    const std::string op = readString(table, "op");
    if (op.empty()) {
        error = "missing 'op' for patch entry '" + entry.path + "'";
        return false;
    }
    if (!parseOp(op, entry.op)) {
        error = "unknown 'op' '" + op + "' (set | add | multiply | remove)";
        return false;
    }

    const toml::node* value = table.get("value");
    if (entry.op == PatchOp::Remove) {
        if (value != nullptr) {
            error = "'remove' takes no 'value'";
            return false;
        }
        return true;
    }
    if (value == nullptr) {
        error = "'" + op + "' needs a 'value'";
        return false;
    }
    if (auto number = value->value<double>()) {
        entry.value.kind = PatchValue::Kind::Number;
        entry.value.number = *number;
        return true;
    }
    if (auto text = value->value<std::string>()) {
        if (entry.op != PatchOp::Set) {
            error = "'" + op + "' needs a numeric 'value'";
            return false;
        }
        entry.value.kind = PatchValue::Kind::String;
        entry.value.text = *text;
        return true;
    }
    error = "'value' must be a number or a string";
    return false;
}

bool parsePatch(std::string_view text, const std::string& source, const std::string& modId, std::string& error,
                Patch& patch) {
    toml::table table;
    try {
        table = toml::parse(text, source);
    } catch (const toml::parse_error& parseError) {
        error = "TOML parse error: " + std::string(parseError.description());
        return false;
    }

    patch.modId = modId;
    patch.source = source;
    patch.target = readString(table, "target");
    if (patch.target.empty()) {
        error = "missing required field 'target'";
        return false;
    }
    if (!isValidContentRef(patch.target)) {
        error = "invalid 'target' '" + patch.target + "' (a content reference, e.g. 'base.core/voyager')";
        return false;
    }

    const toml::array* entries = table.get_as<toml::array>("patch");
    if (entries == nullptr || entries->empty()) {
        error = "a patch needs at least one '[[patch]]' entry";
        return false;
    }
    for (const toml::node& node : *entries) {
        const toml::table* entryTable = node.as_table();
        if (entryTable == nullptr) {
            error = "every '[[patch]]' must be a table";
            return false;
        }
        PatchEntry entry;
        std::string entryError;
        if (!parseEntry(*entryTable, entry, entryError)) {
            error = entryError;
            return false;
        }
        patch.entries.push_back(std::move(entry));
    }
    return true;
}

// `expected` is the reference the folder implies; the `car`/`id` fields must
// agree with it, so a skin cannot be filed under the wrong car.
bool parseSkin(std::string_view text, const std::string& source, const std::string& modId, const std::string& car,
               const std::string& ref, const std::string& skinId, std::string& error, Skin& skin) {
    toml::table table;
    try {
        table = toml::parse(text, source);
    } catch (const toml::parse_error& parseError) {
        error = "TOML parse error: " + std::string(parseError.description());
        return false;
    }

    skin.modId = modId;
    skin.source = source;
    skin.ref = ref;
    skin.car = car;
    skin.name = readString(table, "name");
    skin.description = readString(table, "description");
    skin.bodyColor = readNumberArray(table, "body_color");
    for (double component : skin.bodyColor) {
        if (component < 0.0 || component > 1.0) {
            error = "'body_color' components must be in [0, 1]";
            return false;
        }
    }
    if (!skin.bodyColor.empty() && skin.bodyColor.size() != 3 && skin.bodyColor.size() != 4) {
        error = "'body_color' must have three or four components";
        return false;
    }

    const std::string id = readString(table, "id");
    if (!id.empty() && id != skinId) {
        error = "'id' '" + id + "' does not match the folder '" + skinId + "'";
        return false;
    }
    const std::string declaredCar = readString(table, "car");
    if (!declaredCar.empty() && declaredCar != car) {
        error = "'car' '" + declaredCar + "' does not match the folder '" + car + "'";
        return false;
    }

    if (const toml::array* textures = table.get_as<toml::array>("textures")) {
        for (const toml::node& node : *textures) {
            const toml::table* texture = node.as_table();
            if (texture == nullptr) {
                error = "every '[[textures]]' must be a table";
                return false;
            }
            const std::string slot = readString(*texture, "slot");
            if (slot.empty()) {
                error = "every '[[textures]]' needs a 'slot'";
                return false;
            }
            skin.textures.push_back(slot);
        }
    }
    return true;
}

bool parsePart(std::string_view text, const std::string& source, const std::string& modId, const std::string& car,
               const std::string& slot, const std::string& ref, const std::string& partId, std::string& error,
               Part& part) {
    toml::table table;
    try {
        table = toml::parse(text, source);
    } catch (const toml::parse_error& parseError) {
        error = "TOML parse error: " + std::string(parseError.description());
        return false;
    }

    part.modId = modId;
    part.source = source;
    part.ref = ref;
    part.car = car;
    part.slot = slot;
    part.name = readString(table, "name");
    const std::string id = readString(table, "id");
    if (!id.empty() && id != partId) {
        error = "'id' '" + id + "' does not match the file '" + partId + "'";
        return false;
    }
    const std::string declaredSlot = readString(table, "slot");
    if (!declaredSlot.empty() && declaredSlot != slot) {
        error = "'slot' '" + declaredSlot + "' does not match the folder '" + slot + "'";
        return false;
    }

    // Every remaining numeric key is a slot parameter the part sets.
    for (auto&& [key, node] : table) {
        const std::string name(key.str());
        if (name == "id" || name == "name" || name == "slot" || name == "license" || name == "description" ||
            name == "authors") {
            continue;
        }
        if (auto value = node.value<double>()) {
            part.values.emplace_back(name, *value);
        }
    }
    if (part.values.empty()) {
        error = "a part must set at least one slot value";
        return false;
    }
    return true;
}

void scanPatches(const vfs::VirtualFileSystem& vfs, const ContentOwnerFn& owner, Composition& composition) {
    for (const std::string& file : listSorted(vfs, std::string(kPatchesDir))) {
        const std::string path = std::string(kPatchesDir) + "/" + file;
        if (vfs.isDirectory(path) || path.size() < 5 || path.compare(path.size() - 5, 5, ".toml") != 0) {
            continue;
        }
        const auto text = vfs.readText(path);
        if (!text) {
            composition.errors.push_back(path + ": cannot read the patch");
            continue;
        }
        Patch patch;
        std::string error;
        if (parsePatch(*text, path, owner ? owner(path) : std::string{}, error, patch)) {
            composition.patches.push_back(std::move(patch));
        } else {
            composition.errors.push_back(path + ": " + error);
        }
    }
}

void scanSkins(const vfs::VirtualFileSystem& vfs, const ContentOwnerFn& owner, Composition& composition) {
    for (const std::string& carMod : listSorted(vfs, std::string(kSkinsDir))) {
        const std::string carModDir = std::string(kSkinsDir) + "/" + carMod;
        if (!vfs.isDirectory(carModDir)) {
            continue;
        }
        for (const std::string& car : listSorted(vfs, carModDir)) {
            const std::string carDir = carModDir + "/" + car;
            if (!vfs.isDirectory(carDir)) {
                continue;
            }
            for (const std::string& skinId : listSorted(vfs, carDir)) {
                const std::string skinDir = carDir + "/" + skinId;
                if (!vfs.isDirectory(skinDir)) {
                    continue;
                }
                std::string path = skinDir + "/livery.toml";
                if (!vfs.exists(path)) {
                    path = skinDir + "/skin.toml";
                }
                if (!vfs.exists(path)) {
                    continue;
                }
                const auto text = vfs.readText(path);
                if (!text) {
                    composition.errors.push_back(path + ": cannot read the skin");
                    continue;
                }
                Skin skin;
                std::string error;
                const std::string carRef = carMod + "/" + car;
                if (parseSkin(*text, path, owner ? owner(path) : std::string{}, carRef, carRef + "/" + skinId, skinId,
                              error, skin)) {
                    composition.skins.push_back(std::move(skin));
                } else {
                    composition.errors.push_back(path + ": " + error);
                }
            }
        }
    }
}

void scanParts(const vfs::VirtualFileSystem& vfs, const ContentOwnerFn& owner, Composition& composition) {
    for (const std::string& carMod : listSorted(vfs, std::string(kPartsDir))) {
        const std::string carModDir = std::string(kPartsDir) + "/" + carMod;
        if (!vfs.isDirectory(carModDir)) {
            continue;
        }
        for (const std::string& car : listSorted(vfs, carModDir)) {
            const std::string carDir = carModDir + "/" + car;
            if (!vfs.isDirectory(carDir)) {
                continue;
            }
            for (const std::string& slot : listSorted(vfs, carDir)) {
                const std::string slotDir = carDir + "/" + slot;
                if (!vfs.isDirectory(slotDir)) {
                    continue;
                }
                for (const std::string& file : listSorted(vfs, slotDir)) {
                    const std::string path = slotDir + "/" + file;
                    if (vfs.isDirectory(path) || path.size() < 5 || path.compare(path.size() - 5, 5, ".toml") != 0) {
                        continue;
                    }
                    const std::string partId = file.substr(0, file.size() - 5);
                    const auto text = vfs.readText(path);
                    if (!text) {
                        composition.errors.push_back(path + ": cannot read the part");
                        continue;
                    }
                    Part part;
                    std::string error;
                    const std::string carRef = carMod + "/" + car;
                    if (parsePart(*text, path, owner ? owner(path) : std::string{}, carRef, slot,
                                  carRef + "/" + slot + "/" + partId, partId, error, part)) {
                        composition.parts.push_back(std::move(part));
                    } else {
                        composition.errors.push_back(path + ": " + error);
                    }
                }
            }
        }
    }
}

std::vector<std::string> dedupe(const std::vector<std::string>& values) {
    std::vector<std::string> result;
    for (const std::string& value : values) {
        if (std::find(result.begin(), result.end(), value) == result.end()) {
            result.push_back(value);
        }
    }
    return result;
}

std::string join(const std::vector<std::string>& values) {
    std::string result;
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i != 0) {
            result += ", ";
        }
        result += values[i].empty() ? "(unknown)" : values[i];
    }
    return result;
}

// Records one conflict when more than one distinct mod claims `subject`.
void addConflict(Composition& composition, std::string kind, const std::string& subject,
                 const std::vector<std::string>& mods, const std::string& what) {
    const std::vector<std::string> distinct = dedupe(mods);
    if (distinct.size() < 2) {
        return;
    }
    CompositionConflict conflict;
    conflict.kind = std::move(kind);
    conflict.subject = subject;
    conflict.mods = distinct;
    conflict.message = join(distinct) + " all " + what + " '" + subject + "'; the last one in load order wins";
    composition.conflicts.push_back(std::move(conflict));
}

bool applyEntry(toml::table& root, const PatchEntry& entry, std::string& error) {
    std::vector<std::string> keys;
    if (!splitPath(entry.path, keys)) {
        error = "invalid path";
        return false;
    }

    toml::table* current = &root;
    for (std::size_t i = 0; i + 1 < keys.size(); ++i) {
        toml::node* node = current->get(keys[i]);
        if (node == nullptr) {
            if (entry.op != PatchOp::Set) {
                error = "no such key";
                return false;
            }
            current->insert(keys[i], toml::table{});
            node = current->get(keys[i]);
        }
        toml::table* child = node != nullptr ? node->as_table() : nullptr;
        if (child == nullptr) {
            error = "'" + keys[i] + "' is not a table";
            return false;
        }
        current = child;
    }

    const std::string& leaf = keys.back();
    if (entry.op == PatchOp::Remove) {
        if (!current->contains(leaf)) {
            error = "no such key";
            return false;
        }
        current->erase(leaf);
        return true;
    }
    if (entry.op == PatchOp::Set) {
        if (entry.value.kind == PatchValue::Kind::Number) {
            current->insert_or_assign(leaf, entry.value.number);
        } else if (entry.value.kind == PatchValue::Kind::String) {
            current->insert_or_assign(leaf, entry.value.text);
        } else {
            error = "'set' needs a value";
            return false;
        }
        return true;
    }

    if (entry.value.kind != PatchValue::Kind::Number) {
        error = std::string("'") + opName(entry.op) + "' needs a numeric value";
        return false;
    }
    toml::node* node = current->get(leaf);
    if (node == nullptr) {
        error = "no such key";
        return false;
    }
    const auto currentValue = node->value<double>();
    if (!currentValue) {
        error = "the current value is not a number";
        return false;
    }
    const double next =
        entry.op == PatchOp::Add ? *currentValue + entry.value.number : *currentValue * entry.value.number;
    current->insert_or_assign(leaf, next);
    return true;
}

} // namespace

std::optional<Patch> parsePatchText(std::string_view text, std::string_view source, std::string_view modId,
                                    std::string& error) {
    Patch patch;
    if (!parsePatch(text, std::string(source), std::string(modId), error, patch)) {
        return std::nullopt;
    }
    return patch;
}

std::vector<const Patch*> Composition::patchesFor(std::string_view ref) const {
    std::vector<const Patch*> result;
    for (const Patch& patch : patches) {
        if (patch.target == ref) {
            result.push_back(&patch);
        }
    }
    return result;
}

void applyPatches(const std::vector<const Patch*>& patches, toml::table& table, std::vector<std::string>* errors) {
    for (const Patch* patch : patches) {
        for (const PatchEntry& entry : patch->entries) {
            std::string error;
            if (!applyEntry(table, entry, error)) {
                if (errors != nullptr) {
                    errors->push_back(patch->source + ": patch '" + entry.path + "' failed: " + error);
                }
            }
        }
    }
}

void detectConflicts(Composition& composition, const std::vector<std::pair<std::string, std::string>>& overrides) {
    std::map<std::pair<std::string, std::string>, std::vector<std::string>> patchKeys;
    for (const Patch& patch : composition.patches) {
        for (const PatchEntry& entry : patch.entries) {
            patchKeys[{patch.target, entry.path}].push_back(patch.modId);
        }
    }
    for (const auto& [key, mods] : patchKeys) {
        addConflict(composition, "patch", key.first + " (" + key.second + ")", mods, "patch");
    }

    std::map<std::string, std::vector<std::string>> skinRefs;
    for (const Skin& skin : composition.skins) {
        skinRefs[skin.ref].push_back(skin.modId);
    }
    for (const auto& [ref, mods] : skinRefs) {
        addConflict(composition, "skin", ref, mods, "provide the skin");
    }

    std::map<std::string, std::vector<std::string>> partRefs;
    for (const Part& part : composition.parts) {
        partRefs[part.ref].push_back(part.modId);
    }
    for (const auto& [ref, mods] : partRefs) {
        addConflict(composition, "part", ref, mods, "provide the part");
    }

    std::map<std::string, std::vector<std::string>> overrideFiles;
    for (const auto& [modId, path] : overrides) {
        overrideFiles[path].push_back(modId);
    }
    for (const auto& [path, mods] : overrideFiles) {
        addConflict(composition, "override", path, mods, "override");
    }
}

Composition loadComposition(const vfs::VirtualFileSystem& vfs, const ContentOwnerFn& owner) {
    Composition composition;
    scanPatches(vfs, owner, composition);
    scanSkins(vfs, owner, composition);
    scanParts(vfs, owner, composition);
    detectConflicts(composition, {});
    return composition;
}

} // namespace sim::modding
