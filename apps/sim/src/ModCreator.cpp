// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "ModCreator.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>
#include <system_error>
#include <vector>

#include "Paths.hpp"
#include "core/Config.hpp"
#include "core/Sha256.hpp"
#include "modding/Collection.hpp"
#include "modding/Composition.hpp"
#include "modding/Manifest.hpp"
#include "modding/Package.hpp"
#include "modding/PackageWriter.hpp"
#include "modding/Registry.hpp"
#include "session/CarLoader.hpp"
#include "session/TrackLoader.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace sim::app {
namespace {

namespace fs = std::filesystem;

std::string readTextFile(const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return {};
    }
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

bool writeTextFile(const fs::path& path, const std::string& text, std::string& error) {
    std::error_code ec;
    if (!path.parent_path().empty()) {
        fs::create_directories(path.parent_path(), ec);
    }
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) {
        error = "cannot write '" + path.string() + "'";
        return false;
    }
    out << text;
    return static_cast<bool>(out);
}

// The folder that holds `mod.toml`, whether the user pointed at the package or
// at the folder that contains it.
fs::path findPackageRoot(const fs::path& path) {
    std::error_code ec;
    if (!fs::is_directory(path, ec)) {
        return {};
    }
    if (fs::exists(path / modding::kManifestName, ec)) {
        return path;
    }
    for (const fs::directory_iterator::value_type& entry : fs::directory_iterator(path, ec)) {
        if (entry.is_directory(ec) && fs::exists(entry.path() / modding::kManifestName, ec)) {
            return entry.path();
        }
    }
    return {};
}

std::string upperFirst(std::string text) {
    if (!text.empty()) {
        text[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(text[0])));
    }
    return text;
}

// Replaces `id = "..."` and `name = "..."` in a copied descriptor, so a car
// scaffolded from another one does not keep the other one's identity.
std::string retargetDescriptor(const std::string& text, const std::string& id, const std::string& name) {
    std::string result;
    std::size_t start = 0;
    while (start <= text.size()) {
        const std::size_t end = text.find('\n', start);
        const std::string line = text.substr(start, end == std::string::npos ? std::string::npos : end - start);
        const std::size_t equals = line.find('=');
        if (equals != std::string::npos && line.find('[') == std::string::npos) {
            const std::string key = [&line, equals] {
                std::string text = line.substr(0, equals);
                while (!text.empty() && (text.back() == ' ' || text.back() == '\t')) {
                    text.pop_back();
                }
                return text;
            }();
            if (key == "id") {
                result += "id = \"" + id + "\"";
            } else if (key == "name") {
                result += "name = \"" + name + "\"";
            } else {
                result += line;
            }
        } else {
            result += line;
        }
        if (end == std::string::npos) {
            break;
        }
        result += '\n';
        start = end + 1;
    }
    return result;
}

// A complete, valid car: it validates on its own and drives like the base car
// until the author changes something.
std::string carScaffold(const std::string& id, const std::string& name) {
    return "# " + name +
           " - scaffolded by `sim mod new`. All values are SI: metres,\n"
           "# kilograms, radians, seconds; engine speeds are rad/s. Every key is optional;\n"
           "# delete what you do not need. See docs/MOD_FORMAT.md and docs/schemas/.\n"
           "\n"
           "id = \"" +
           id +
           "\"\n"
           "name = \"" +
           name +
           "\"\n"
           "author = \"Your name\"\n"
           "license = \"CC-BY-SA-4.0\"\n"
           "category = \"Data-defined car\"\n"
           "description = \"\"\n"
           "color = [0.8, 0.2, 0.2]\n"
           "\n"
           "[chassis]\n"
           "mass = 1250.0\n"
           "inertia = [1900.0, 2100.0, 500.0]   # pitch (X), yaw (Y), roll (Z)\n"
           "half_extents = [0.88, 0.35, 2.15]\n"
           "\n"
           "[geometry]\n"
           "front_axle_z = -1.25                # body -Z is forward\n"
           "rear_axle_z = 1.35\n"
           "half_track = 0.775\n"
           "mount_y = 0.0\n"
           "rest_length = 0.25\n"
           "\n"
           "[suspension]\n"
           "spring_stiffness_front = 38000.0\n"
           "spring_stiffness_rear = 34000.0\n"
           "damper_bump = 2800.0\n"
           "damper_rebound = 3600.0\n"
           "bump_stop_length = 0.06\n"
           "bump_stop_stiffness = 200000.0\n"
           "bump_stop_damping = 800.0\n"
           "bump_stop_exponent = 2.0\n"
           "\n"
           "[wheels]\n"
           "radius = 0.32\n"
           "inertia = 1.2                       # wheel + tyre + brake disc\n"
           "\n"
           "[tyres]\n"
           "peak_friction_longitudinal = 1.4\n"
           "peak_friction_lateral = 1.4\n"
           "nominal_load = 4000.0\n"
           "\n"
           "[engine]\n"
           "idle_speed = 90.0                   # rad/s\n"
           "rev_limit = 700.0\n"
           "torque_curve_speed = [100.0, 200.0, 300.0, 450.0, 600.0, 700.0]\n"
           "torque_curve_torque = [170.0, 215.0, 240.0, 250.0, 230.0, 190.0]   # N*m at full throttle\n"
           "friction_constant = 10.0\n"
           "friction_linear = 0.05\n"
           "\n"
           "[drivetrain]\n"
           "gear_ratios = [3.20, 2.15, 1.55, 1.20, 0.97, 0.80]   # strictly decreasing\n"
           "reverse_ratio = 3.30\n"
           "final_drive = 3.60\n"
           "efficiency = 0.92\n"
           "upshift_speed = 650.0\n"
           "downshift_speed = 250.0\n"
           "\n"
           "[brakes]\n"
           "max_brake_pressure = 1.40e7          # Pa at full pedal\n"
           "brake_bias = 0.70                    # front fraction\n"
           "handbrake_pressure = 3.0e6\n"
           "pad_friction = 0.40\n"
           "piston_area = 0.002\n"
           "effective_radius = 0.13\n"
           "\n"
           "[steering]\n"
           "max_steer_angle = 0.55               # rad at a standstill\n"
           "min_steer_angle = 0.10\n"
           "steer_speed_scale = 14.0\n"
           "steer_input_rate = 3.0\n"
           "steer_return_rate = 5.0\n"
           "\n"
           "[aero]\n"
           "drag_coefficient = 0.30\n"
           "lift_coefficient = 0.0               # + = downforce\n"
           "frontal_area = 2.1\n"
           "centre_of_pressure = [0.0, 0.4, 0.0]\n"
           "air_density = 1.225\n"
           "\n"
           "[model]\n"
           "# mesh = \"model/car.glb\"            # optional glTF/KTX2 model\n";
}

std::string trackScaffold(const std::string& id, const std::string& name) {
    return "# " + name +
           " - scaffolded by `sim mod new`. All values are SI: metres.\n"
           "# The centreline is a closed centripetal Catmull-Rom spline through the\n"
           "# control points, resampled by arc length. See docs/MOD_FORMAT.md.\n"
           "\n"
           "id = \"" +
           id +
           "\"\n"
           "name = \"" +
           name +
           "\"\n"
           "author = \"Your name\"\n"
           "license = \"CC-BY-SA-4.0\"\n"
           "location = \"\"\n"
           "description = \"\"\n"
           "\n"
           "[geometry]\n"
           "loop = true\n"
           "sample_spacing = 1.0        # metres between resampled centreline points\n"
           "half_width = 5.0            # asphalt half-width\n"
           "kerb_width = 1.0\n"
           "start_z = 40.0              # z of the start/finish line\n"
           "\n"
           "# Control points in the direction of travel, right-handed: +X right,\n"
           "# +Y up, -Z forward. The car starts on the straight heading -Z.\n"
           "control_points = [\n"
           "    [0.0, 120.0],\n"
           "    [0.0, 20.0],\n"
           "    [0.0, -90.0],\n"
           "    [60.0, -160.0],\n"
           "    [120.0, -100.0],\n"
           "    [120.0, 60.0],\n"
           "    [60.0, 150.0],\n"
           "    [10.0, 180.0],\n"
           "]\n"
           "\n"
           "[model]\n"
           "# mesh = \"model/track.glb\"\n";
}

std::string skinScaffold(const std::string& id, const std::string& car) {
    return "# " + id + " - a livery for '" + car +
           "'. A skin only adds: the car it dresses is\n"
           "# never copied, and another mod may ship more skins for the same car.\n"
           "\n"
           "id = \"" +
           id +
           "\"\n"
           "name = \"" +
           id +
           "\"\n"
           "description = \"\"\n"
           "car = \"" +
           car +
           "\"                    # must match the folder\n"
           "body_color = [0.8, 0.2, 0.2]   # optional, three or four numbers in [0, 1]\n"
           "\n"
           "# A texture the skin replaces (the file sits next to this one).\n"
           "# [[textures]]\n"
           "# slot = \"livery\"\n"
           "# file = \"livery.png\"\n";
}

std::string patchScaffold(const std::string& target) {
    return "# A patch edits another mod's descriptor without copying it, so an\n"
           "# update to that mod does not undo your change. See docs/MOD_FORMAT.md.\n"
           "\n"
           "target = \"" +
           target +
           "\"\n"
           "\n"
           "# [[change]]\n"
           "# path = \"suspension.spring_stiffness_front\"   # dotted key inside the target\n"
           "# op = \"multiply\"                             # set | add | multiply | remove\n"
           "# value = 0.85\n";
}

std::string modManifest(const std::string& id, const std::string& name, const std::string& type) {
    return "# " + name +
           " - scaffolded by `sim mod new`.\n"
           "# `format = 2` and a namespaced `id` (<author>.<name>) are required.\n"
           "\n"
           "format = 2\n"
           "\n"
           "id = \"" +
           id +
           "\"\n"
           "name = \"" +
           name +
           "\"\n"
           "version = \"0.1.0\"\n"
           "authors = [\"Your name\"]\n"
           "license = \"CC-BY-SA-4.0\"\n"
           "description = \"\"\n"
           "\n"
           "types = [\"" +
           type +
           "\"]\n"
           "\n"
           "# dependencies = [\"base.core >= 0.1.0\"]\n"
           "# conflicts = []\n"
           "# permissions = []          # only what the scripts need\n"
           "# website = \"\"\n"
           "# changelog = \"\"\n";
}

// The virtual path of a descriptor inside a package, for `--from`.
std::string descriptorPathFor(const std::string& type, const ContentRef& ref) {
    if (type == "car") {
        return std::string(modding::kCarsDir) + "/" + ref.id + "/car.toml";
    }
    if (type == "track") {
        return std::string(modding::kTracksDir) + "/" + ref.id + "/track.toml";
    }
    return {};
}

void reportIssues(const std::vector<std::string>& errors, const std::vector<std::string>& warnings) {
    for (const std::string& warning : warnings) {
        std::printf("  warning: %s\n", warning.c_str());
    }
    for (const std::string& error : errors) {
        std::printf("  error: %s\n", error.c_str());
    }
}

} // namespace

ContentRef splitRef(const std::string& ref) {
    const std::size_t slash = ref.find('/');
    if (slash == std::string::npos) {
        return {"", ref};
    }
    return {ref.substr(0, slash), ref.substr(slash + 1)};
}

int createMod(const ModNewOptions& options) {
    const ContentRef target = splitRef(options.target);
    if (target.modId.empty() || target.id.empty()) {
        std::fprintf(stderr, "expected '<mod-id>/<name>', for example ana.karts/kart (got '%s')\n",
                     options.target.c_str());
        return 2;
    }
    if (!modding::isValidModId(target.modId)) {
        std::fprintf(stderr, "'%s' is not a valid mod id: it is <author>.<name>, lower-case letters, digits and '-'\n",
                     target.modId.c_str());
        return 2;
    }
    if (!modding::isValidContentId(target.id)) {
        std::fprintf(stderr, "'%s' is not a valid content id: lower-case letters, digits, '_' and '-'\n",
                     target.id.c_str());
        return 2;
    }
    const std::string type = options.type;
    if (type != "car" && type != "track" && type != "skin" && type != "patch") {
        std::fprintf(stderr, "unknown content type '%s' (car, track, skin or patch)\n", type.c_str());
        return 2;
    }

    const std::filesystem::path root = locateRootDirectory();
    core::Config config(root / "config.toml");
    config.load();

    const fs::path directory =
        options.output.empty() ? locateModsDirectory(config) / target.modId : fs::path(options.output);
    std::error_code ec;
    if (fs::exists(directory, ec) && !fs::is_empty(directory, ec)) {
        std::fprintf(stderr, "'%s' already exists and is not empty; refusing to overwrite it\n",
                     directory.string().c_str());
        return 1;
    }

    // `--from` copies a descriptor of an existing mod; otherwise a complete,
    // already-valid scaffold is used.
    std::string body;
    std::string fromPath;
    if (!options.from.empty()) {
        const ContentRef source = splitRef(options.from);
        fromPath = descriptorPathFor(type, source);
        if (fromPath.empty()) {
            std::fprintf(stderr, "'--from' cannot be used with a %s\n", type.c_str());
            return 2;
        }
        vfs::VirtualFileSystem vfs;
        vfs.mount(root / "content" / "base", "/");
        const fs::path modsDirectory = locateModsDirectory(config);
        if (fs::is_directory(modsDirectory, ec)) {
            vfs.mount(modsDirectory, "/");
        }
        const auto text = vfs.readText(fromPath);
        if (!text) {
            std::fprintf(stderr, "cannot read '%s' (is '%s' installed?)\n", fromPath.c_str(), options.from.c_str());
            return 2;
        }
        body = retargetDescriptor(*text, target.id, upperFirst(target.id));
    }

    std::string error;
    fs::path descriptor;
    std::string scaffold;
    if (type == "car") {
        descriptor = directory / modding::kCarsDir / target.id / "car.toml";
        scaffold = carScaffold(target.id, upperFirst(target.id));
    } else if (type == "track") {
        descriptor = directory / modding::kTracksDir / target.id / "track.toml";
        scaffold = trackScaffold(target.id, upperFirst(target.id));
    } else if (type == "patch") {
        if (options.forCar.empty()) {
            std::fprintf(stderr, "a patch needs '--for <mod>/<content>' to say what it targets\n");
            return 2;
        }
        descriptor = directory / modding::kPatchesDir / (target.id + ".toml");
        scaffold = patchScaffold(options.forCar);
    } else {
        if (options.forCar.empty()) {
            std::fprintf(stderr, "a skin needs '--for <car-mod>/<car>' to say which car it dresses\n");
            return 2;
        }
        const ContentRef car = splitRef(options.forCar);
        if (car.modId.empty() || car.id.empty()) {
            std::fprintf(stderr, "'--for' must be '<car-mod>/<car>', for example base.core/voyager\n");
            return 2;
        }
        descriptor = directory / modding::kSkinsDir / car.modId / car.id / target.id / "livery.toml";
        scaffold = skinScaffold(target.id, options.forCar);
    }

    if (!writeTextFile(directory / modding::kManifestName, modManifest(target.modId, upperFirst(target.id), type),
                       error) ||
        !writeTextFile(descriptor, body.empty() ? scaffold : body, error)) {
        std::fprintf(stderr, "%s\n", error.c_str());
        return 1;
    }

    std::printf("created %s\n", directory.generic_string().c_str());
    std::printf("  %s\n", (directory / modding::kManifestName).generic_string().c_str());
    std::printf("  %s%s\n", descriptor.generic_string().c_str(), body.empty() ? "" : "  (copied from the source)");
    if (!body.empty()) {
        std::printf("note: the id and the name were changed; review the rest of the values.\n");
    }
    std::printf("next: sim mod validate \"%s\" && sim mod pack \"%s\"\n", directory.generic_string().c_str(),
                directory.generic_string().c_str());
    return 0;
}

int validateMod(const std::string& path) {
    const fs::path payload = findPackageRoot(path);
    if (payload.empty()) {
        std::fprintf(stderr, "'%s' has no %s\n", path.c_str(), std::string(modding::kManifestName).c_str());
        return 2;
    }

    std::vector<std::string> errors;
    std::vector<std::string> warnings;

    const std::string manifestText = readTextFile(payload / modding::kManifestName);
    const modding::ManifestResult manifest = modding::parseManifest(manifestText, payload.filename().string());
    for (const std::string& warning : manifest.warnings) {
        warnings.push_back(warning);
    }
    if (!manifest.ok()) {
        for (const std::string& error : manifest.errors) {
            errors.push_back(error);
        }
        std::printf("%s\n", payload.string().c_str());
        reportIssues(errors, warnings);
        return 1;
    }

    // Everything a package may contain at the top level (docs/MOD_FORMAT.md).
    const std::set<std::string> knownFolders = {
        std::string(modding::kCarsDir),   std::string(modding::kTracksDir),  std::string(modding::kSkinsDir),
        std::string(modding::kPartsDir),  std::string(modding::kTyresDir),   std::string(modding::kSetupsDir),
        std::string(modding::kSoundsDir), std::string(modding::kWeatherDir), std::string(modding::kAppsDir),
        std::string(modding::kModesDir),  std::string(modding::kVisualsDir), std::string(modding::kLocaleDir),
        std::string(modding::kInputDir),  std::string(modding::kPluginsDir), std::string(modding::kPatchesDir),
    };
    const std::set<std::string> knownFiles = {
        std::string(modding::kManifestName),
        std::string(modding::kCollectionName),
        "README.md",
        "README.txt",
        "LICENSE",
        "LICENSE.md",
        "cover.png",
        "cover.jpg",
    };

    std::error_code ec;
    for (const fs::directory_iterator::value_type& entry : fs::directory_iterator(payload, ec)) {
        const std::string name = entry.path().filename().string();
        if (entry.is_directory(ec)) {
            if (!knownFolders.contains(name)) {
                warnings.push_back("'" + name + "' is not a folder the game reads");
            }
            continue;
        }
        if (!knownFiles.contains(name)) {
            warnings.push_back("'" + name + "' is not a file the game reads");
        }
    }

    std::size_t files = 0;
    for (const fs::recursive_directory_iterator::value_type& entry : fs::recursive_directory_iterator(payload, ec)) {
        const std::string relative = entry.path().lexically_relative(payload).generic_string();
        if (entry.is_directory(ec)) {
            if (!modding::isSafePackagePath(relative)) {
                errors.push_back("unsafe path '" + relative + "'");
            }
            continue;
        }
        ++files;
        if (!modding::isSafePackagePath(relative)) {
            errors.push_back("unsafe path '" + relative + "'");
        }
        if (modding::isNativeBinaryPath(relative)) {
            errors.push_back("'" + relative + "' is a native binary; only plugins/ may ship one");
        }
    }

    // The game's own parsers, on a scratch mount of this package.
    vfs::VirtualFileSystem vfs;
    vfs.mount(payload, "/");
    const CarLoadResult cars = loadCars(vfs);
    const TrackLoadResult tracks = loadTracks(vfs);
    const modding::Composition composition = modding::loadComposition(vfs);
    for (const std::string& error : cars.errors) {
        errors.push_back(error);
    }
    for (const std::string& error : tracks.errors) {
        errors.push_back(error);
    }
    for (const std::string& error : composition.errors) {
        errors.push_back(error);
    }
    for (const modding::CompositionConflict& conflict : composition.conflicts) {
        warnings.push_back(conflict.message);
    }

    if (vfs.exists(std::string(modding::kCollectionName))) {
        const auto text = vfs.readText(std::string(modding::kCollectionName));
        const modding::CollectionResult collection = modding::parseCollection(text ? *text : "", "collection.toml");
        for (const std::string& warning : collection.warnings) {
            warnings.push_back(warning);
        }
        for (const std::string& error : collection.errors) {
            errors.push_back(error);
        }
    }

    std::printf("%s: %s %s (%s)\n", payload.string().c_str(), manifest.manifest->name.c_str(),
                manifest.manifest->version.toString().c_str(), manifest.manifest->id.c_str());
    std::printf("  %zu file(s), %zu car(s), %zu track(s), %zu skin(s), %zu part(s), %zu patch(es)\n", files,
                cars.cars.size(), tracks.tracks.size(), composition.skins.size(), composition.parts.size(),
                composition.patches.size());
    reportIssues(errors, warnings);
    if (!errors.empty()) {
        std::printf("invalid: %zu error(s)\n", errors.size());
        return 1;
    }
    std::printf("ok\n");
    return 0;
}

int packMod(const std::string& path, const std::string& output) {
    const fs::path payload = findPackageRoot(path);
    if (payload.empty()) {
        std::fprintf(stderr, "'%s' has no %s\n", path.c_str(), std::string(modding::kManifestName).c_str());
        return 2;
    }

    const std::string manifestText = readTextFile(payload / modding::kManifestName);
    const modding::ManifestResult manifest = modding::parseManifest(manifestText, payload.filename().string());
    if (!manifest.ok()) {
        for (const std::string& error : manifest.errors) {
            std::fprintf(stderr, "%s\n", error.c_str());
        }
        return 1;
    }

    const fs::path destination =
        output.empty() ? fs::current_path() / (manifest.manifest->id + "-" + manifest.manifest->version.toString() +
                                               std::string(modding::kPackageExtension))
                       : fs::path(output);

    std::string error;
    if (!modding::writePackage(payload, destination, error)) {
        std::fprintf(stderr, "%s\n", error.c_str());
        return 1;
    }

    std::string digestError;
    const std::string digest = core::sha256File(destination, digestError);
    if (digest.empty()) {
        std::fprintf(stderr, "%s\n", digestError.c_str());
        return 1;
    }

    std::error_code sizeError;
    const auto size = static_cast<unsigned long long>(fs::file_size(destination, sizeError));

    // Everything the index needs, ready to paste (docs/MODS.md section 6).
    std::printf("packed %s\n", destination.generic_string().c_str());
    std::printf("  %llu bytes, sha256 %s\n", size, digest.c_str());
    std::printf("\n[[version]]\n");
    std::printf("id = \"%s\"\n", manifest.manifest->id.c_str());
    std::printf("version = \"%s\"\n", manifest.manifest->version.toString().c_str());
    std::printf("download = \"<where you host it>/%s\"\n", destination.filename().string().c_str());
    std::printf("sha256 = \"%s\"\n", digest.c_str());
    std::printf("size = %llu\n", size);
    if (!manifest.manifest->license.empty()) {
        std::printf("license = \"%s\"\n", manifest.manifest->license.c_str());
    }
    return 0;
}

} // namespace sim::app
