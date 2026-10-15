// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "input/CompatibilityDatabase.hpp"

#include <array>
#include <cctype>
#include <cstring>
#include <string>

namespace sim::input {

namespace {

bool containsIgnoreCase(const std::string& haystack, const char* needle) {
    if (needle == nullptr || needle[0] == '\0') {
        return true; // null hint = match any name of this vendor
    }
    const std::size_t n = std::strlen(needle);
    if (n == 0 || haystack.size() < n) {
        return false;
    }
    for (std::size_t i = 0; i + n <= haystack.size(); ++i) {
        bool match = true;
        for (std::size_t j = 0; j < n; ++j) {
            const auto a = static_cast<unsigned char>(haystack[i + j]);
            const auto b = static_cast<unsigned char>(needle[j]);
            if (std::tolower(a) != std::tolower(b)) {
                match = false;
                break;
            }
        }
        if (match) {
            return true;
        }
    }
    return false;
}

constexpr FfbCapabilities wheelFfb() {
    FfbCapabilities ffb;
    ffb.constantForce = true;
    ffb.spring = true;
    ffb.damper = true;
    return ffb;
}

constexpr FfbCapabilities rumbleOnly() {
    FfbCapabilities ffb;
    ffb.rumble = true;
    return ffb;
}

// Logitech wheels expose their effects through the generic DirectInput force
// feedback path, which SDL forwards on Windows. PIDs come from the public SDL
// game controller database and the Linux kernel HID tables.
constexpr DeviceModel kLogitechMomo = {0x046D,
                                       0xC295,
                                       nullptr,
                                       "Logitech",
                                       "MOMO Force Feedback",
                                       DeviceKind::WheelBase,
                                       wheelFfb(),
                                       false,
                                       VerificationStatus::CommunityReported,
                                       "SDL gamecontrollerdb",
                                       "WingMan MOMO; no clutch pedal."};
constexpr DeviceModel kLogitechDrivingForce = {0x046D,
                                               0xC294,
                                               nullptr,
                                               "Logitech",
                                               "Driving Force",
                                               DeviceKind::WheelBase,
                                               wheelFfb(),
                                               false,
                                               VerificationStatus::CommunityReported,
                                               "SDL gamecontrollerdb",
                                               ""};
constexpr DeviceModel kLogitechDrivingForcePro = {0x046D,
                                                  0xC298,
                                                  nullptr,
                                                  "Logitech",
                                                  "Driving Force Pro",
                                                  DeviceKind::WheelBase,
                                                  wheelFfb(),
                                                  false,
                                                  VerificationStatus::CommunityReported,
                                                  "SDL gamecontrollerdb",
                                                  ""};
constexpr DeviceModel kLogitechG25 = {0x046D,
                                      0xC299,
                                      nullptr,
                                      "Logitech",
                                      "G25 Racing Wheel",
                                      DeviceKind::WheelBase,
                                      wheelFfb(),
                                      false,
                                      VerificationStatus::CommunityReported,
                                      "SDL gamecontrollerdb",
                                      "H-pattern shifter plus sequential; clutch on a separate axis."};
constexpr DeviceModel kLogitechDrivingForceGT = {0x046D,
                                                 0xC29A,
                                                 nullptr,
                                                 "Logitech",
                                                 "Driving Force GT",
                                                 DeviceKind::WheelBase,
                                                 wheelFfb(),
                                                 false,
                                                 VerificationStatus::CommunityReported,
                                                 "SDL gamecontrollerdb",
                                                 ""};
constexpr DeviceModel kLogitechG27 = {0x046D,
                                      0xC29B,
                                      nullptr,
                                      "Logitech",
                                      "G27 Racing Wheel",
                                      DeviceKind::WheelBase,
                                      wheelFfb(),
                                      false,
                                      VerificationStatus::CommunityReported,
                                      "SDL gamecontrollerdb",
                                      "H-pattern shifter plus sequential; clutch on a separate axis."};
constexpr DeviceModel kLogitechG29 = {0x046D,
                                      0xC24F,
                                      nullptr,
                                      "Logitech",
                                      "G29 Driving Force",
                                      DeviceKind::WheelBase,
                                      wheelFfb(),
                                      false,
                                      VerificationStatus::CommunityReported,
                                      "SDL gamecontrollerdb",
                                      "PS/PC variant. Xbox variant is the G920."};
constexpr DeviceModel kLogitechG920 = {0x046D,
                                       0xC262,
                                       nullptr,
                                       "Logitech",
                                       "G920 Driving Force",
                                       DeviceKind::WheelBase,
                                       wheelFfb(),
                                       false,
                                       VerificationStatus::CommunityReported,
                                       "SDL gamecontrollerdb",
                                       "Xbox/PC variant of the G29."};
constexpr DeviceModel kLogitechG923 = {
    0x046D,
    0xC266,
    nullptr,
    "Logitech",
    "G923 Racing Wheel",
    DeviceKind::WheelBase,
    wheelFfb(),
    false,
    VerificationStatus::CommunityReported,
    "SDL gamecontrollerdb",
    "Xbox/PC variant; TrueForce haptics are vendor-proprietary and not driven here."};
constexpr DeviceModel kLogitechG923Ps = {
    0x046D,
    0xC267,
    nullptr,
    "Logitech",
    "G923 Racing Wheel",
    DeviceKind::WheelBase,
    wheelFfb(),
    false,
    VerificationStatus::CommunityReported,
    "SDL gamecontrollerdb",
    "PS/PC variant; TrueForce haptics are vendor-proprietary and not driven here."};
constexpr DeviceModel kLogitechGPro = {0x046D,
                                       0xC272,
                                       nullptr,
                                       "Logitech",
                                       "G Pro Racing Wheel",
                                       DeviceKind::WheelBase,
                                       wheelFfb(),
                                       false,
                                       VerificationStatus::CommunityReported,
                                       "SDL gamecontrollerdb",
                                       "Direct-drive; vendor tuning software recommended."};

// Thrustmaster wheels (VID 0x044F).
constexpr DeviceModel kTMDrivingForce = {0x044F,
                                         0xB65D,
                                         nullptr,
                                         "Thrustmaster",
                                         "T500 RS",
                                         DeviceKind::WheelBase,
                                         wheelFfb(),
                                         false,
                                         VerificationStatus::CommunityReported,
                                         "SDL gamecontrollerdb",
                                         ""};
constexpr DeviceModel kTMClutch458 = {0x044F,
                                      0xB660,
                                      nullptr,
                                      "Thrustmaster",
                                      "Ferrari 458 Italia",
                                      DeviceKind::WheelBase,
                                      wheelFfb(),
                                      false,
                                      VerificationStatus::CommunityReported,
                                      "SDL gamecontrollerdb",
                                      ""};
constexpr DeviceModel kTMTX = {0x044F,
                               0xB669,
                               nullptr,
                               "Thrustmaster",
                               "TX Racing Wheel",
                               DeviceKind::WheelBase,
                               wheelFfb(),
                               false,
                               VerificationStatus::CommunityReported,
                               "SDL gamecontrollerdb",
                               ""};
constexpr DeviceModel kTMT300 = {0x044F,
                                 0xB66D,
                                 nullptr,
                                 "Thrustmaster",
                                 "T300 RS",
                                 DeviceKind::WheelBase,
                                 wheelFfb(),
                                 false,
                                 VerificationStatus::CommunityReported,
                                 "SDL gamecontrollerdb",
                                 "PS/PC. The TX is the Xbox equivalent."};
constexpr DeviceModel kTMT150 = {0x044F,
                                 0xB677,
                                 nullptr,
                                 "Thrustmaster",
                                 "T150",
                                 DeviceKind::WheelBase,
                                 wheelFfb(),
                                 false,
                                 VerificationStatus::CommunityReported,
                                 "SDL gamecontrollerdb",
                                 ""};
constexpr DeviceModel kTMTGT = {0x044F,
                                0xB67D,
                                nullptr,
                                "Thrustmaster",
                                "T-GT",
                                DeviceKind::WheelBase,
                                wheelFfb(),
                                false,
                                VerificationStatus::CommunityReported,
                                "SDL gamecontrollerdb",
                                ""};
constexpr DeviceModel kTMTMX = {0x044F,
                                0xB67F,
                                nullptr,
                                "Thrustmaster",
                                "TMX Force Feedback",
                                DeviceKind::WheelBase,
                                wheelFfb(),
                                false,
                                VerificationStatus::CommunityReported,
                                "SDL gamecontrollerdb",
                                "Xbox/PC equivalent of the T150."};
constexpr DeviceModel kTMTSpc = {0x044F,
                                 0xB689,
                                 nullptr,
                                 "Thrustmaster",
                                 "TS-PC Racer",
                                 DeviceKind::WheelBase,
                                 wheelFfb(),
                                 false,
                                 VerificationStatus::CommunityReported,
                                 "SDL gamecontrollerdb",
                                 ""};
constexpr DeviceModel kTMTSxw = {0x044F,
                                 0xB692,
                                 nullptr,
                                 "Thrustmaster",
                                 "TS-XW Racer",
                                 DeviceKind::WheelBase,
                                 wheelFfb(),
                                 false,
                                 VerificationStatus::CommunityReported,
                                 "SDL gamecontrollerdb",
                                 ""};
constexpr DeviceModel kTMT248 = {0x044F,
                                 0xB696,
                                 nullptr,
                                 "Thrustmaster",
                                 "T248",
                                 DeviceKind::WheelBase,
                                 wheelFfb(),
                                 false,
                                 VerificationStatus::CommunityReported,
                                 "SDL gamecontrollerdb",
                                 ""};

// Vendor-level fallbacks for brands whose individual PIDs are not pinned here.
// These match any product of the vendor and only refine the manufacturer/kind;
// capabilities fall back to whatever the device itself reports.
constexpr DeviceModel kFanatecFamily = {0x0EB7,
                                        0x0000,
                                        nullptr,
                                        "Fanatec",
                                        "Fanatec wheel base",
                                        DeviceKind::WheelBase,
                                        wheelFfb(),
                                        true,
                                        VerificationStatus::CommunityReported,
                                        "SDL gamecontrollerdb",
                                        "CSL / ClubSport / Podium / DD families share VID 0x0EB7; individual PIDs are "
                                        "not pinned here, so treat the model name as unconfirmed."};
constexpr DeviceModel kSimucubeFamily = {0x16D0,
                                         0x0000,
                                         nullptr,
                                         "Simucube",
                                         "Simucube wheel base",
                                         DeviceKind::WheelBase,
                                         wheelFfb(),
                                         true,
                                         VerificationStatus::CommunityReported,
                                         "Linux hid-ids",
                                         "Granite Devices VID; configuration is normally done in Simucube software."};
constexpr DeviceModel kAsetekFamily = {0x2433,
                                       0x0000,
                                       nullptr,
                                       "Asetek SimSports",
                                       "Asetek wheel base",
                                       DeviceKind::WheelBase,
                                       wheelFfb(),
                                       true,
                                       VerificationStatus::CommunityReported,
                                       "Linux hid-ids",
                                       "Direct-drive; some features require Asetek RaceHub."};
constexpr DeviceModel kMozaFamily = {
    0x346E,
    0x0000,
    nullptr,
    "MOZA Racing",
    "MOZA wheel base",
    DeviceKind::WheelBase,
    wheelFfb(),
    true,
    VerificationStatus::CommunityReported,
    "Vendor / community reports",
    "R3/R5/R9/R12/R16/R21 and CS/GS/KS/ES wheels; Pit House software recommended. PIDs not pinned here."};
constexpr DeviceModel kHoriFamily = {0x0F0D,
                                     0x0000,
                                     nullptr,
                                     "Hori",
                                     "Hori racing controller",
                                     DeviceKind::WheelBase,
                                     FfbCapabilities{},
                                     false,
                                     VerificationStatus::CommunityReported,
                                     "SDL gamecontrollerdb",
                                     ""};
constexpr DeviceModel kPxnFamily = {0x11FF,
                                    0x0000,
                                    nullptr,
                                    "PXN",
                                    "PXN wheel",
                                    DeviceKind::WheelBase,
                                    rumbleOnly(),
                                    false,
                                    VerificationStatus::Unknown,
                                    "Community reports",
                                    "Vendor id is unconfirmed."};
constexpr DeviceModel kTurtleBeachFamily = {0x10F0,
                                            0x0000,
                                            nullptr,
                                            "Turtle Beach",
                                            "Turtle Beach wheel",
                                            DeviceKind::WheelBase,
                                            rumbleOnly(),
                                            false,
                                            VerificationStatus::Unknown,
                                            "Community reports",
                                            "Vendor id is unconfirmed."};
constexpr DeviceModel kVrsFamily = {0x0483,
                                    0x0000,
                                    "VRS",
                                    "VRS (Virtual Racing School)",
                                    "VRS DirectForce Pro",
                                    DeviceKind::WheelBase,
                                    wheelFfb(),
                                    true,
                                    VerificationStatus::CommunityReported,
                                    "Community reports",
                                    "STM32 vendor id shared with many boards; matched by name only."};

std::vector<DeviceModel> buildEntries() {
    return {
        kLogitechMomo,
        kLogitechDrivingForce,
        kLogitechDrivingForcePro,
        kLogitechG25,
        kLogitechDrivingForceGT,
        kLogitechG27,
        kLogitechG29,
        kLogitechG920,
        kLogitechG923,
        kLogitechG923Ps,
        kLogitechGPro,
        kTMDrivingForce,
        kTMClutch458,
        kTMTX,
        kTMT300,
        kTMT150,
        kTMTGT,
        kTMTMX,
        kTMTSpc,
        kTMTSxw,
        kTMT248,
        kFanatecFamily,
        kSimucubeFamily,
        kAsetekFamily,
        kMozaFamily,
        kHoriFamily,
        kPxnFamily,
        kTurtleBeachFamily,
        kVrsFamily,
    };
}

} // namespace

CompatibilityDatabase::CompatibilityDatabase() : entries_(buildEntries()) {}

CompatibilityDatabase& CompatibilityDatabase::instance() {
    static CompatibilityDatabase database;
    return database;
}

const char* CompatibilityDatabase::vendorName(std::uint16_t vendorId) {
    switch (vendorId) {
    case 0x046D:
        return "Logitech";
    case 0x044F:
        return "Thrustmaster";
    case 0x0EB7:
        return "Fanatec";
    case 0x16D0:
        return "Simucube";
    case 0x2433:
        return "Asetek SimSports";
    case 0x346E:
        return "MOZA Racing";
    case 0x0F0D:
        return "Hori";
    case 0x11FF:
        return "PXN";
    case 0x054C:
        return "Sony";
    case 0x045E:
        return "Microsoft";
    case 0x057E:
        return "Nintendo";
    case 0x28DE:
        return "Valve";
    default:
        return "";
    }
}

Identification CompatibilityDatabase::identify(const DeviceDescriptor& device) const {
    // 1. Exact VID/PID.
    for (const DeviceModel& entry : entries_) {
        if (entry.productId != 0 && entry.vendorId == device.vendorId && entry.productId == device.productId) {
            Identification id;
            id.model = &entry;
            id.kind = entry.kind;
            id.manufacturer = entry.manufacturer;
            id.modelName = entry.model;
            id.status = entry.status;
            id.source = entry.source;
            id.notes = entry.notes;
            id.declaredFfb = entry.ffb;
            id.requiresVendorDriver = entry.requiresVendorDriver;
            return id;
        }
    }

    // 2. Vendor match with a name hint (covers families whose PIDs move).
    for (const DeviceModel& entry : entries_) {
        if (entry.productId == 0 && entry.vendorId == device.vendorId &&
            containsIgnoreCase(device.name, entry.nameMatch)) {
            Identification id;
            id.model = &entry;
            id.kind = entry.kind;
            id.manufacturer = entry.manufacturer;
            id.modelName = device.name.empty() ? entry.model : device.name;
            id.status = entry.status;
            id.source = entry.source;
            id.notes = entry.notes;
            id.declaredFfb = entry.ffb;
            id.requiresVendorDriver = entry.requiresVendorDriver;
            return id;
        }
    }

    return classify(device);
}

Identification CompatibilityDatabase::classify(const DeviceDescriptor& device) const {
    Identification id;
    id.kind = device.isGamepad ? DeviceKind::Gamepad : DeviceKind::GenericHid;

    const std::string& name = device.name;
    if (containsIgnoreCase(name, "clutch") || containsIgnoreCase(name, "pedal")) {
        id.kind = DeviceKind::Pedals;
    } else if (containsIgnoreCase(name, "shifter") || containsIgnoreCase(name, "h-pattern") ||
               containsIgnoreCase(name, "sequential")) {
        id.kind = DeviceKind::Shifter;
    } else if (containsIgnoreCase(name, "handbrake")) {
        id.kind = DeviceKind::Handbrake;
    } else if (containsIgnoreCase(name, "wheel") || containsIgnoreCase(name, "racing") ||
               containsIgnoreCase(name, "drive force") || containsIgnoreCase(name, "force feedback") ||
               containsIgnoreCase(name, "ffb")) {
        id.kind = DeviceKind::WheelBase;
    }

    if (const char* vendor = vendorName(device.vendorId); vendor[0] != '\0') {
        id.manufacturer = vendor;
    } else if (!device.manufacturer.empty()) {
        id.manufacturer = device.manufacturer;
    } else {
        id.manufacturer = "Unknown";
    }
    id.modelName = name.empty() ? "Unknown device" : name;
    id.status = VerificationStatus::Unknown;
    id.source = "Runtime heuristics";
    id.notes = "Not present in the compatibility database.";
    return id;
}

} // namespace sim::input
