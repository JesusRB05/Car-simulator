// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "drive/Json.hpp"

#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>

namespace sim::app::json {
namespace {

constexpr int kMaxDepth = 64;

class Parser {
public:
    Parser(std::string_view text, std::string& error) : text_(text), error_(error) {}

    std::optional<Value> run() {
        skipWhitespace();
        std::optional<Value> value = parseValue(0);
        if (!value) {
            return std::nullopt;
        }
        skipWhitespace();
        if (pos_ != text_.size()) {
            fail("unexpected trailing character");
            return std::nullopt;
        }
        return value;
    }

    // Thrown to unwind to `parse()`, which turns it into a nullopt plus `error`.
    struct Failure {};

private:
    [[noreturn]] void fail(const std::string& what) {
        if (error_.empty()) {
            error_ = what + " at line " + std::to_string(line_) + ", column " + std::to_string(column_);
        }
        throw Failure{};
    }

    bool atEnd() const { return pos_ >= text_.size(); }
    char peek() const { return atEnd() ? '\0' : text_[pos_]; }

    char take() {
        const char c = text_[pos_++];
        if (c == '\n') {
            ++line_;
            column_ = 1;
        } else {
            ++column_;
        }
        return c;
    }

    void skipWhitespace() {
        while (!atEnd()) {
            const char c = peek();
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
                take();
            } else {
                break;
            }
        }
    }

    std::optional<Value> parseValue(int depth) {
        if (depth > kMaxDepth) {
            fail("nesting is too deep");
        }
        skipWhitespace();
        if (atEnd()) {
            fail("unexpected end of input");
        }
        switch (peek()) {
        case '{':
            return parseObject(depth);
        case '[':
            return parseArray(depth);
        case '"': {
            Value value;
            value.type = Value::Type::String;
            value.text = parseString();
            return value;
        }
        case 't':
            expect("true");
            return boolValue(true);
        case 'f':
            expect("false");
            return boolValue(false);
        case 'n':
            expect("null");
            return Value{};
        default:
            return parseNumber();
        }
    }

    static Value boolValue(bool value) {
        Value result;
        result.type = Value::Type::Bool;
        result.boolean = value;
        return result;
    }

    void expect(std::string_view literal) {
        if (text_.substr(pos_, literal.size()) != literal) {
            fail("expected '" + std::string(literal) + "'");
        }
        for (std::size_t i = 0; i < literal.size(); ++i) {
            take();
        }
    }

    std::optional<Value> parseObject(int depth) {
        Value object;
        object.type = Value::Type::Object;
        take(); // '{'
        skipWhitespace();
        if (peek() == '}') {
            take();
            return object;
        }
        while (true) {
            skipWhitespace();
            if (peek() != '"') {
                fail("expected a member name");
            }
            std::string key = parseString();
            for (const auto& member : object.members) {
                if (member.first == key) {
                    fail("duplicate key '" + key + "'");
                }
            }
            skipWhitespace();
            if (peek() != ':') {
                fail("expected ':' after the member name");
            }
            take();
            std::optional<Value> value = parseValue(depth + 1);
            if (!value) {
                return std::nullopt;
            }
            object.members.emplace_back(std::move(key), std::move(*value));
            skipWhitespace();
            if (peek() == ',') {
                take();
                continue;
            }
            if (peek() == '}') {
                take();
                return object;
            }
            fail("expected ',' or '}'");
        }
    }

    std::optional<Value> parseArray(int depth) {
        Value array;
        array.type = Value::Type::Array;
        take(); // '['
        skipWhitespace();
        if (peek() == ']') {
            take();
            return array;
        }
        while (true) {
            std::optional<Value> value = parseValue(depth + 1);
            if (!value) {
                return std::nullopt;
            }
            array.items.push_back(std::move(*value));
            skipWhitespace();
            if (peek() == ',') {
                take();
                continue;
            }
            if (peek() == ']') {
                take();
                return array;
            }
            fail("expected ',' or ']'");
        }
    }

    static void appendUtf8(std::string& out, std::uint32_t code) {
        if (code <= 0x7F) {
            out.push_back(static_cast<char>(code));
        } else if (code <= 0x7FF) {
            out.push_back(static_cast<char>(0xC0 | (code >> 6)));
            out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
        } else if (code <= 0xFFFF) {
            out.push_back(static_cast<char>(0xE0 | (code >> 12)));
            out.push_back(static_cast<char>(0x80 | ((code >> 6) & 0x3F)));
            out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
        } else {
            out.push_back(static_cast<char>(0xF0 | (code >> 18)));
            out.push_back(static_cast<char>(0x80 | ((code >> 12) & 0x3F)));
            out.push_back(static_cast<char>(0x80 | ((code >> 6) & 0x3F)));
            out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
        }
    }

    std::uint32_t parseHex4() {
        std::uint32_t value = 0;
        for (int i = 0; i < 4; ++i) {
            if (atEnd()) {
                fail("truncated \\u escape");
            }
            const char c = take();
            value <<= 4;
            if (c >= '0' && c <= '9') {
                value |= static_cast<std::uint32_t>(c - '0');
            } else if (c >= 'a' && c <= 'f') {
                value |= static_cast<std::uint32_t>(c - 'a' + 10);
            } else if (c >= 'A' && c <= 'F') {
                value |= static_cast<std::uint32_t>(c - 'A' + 10);
            } else {
                fail("invalid hex digit in \\u escape");
            }
        }
        return value;
    }

    std::string parseString() {
        take(); // opening quote
        std::string out;
        while (true) {
            if (atEnd()) {
                fail("unterminated string");
            }
            const char c = take();
            if (c == '"') {
                return out;
            }
            if (static_cast<unsigned char>(c) < 0x20) {
                fail("control character in string");
            }
            if (c != '\\') {
                out.push_back(c);
                continue;
            }
            if (atEnd()) {
                fail("unterminated escape");
            }
            switch (take()) {
            case '"':
                out.push_back('"');
                break;
            case '\\':
                out.push_back('\\');
                break;
            case '/':
                out.push_back('/');
                break;
            case 'b':
                out.push_back('\b');
                break;
            case 'f':
                out.push_back('\f');
                break;
            case 'n':
                out.push_back('\n');
                break;
            case 'r':
                out.push_back('\r');
                break;
            case 't':
                out.push_back('\t');
                break;
            case 'u': {
                const std::uint32_t first = parseHex4();
                std::uint32_t code = first;
                if (first >= 0xD800 && first <= 0xDBFF && text_.substr(pos_, 2) == "\\u") {
                    take();
                    take();
                    const std::uint32_t second = parseHex4();
                    if (second < 0xDC00 || second > 0xDFFF) {
                        fail("invalid low surrogate");
                    }
                    code = 0x10000 + ((first - 0xD800) << 10) + (second - 0xDC00);
                }
                appendUtf8(out, code);
                break;
            }
            default:
                fail("invalid escape");
            }
        }
    }

    std::optional<Value> parseNumber() {
        const std::size_t start = pos_;
        if (peek() == '-') {
            take();
        }
        if (atEnd() || peek() < '0' || peek() > '9') {
            fail("expected a value");
        }
        // JSON forbids leading zeros ("01" is not a number).
        if (peek() == '0') {
            take();
            if (!atEnd() && peek() >= '0' && peek() <= '9') {
                fail("numbers must not have leading zeros");
            }
        } else {
            while (!atEnd() && peek() >= '0' && peek() <= '9') {
                take();
            }
        }
        if (!atEnd() && peek() == '.') {
            take();
            if (atEnd() || peek() < '0' || peek() > '9') {
                fail("expected a digit after '.'");
            }
            while (!atEnd() && peek() >= '0' && peek() <= '9') {
                take();
            }
        }
        if (!atEnd() && (peek() == 'e' || peek() == 'E')) {
            take();
            if (!atEnd() && (peek() == '+' || peek() == '-')) {
                take();
            }
            if (atEnd() || peek() < '0' || peek() > '9') {
                fail("expected a digit in the exponent");
            }
            while (!atEnd() && peek() >= '0' && peek() <= '9') {
                take();
            }
        }
        const std::string_view literal = text_.substr(start, pos_ - start);
        Value value;
        value.type = Value::Type::Number;
        const auto result = std::from_chars(literal.data(), literal.data() + literal.size(), value.number);
        if (result.ec != std::errc{} || result.ptr != literal.data() + literal.size() || !std::isfinite(value.number)) {
            fail("number out of range");
        }
        return value;
    }

    std::string_view text_;
    std::string& error_;
    std::size_t pos_ = 0;
    int line_ = 1;
    int column_ = 1;
};

} // namespace

const Value* Value::find(std::string_view key) const {
    for (const auto& member : members) {
        if (member.first == key) {
            return &member.second;
        }
    }
    return nullptr;
}

const Value* Value::at(std::size_t index) const {
    return index < items.size() ? &items[index] : nullptr;
}

std::optional<Value> parse(std::string_view text, std::string& error) {
    Parser parser(text, error);
    try {
        return parser.run();
    } catch (const Parser::Failure&) {
        return std::nullopt;
    }
}

} // namespace sim::app::json
