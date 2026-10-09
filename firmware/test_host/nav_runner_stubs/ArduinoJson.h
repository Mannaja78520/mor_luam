#pragma once
#include "Arduino.h"
#include <map>
#include <memory>
#include <vector>
#include <type_traits>
struct JsonNode {
    std::map<std::string, JsonNode> children;
    std::vector<JsonNode> items;
    std::string text;
    double number = 0;
    bool null = true;
};
class JsonObject;
class JsonProxy {
public:
    explicit JsonProxy(JsonNode* node) : node_(node) {}
    template<class T, std::enable_if_t<std::is_arithmetic_v<T>, int> = 0>
    JsonProxy& operator=(T value) { node_->number = static_cast<double>(value); node_->null = false; return *this; }
    JsonProxy& operator=(const char* value) { node_->text = value; node_->null = false; return *this; }
    JsonProxy& operator=(const String& value) { return *this = value.c_str(); }
    JsonProxy& operator=(std::nullptr_t) { node_->null = true; return *this; }
    template<class T> T to() { node_->null = false; return T(node_); }   // like ArduinoJson: now an object/array
private:
    JsonNode* node_;
};
class JsonObject {
public:
    explicit JsonObject(JsonNode* node) : node_(node) {}
    JsonProxy operator[](const char* key) { return JsonProxy(&node_->children[key]); }
private:
    JsonNode* node_;
};
class JsonArray {
public:
    explicit JsonArray(JsonNode* node) : node_(node) {}
    template<class T> T add() { node_->items.emplace_back(); return T(&node_->items.back()); }
private:
    JsonNode* node_;
};
