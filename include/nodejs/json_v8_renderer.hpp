#ifndef OSRM_BINDINGS_NODE_JSON_V8_RENDERER_HPP
#define OSRM_BINDINGS_NODE_JSON_V8_RENDERER_HPP

#include "osrm/json_container.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <napi.h>
#include <uv.h>
#pragma GCC diagnostic pop

#include <functional>

namespace node_osrm
{

struct V8Renderer
{
    explicit V8Renderer(Napi::Value &_out) : out(_out) {}

    void operator()(const osrm::json::String &string) const
    {
        out = Napi::New(env, std::cref(string.value));
    }

    void operator()(const osrm::json::Number &number) const { out = Napi::New(env, number.value); }

    void operator()(const osrm::json::Object &object) const
    {
        Napi::Object obj = Napi::Object::New(env);
        for (const auto &keyValue : object.values)
        {
            Napi::Value child;
            mapbox::util::apply_visitor(V8Renderer(child), keyValue.second);
            obj.Set(Napi::New(env, keyValue.first), child);
        }
        out = obj;
    }

    void operator()(const osrm::json::Array &array) const
    {
        Napi::Array a = Napi::Array::New(env, array.values.size());
        for (auto i = 0u; i < array.values.size(); ++i)
        {
            Napi::Value child;
            mapbox::util::apply_visitor(V8Renderer(child), array.values[i]);
            a.Set(i, child);
        }
        out = a;
    }

    void operator()(const osrm::json::True &) const { out = Napi::New(env, true); }

    void operator()(const osrm::json::False &) const { out = Napi::New(env, false); }

    void operator()(const osrm::json::Null &) const { out = env.Null(); }

  private:
    Napi::Value &out;
};

inline void renderToV8(Napi::Value &out, const osrm::json::Object &object)
{
    osrm::json::Value value = object;
    mapbox::util::apply_visitor(V8Renderer(out), value);
}
}

#endif // JSON_V8_RENDERER_HPP
