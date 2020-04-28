#ifndef OSRM_BINDINGS_NODE_HPP
#define OSRM_BINDINGS_NODE_HPP

#include "osrm/osrm_fwd.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <napi.h>
#include <uv.h>
#pragma GCC diagnostic pop

#include <memory>

namespace node_osrm
{

struct Engine final : public Napi::ObjectWrap
{
    using Base = Napi::ObjectWrap;

    static Napi::Object Init(Napi::Env env, Napi::Object exports);

    static Napi::Value New(const Napi::CallbackInfo& info);

    static Napi::Value route(const Napi::CallbackInfo& info);
    static Napi::Value nearest(const Napi::CallbackInfo& info);
    static Napi::Value table(const Napi::CallbackInfo& info);
    static Napi::Value tile(const Napi::CallbackInfo& info);
    static Napi::Value match(const Napi::CallbackInfo& info);
    static Napi::Value trip(const Napi::CallbackInfo& info);

    Engine(osrm::EngineConfig &config);

    // Thread-safe singleton accessor
    static Napi::FunctionReference &constructor();

    // Ref-counted OSRM alive even after shutdown until last callback is done
    std::shared_ptr<osrm::OSRM> this_;
};

} // ns node_osrm

NODE_API_MODULE(osrm, node_osrm::Engine::Init)

#endif
