#ifndef OSRM_BINDINGS_NODE_SUPPORT_HPP
#define OSRM_BINDINGS_NODE_SUPPORT_HPP

#include "nodejs/json_v8_renderer.hpp"
#include "util/json_renderer.hpp"

#include "osrm/approach.hpp"
#include "osrm/bearing.hpp"
#include "osrm/coordinate.hpp"
#include "osrm/engine_config.hpp"
#include "osrm/json_container.hpp"
#include "osrm/match_parameters.hpp"
#include "osrm/nearest_parameters.hpp"
#include "osrm/osrm.hpp"
#include "osrm/route_parameters.hpp"
#include "osrm/status.hpp"
#include "osrm/storage_config.hpp"
#include "osrm/table_parameters.hpp"
#include "osrm/tile_parameters.hpp"
#include "osrm/trip_parameters.hpp"

#include <boost/assert.hpp>
#include <boost/optional.hpp>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include <exception>
#include <memory>
#include <utility>

namespace node_osrm
{

using engine_config_ptr = std::unique_ptr<osrm::EngineConfig>;
using route_parameters_ptr = std::unique_ptr<osrm::RouteParameters>;
using trip_parameters_ptr = std::unique_ptr<osrm::TripParameters>;
using tile_parameters_ptr = std::unique_ptr<osrm::TileParameters>;
using match_parameters_ptr = std::unique_ptr<osrm::MatchParameters>;
using nearest_parameters_ptr = std::unique_ptr<osrm::NearestParameters>;
using table_parameters_ptr = std::unique_ptr<osrm::TableParameters>;

struct PluginParameters
{
    bool renderJSONToBuffer = false;
};

using ObjectOrString = typename mapbox::util::variant<osrm::json::Object, std::string>;

template <typename ResultT> inline Napi::Value render(const ResultT &result);

template <> Napi::Value inline render(const std::string &result)
{
    return Napi::Buffer::Copy(env, result.data(), result.size());
}

template <> Napi::Value inline render(const ObjectOrString &result)
{
    if (result.is<osrm::json::Object>())
    {
        // Convert osrm::json object tree into matching v8 object tree
        Napi::Value value;
        renderToV8(value, result.get<osrm::json::Object>());
        return value;
    }
    else
    {
        // Return the string object as a node Buffer
        return Napi::Buffer::Copy(env, result.get<std::string>().data(), result.get<std::string>().size())
            ;
    }
}

inline void ParseResult(const osrm::Status &result_status, osrm::json::Object &result)
{
    const auto code_iter = result.values.find("code");
    const auto end_iter = result.values.end();

    BOOST_ASSERT(code_iter != end_iter);

    if (result_status == osrm::Status::Error)
    {
        throw std::logic_error(code_iter->second.get<osrm::json::String>().value.c_str());
    }

    result.values.erase(code_iter);
    const auto message_iter = result.values.find("message");
    if (message_iter != end_iter)
    {
        result.values.erase(message_iter);
    }
}

inline void ParseResult(const osrm::Status & /*result_status*/, const std::string & /*unused*/) {}

inline engine_config_ptr argumentsToEngineConfig(const Napi::CallbackInfo&args)
{
    Napi::HandleScope scope(env);
    auto engine_config = std::make_unique<osrm::EngineConfig>();

    if (args.Length() == 0)
    {
        return engine_config;
    }
    else if (args.Length() > 1)
    {
        Napi::Error::New(env, "Only accepts one parameter").ThrowAsJavaScriptException();

        return engine_config_ptr();
    }

    BOOST_ASSERT(args.Length() == 1);

    if (args[0].IsString())
    {
        engine_config->storage_config = osrm::StorageConfig(
            *v8::String::Utf8Value(args[0].To<Napi::String>()));
        engine_config->use_shared_memory = false;
        return engine_config;
    }
    else if (!args[0].IsObject())
    {
        Napi::Error::New(env, "Parameter must be a path or options object").ThrowAsJavaScriptException();

        return engine_config_ptr();
    }

    BOOST_ASSERT(args[0].IsObject());
    auto params = args[0].To<Napi::Object>();

    auto path = params->Get(Napi::String::New(env, "path"));
    if (path.IsEmpty())
        return engine_config_ptr();

    auto memory_file = params->Get(Napi::String::New(env, "memory_file"));
    if (memory_file.IsEmpty())
        return engine_config_ptr();

    auto shared_memory = params->Get(Napi::String::New(env, "shared_memory"));
    if (shared_memory.IsEmpty())
        return engine_config_ptr();

    auto mmap_memory = params->Get(Napi::String::New(env, "mmap_memory"));
    if (mmap_memory.IsEmpty())
        return engine_config_ptr();

    if (!memory_file->IsUndefined())
    {
        if (path->IsUndefined())
        {
            Napi::Error::New(env, "memory_file option requires a path to a file.").ThrowAsJavaScriptException();

            return engine_config_ptr();
        }

        engine_config->memory_file =
            *v8::String::Utf8Value(memory_file.To<Napi::String>());
    }

    auto dataset_name = params->Get(Napi::String::New(env, "dataset_name"));
    if (dataset_name.IsEmpty())
        return engine_config_ptr();
    if (!dataset_name->IsUndefined())
    {
        if (dataset_name.IsString())
        {
            engine_config->dataset_name =
                *v8::String::Utf8Value(dataset_name.To<Napi::String>());
        }
        else
        {
            Napi::Error::New(env, "dataset_name needs to be a string").ThrowAsJavaScriptException();

            return engine_config_ptr();
        }
    }

    if (!path->IsUndefined())
    {
        engine_config->storage_config =
            osrm::StorageConfig(*v8::String::Utf8Value(path.To<Napi::String>()));

        engine_config->use_shared_memory = false;
    }
    if (!shared_memory->IsUndefined())
    {
        if (shared_memory->IsBoolean())
        {
            engine_config->use_shared_memory = shared_memory.As<Napi::Boolean>().Value();
        }
        else
        {
            Napi::Error::New(env, "Shared_memory option must be a boolean").ThrowAsJavaScriptException();

            return engine_config_ptr();
        }
    }
    if (!mmap_memory->IsUndefined())
    {
        if (mmap_memory->IsBoolean())
        {
            engine_config->use_mmap = mmap_memory.As<Napi::Boolean>().Value();
        }
        else
        {
            Napi::Error::New(env, "mmap_memory option must be a boolean").ThrowAsJavaScriptException();

            return engine_config_ptr();
        }
    }

    if (path->IsUndefined() && !engine_config->use_shared_memory)
    {
        Napi::ThrowError("Shared_memory must be enabled if no path is "
                        "specified");
        return engine_config_ptr();
    }

    auto algorithm = params->Get(Napi::String::New(env, "algorithm"));
    if (algorithm.IsEmpty())
        return engine_config_ptr();

    if (algorithm.IsString())
    {
        auto algorithm_str = algorithm.To<Napi::String>();
        if (*v8::String::Utf8Value(algorithm_str) == std::string("CH"))
        {
            engine_config->algorithm = osrm::EngineConfig::Algorithm::CH;
        }
        else if (*v8::String::Utf8Value(algorithm_str) == std::string("CoreCH"))
        {
            engine_config->algorithm = osrm::EngineConfig::Algorithm::CH;
        }
        else if (*v8::String::Utf8Value(algorithm_str) == std::string("MLD"))
        {
            engine_config->algorithm = osrm::EngineConfig::Algorithm::MLD;
        }
        else
        {
            Napi::Error::New(env, "algorithm option must be one of 'CH', 'CoreCH', or 'MLD'.").ThrowAsJavaScriptException();

            return engine_config_ptr();
        }
    }
    else if (!algorithm->IsUndefined())
    {
        Napi::Error::New(env, "algorithm option must be a string and one of 'CH', 'CoreCH', or 'MLD'.").ThrowAsJavaScriptException();

        return engine_config_ptr();
    }

    // Set EngineConfig system-wide limits on construction, if requested

    auto max_locations_trip = params->Get(Napi::String::New(env, "max_locations_trip"));
    auto max_locations_viaroute = params->Get(Napi::String::New(env, "max_locations_viaroute"));
    auto max_locations_distance_table =
        params->Get(Napi::String::New(env, "max_locations_distance_table"));
    auto max_locations_map_matching =
        params->Get(Napi::String::New(env, "max_locations_map_matching"));
    auto max_results_nearest = params->Get(Napi::String::New(env, "max_results_nearest"));
    auto max_alternatives = params->Get(Napi::String::New(env, "max_alternatives"));
    auto max_radius_map_matching =
        params->Get(Napi::String::New(env, "max_radius_map_matching"));

    if (!max_locations_trip->IsUndefined() && !max_locations_trip.IsNumber())
    {
        Napi::Error::New(env, "max_locations_trip must be an integral number").ThrowAsJavaScriptException();

        return engine_config_ptr();
    }
    if (!max_locations_viaroute->IsUndefined() && !max_locations_viaroute.IsNumber())
    {
        Napi::Error::New(env, "max_locations_viaroute must be an integral number").ThrowAsJavaScriptException();

        return engine_config_ptr();
    }
    if (!max_locations_distance_table->IsUndefined() && !max_locations_distance_table.IsNumber())
    {
        Napi::Error::New(env, "max_locations_distance_table must be an integral number").ThrowAsJavaScriptException();

        return engine_config_ptr();
    }
    if (!max_locations_map_matching->IsUndefined() && !max_locations_map_matching.IsNumber())
    {
        Napi::Error::New(env, "max_locations_map_matching must be an integral number").ThrowAsJavaScriptException();

        return engine_config_ptr();
    }
    if (!max_results_nearest->IsUndefined() && !max_results_nearest.IsNumber())
    {
        Napi::Error::New(env, "max_results_nearest must be an integral number").ThrowAsJavaScriptException();

        return engine_config_ptr();
    }
    if (!max_alternatives->IsUndefined() && !max_alternatives.IsNumber())
    {
        Napi::Error::New(env, "max_alternatives must be an integral number").ThrowAsJavaScriptException();

        return engine_config_ptr();
    }

    if (max_locations_trip.IsNumber())
        engine_config->max_locations_trip = static_cast<int>(max_locations_trip.As<Napi::Number>().DoubleValue());
    if (max_locations_viaroute.IsNumber())
        engine_config->max_locations_viaroute =
            static_cast<int>(max_locations_viaroute.As<Napi::Number>().DoubleValue());
    if (max_locations_distance_table.IsNumber())
        engine_config->max_locations_distance_table =
            static_cast<int>(max_locations_distance_table.As<Napi::Number>().DoubleValue());
    if (max_locations_map_matching.IsNumber())
        engine_config->max_locations_map_matching =
            static_cast<int>(max_locations_map_matching.As<Napi::Number>().DoubleValue());
    if (max_results_nearest.IsNumber())
        engine_config->max_results_nearest = static_cast<int>(max_results_nearest.As<Napi::Number>().DoubleValue());
    if (max_alternatives.IsNumber())
        engine_config->max_alternatives = static_cast<int>(max_alternatives.As<Napi::Number>().DoubleValue());
    if (max_radius_map_matching.IsNumber())
        engine_config->max_radius_map_matching =
            static_cast<double>(max_radius_map_matching.As<Napi::Number>().DoubleValue());

    return engine_config;
}

inline boost::optional<std::vector<osrm::Coordinate>>
parseCoordinateArray(const Napi::Array &coordinates_array)
{
    Napi::HandleScope scope(env);
    boost::optional<std::vector<osrm::Coordinate>> resulting_coordinates;
    std::vector<osrm::Coordinate> temp_coordinates;

    for (uint32_t i = 0; i < coordinates_array->Length(); ++i)
    {
        Napi::Value coordinate = coordinates_array->Get(i);
        if (coordinate.IsEmpty())
            return resulting_coordinates;

        if (!coordinate->IsArray())
        {
            Napi::Error::New(env, "Coordinates must be an array of (lon/lat) pairs").ThrowAsJavaScriptException();

            return resulting_coordinates;
        }

        Napi::Array coordinate_pair = coordinate.As<Napi::Array>();
        if (coordinate_pair->Length() != 2)
        {
            Napi::Error::New(env, "Coordinates must be an array of (lon/lat) pairs").ThrowAsJavaScriptException();

            return resulting_coordinates;
        }

        if (!coordinate_pair->Get(0).IsNumber() || !coordinate_pair->Get(1).IsNumber())
        {
            Napi::Error::New(env, "Each member of a coordinate pair must be a number").ThrowAsJavaScriptException();

            return resulting_coordinates;
        }

        double lon = coordinate_pair->Get(0).As<Napi::Number>().DoubleValue();
        double lat = coordinate_pair->Get(1).As<Napi::Number>().DoubleValue();

        if (std::isnan(lon) || std::isnan(lat) || std::isinf(lon) || std::isinf(lat))
        {
            Napi::Error::New(env, "Lng/Lat coordinates must be valid numbers").ThrowAsJavaScriptException();

            return resulting_coordinates;
        }

        if (lon > 180 || lon < -180 || lat > 90 || lat < -90)
        {
            Napi::ThrowError("Lng/Lat coordinates must be within world bounds "
                            "(-180 < lng < 180, -90 < lat < 90)");
            return resulting_coordinates;
        }

        temp_coordinates.emplace_back(osrm::util::FloatLongitude{std::move(lon)},
                                      osrm::util::FloatLatitude{std::move(lat)});
    }

    resulting_coordinates = boost::make_optional(std::move(temp_coordinates));
    return resulting_coordinates;
}

// Parses all the non-service specific parameters
template <typename ParamType>
inline bool argumentsToParameter(const Napi::CallbackInfo&args,
                                 ParamType &params,
                                 bool requires_multiple_coordinates)
{
    Napi::HandleScope scope(env);

    if (args.Length() < 2)
    {
        Napi::TypeError::New(env, "Two arguments required").ThrowAsJavaScriptException();

        return false;
    }

    if (!args[0].IsObject())
    {
        Napi::TypeError::New(env, "First arg must be an object").ThrowAsJavaScriptException();

        return false;
    }

    Napi::Object obj = args[0].To<Napi::Object>();

    Napi::Value coordinates = obj->Get(Napi::String::New(env, "coordinates"));
    if (coordinates.IsEmpty())
        return false;

    if (coordinates->IsUndefined())
    {
        Napi::Error::New(env, "Must provide a coordinates property").ThrowAsJavaScriptException();

        return false;
    }
    else if (coordinates->IsArray())
    {
        auto coordinates_array = coordinates.As<Napi::Array>();
        if (coordinates_array->Length() < 2 && requires_multiple_coordinates)
        {
            Napi::Error::New(env, "At least two coordinates must be provided").ThrowAsJavaScriptException();

            return false;
        }
        else if (!requires_multiple_coordinates && coordinates_array->Length() != 1)
        {
            Napi::Error::New(env, "Exactly one coordinate pair must be provided").ThrowAsJavaScriptException();

            return false;
        }
        auto maybe_coordinates = parseCoordinateArray(coordinates_array);
        if (maybe_coordinates)
        {
            std::copy(maybe_coordinates->begin(),
                      maybe_coordinates->end(),
                      std::back_inserter(params->coordinates));
        }
        else
        {
            return false;
        }
    }
    else if (!coordinates->IsUndefined())
    {
        BOOST_ASSERT(!coordinates->IsArray());
        Napi::Error::New(env, "Coordinates must be an array of (lon/lat) pairs").ThrowAsJavaScriptException();

        return false;
    }

    if (obj->Has(Napi::String::New(env, "approaches")))
    {
        Napi::Value approaches = obj->Get(Napi::String::New(env, "approaches"));
        if (approaches.IsEmpty())
            return false;

        if (!approaches->IsArray())
        {
            Napi::Error::New(env, "Approaches must be an arrays of strings").ThrowAsJavaScriptException();

            return false;
        }

        auto approaches_array = approaches.As<Napi::Array>();

        if (approaches_array->Length() != params->coordinates.size())
        {
            Napi::Error::New(env, "Approaches array must have the same length as coordinates array").ThrowAsJavaScriptException();

            return false;
        }

        for (uint32_t i = 0; i < approaches_array->Length(); ++i)
        {
            Napi::Value approach_raw = approaches_array->Get(i);
            if (approach_raw.IsEmpty())
                return false;

            if (approach_raw->IsNull())
            {
                params->approaches.emplace_back();
            }
            else if (approach_raw.IsString())
            {
                const std::string approach_utf8str = approach_raw.As<Napi::String>();
                std::string approach_str{*approach_utf8str,
                                         *approach_utf8str + approach_utf8str.Length()};
                if (approach_str == "curb")
                {
                    params->approaches.push_back(osrm::Approach::CURB);
                }
                else if (approach_str == "unrestricted")
                {
                    params->approaches.push_back(osrm::Approach::UNRESTRICTED);
                }
                else
                {
                    Napi::Error::New(env, "'approaches' param must be one of [curb, unrestricted]").ThrowAsJavaScriptException();

                    return false;
                }
            }
            else
            {
                Napi::Error::New(env, "Approach must be a string: [curb, unrestricted] or null").ThrowAsJavaScriptException();

                return false;
            }
        }
    }

    if (obj->Has(Napi::String::New(env, "bearings")))
    {
        Napi::Value bearings = obj->Get(Napi::String::New(env, "bearings"));
        if (bearings.IsEmpty())
            return false;

        if (!bearings->IsArray())
        {
            Napi::Error::New(env, "Bearings must be an array of arrays of numbers").ThrowAsJavaScriptException();

            return false;
        }

        auto bearings_array = bearings.As<Napi::Array>();

        if (bearings_array->Length() != params->coordinates.size())
        {
            Napi::Error::New(env, "Bearings array must have the same length as coordinates array").ThrowAsJavaScriptException();

            return false;
        }

        for (uint32_t i = 0; i < bearings_array->Length(); ++i)
        {
            Napi::Value bearing_raw = bearings_array->Get(i);
            if (bearing_raw.IsEmpty())
                return false;

            if (bearing_raw->IsNull())
            {
                params->bearings.emplace_back();
            }
            else if (bearing_raw->IsArray())
            {
                auto bearing_pair = bearing_raw.As<Napi::Array>();
                if (bearing_pair->Length() == 2)
                {
                    if (!bearing_pair->Get(0).IsNumber() || !bearing_pair->Get(1).IsNumber())
                    {
                        Napi::Error::New(env, "Bearing values need to be numbers in range 0..360").ThrowAsJavaScriptException();

                        return false;
                    }

                    const auto bearing = static_cast<short>(bearing_pair->Get(0).As<Napi::Number>().DoubleValue());
                    const auto range = static_cast<short>(bearing_pair->Get(1).As<Napi::Number>().DoubleValue());

                    if (bearing < 0 || bearing > 360 || range < 0 || range > 180)
                    {
                        Napi::Error::New(env, "Bearing values need to be in range 0..360, 0..180").ThrowAsJavaScriptException();

                        return false;
                    }

                    params->bearings.push_back(osrm::Bearing{bearing, range});
                }
                else
                {
                    Napi::Error::New(env, "Bearing must be an array of [bearing, range] or null").ThrowAsJavaScriptException();

                    return false;
                }
            }
            else
            {
                Napi::Error::New(env, "Bearing must be an array of [bearing, range] or null").ThrowAsJavaScriptException();

                return false;
            }
        }
    }

    if (obj->Has(Napi::String::New(env, "hints")))
    {
        Napi::Value hints = obj->Get(Napi::String::New(env, "hints"));
        if (hints.IsEmpty())
            return false;

        if (!hints->IsArray())
        {
            Napi::Error::New(env, "Hints must be an array of strings/null").ThrowAsJavaScriptException();

            return false;
        }

        Napi::Array hints_array = hints.As<Napi::Array>();

        if (hints_array->Length() != params->coordinates.size())
        {
            Napi::Error::New(env, "Hints array must have the same length as coordinates array").ThrowAsJavaScriptException();

            return false;
        }

        for (uint32_t i = 0; i < hints_array->Length(); ++i)
        {
            Napi::Value hint = hints_array->Get(i);
            if (hint.IsEmpty())
                return false;

            if (hint.IsString())
            {
                if (hint->ToString()->Length() == 0)
                {
                    Napi::Error::New(env, "Hint cannot be an empty string").ThrowAsJavaScriptException();

                    return false;
                }

                params->hints.push_back(
                    osrm::engine::Hint::FromBase64(*v8::String::Utf8Value(hint)));
            }
            else if (hint->IsNull())
            {
                params->hints.emplace_back();
            }
            else
            {
                Napi::Error::New(env, "Hint must be null or string").ThrowAsJavaScriptException();

                return false;
            }
        }
    }

    if (obj->Has(Napi::String::New(env, "radiuses")))
    {
        Napi::Value radiuses = obj->Get(Napi::String::New(env, "radiuses"));
        if (radiuses.IsEmpty())
            return false;

        if (!radiuses->IsArray())
        {
            Napi::Error::New(env, "Radiuses must be an array of non-negative doubles or null").ThrowAsJavaScriptException();

            return false;
        }

        Napi::Array radiuses_array = radiuses.As<Napi::Array>();

        if (radiuses_array->Length() != params->coordinates.size())
        {
            Napi::Error::New(env, "Radiuses array must have the same length as coordinates array").ThrowAsJavaScriptException();

            return false;
        }

        for (uint32_t i = 0; i < radiuses_array->Length(); ++i)
        {
            Napi::Value radius = radiuses_array->Get(i);
            if (radius.IsEmpty())
                return false;

            if (radius->IsNull())
            {
                params->radiuses.emplace_back();
            }
            else if (radius.IsNumber() && radius.As<Napi::Number>().DoubleValue() >= 0)
            {
                params->radiuses.push_back(static_cast<double>(radius.As<Napi::Number>().DoubleValue()));
            }
            else
            {
                Napi::Error::New(env, "Radius must be non-negative double or null").ThrowAsJavaScriptException();

                return false;
            }
        }
    }

    if (obj->Has(Napi::String::New(env, "generate_hints")))
    {
        Napi::Value generate_hints = obj->Get(Napi::String::New(env, "generate_hints"));
        if (generate_hints.IsEmpty())
            return false;

        if (!generate_hints->IsBoolean())
        {
            Napi::Error::New(env, "generate_hints must be of type Boolean").ThrowAsJavaScriptException();

            return false;
        }

        params->generate_hints = generate_hints.As<Napi::Boolean>().Value();
    }

    if (obj->Has(Napi::String::New(env, "exclude")))
    {
        Napi::Value exclude = obj->Get(Napi::String::New(env, "exclude"));
        if (exclude.IsEmpty())
            return false;

        if (!exclude->IsArray())
        {
            Napi::Error::New(env, "Exclude must be an array of strings or empty").ThrowAsJavaScriptException();

            return false;
        }

        Napi::Array exclude_array = exclude.As<Napi::Array>();

        for (uint32_t i = 0; i < exclude_array->Length(); ++i)
        {
            Napi::Value class_name = exclude_array->Get(i);
            if (class_name.IsEmpty())
                return false;

            if (class_name.IsString())
            {
                std::string class_name_str = *v8::String::Utf8Value(class_name);
                params->exclude.emplace_back(class_name_str);
            }
            else
            {
                Napi::Error::New(env, "Exclude must be an array of strings or empty").ThrowAsJavaScriptException();

                return false;
            }
        }
    }

    return true;
}

template <typename ParamType>
inline bool parseCommonParameters(const Napi::Object &obj, ParamType &params)
{
    if (obj->Has(Napi::String::New(env, "steps")))
    {
        auto steps = obj->Get(Napi::String::New(env, "steps"));
        if (steps.IsEmpty())
            return false;

        if (steps->IsBoolean())
        {
            params->steps = steps.As<Napi::Boolean>().Value();
        }
        else
        {
            Napi::Error::New(env, "'steps' param must be a boolean").ThrowAsJavaScriptException();

            return false;
        }
    }

    if (obj->Has(Napi::String::New(env, "annotations")))
    {
        auto annotations = obj->Get(Napi::String::New(env, "annotations"));
        if (annotations.IsEmpty())
            return false;

        if (annotations->IsBoolean())
        {
            params->annotations = annotations.As<Napi::Boolean>().Value();
        }
        else if (annotations->IsArray())
        {
            Napi::Array annotations_array = annotations.As<Napi::Array>();
            for (std::size_t i = 0; i < annotations_array->Length(); i++)
            {
                const std::string annotations_utf8str = annotations_array->Get(i.As<Napi::String>());
                std::string annotations_str{*annotations_utf8str,
                                            *annotations_utf8str + annotations_utf8str.Length()};

                if (annotations_str == "duration")
                {
                    params->annotations_type =
                        params->annotations_type | osrm::RouteParameters::AnnotationsType::Duration;
                }
                else if (annotations_str == "nodes")
                {
                    params->annotations_type =
                        params->annotations_type | osrm::RouteParameters::AnnotationsType::Nodes;
                }
                else if (annotations_str == "distance")
                {
                    params->annotations_type =
                        params->annotations_type | osrm::RouteParameters::AnnotationsType::Distance;
                }
                else if (annotations_str == "weight")
                {
                    params->annotations_type =
                        params->annotations_type | osrm::RouteParameters::AnnotationsType::Weight;
                }
                else if (annotations_str == "datasources")
                {
                    params->annotations_type = params->annotations_type |
                                               osrm::RouteParameters::AnnotationsType::Datasources;
                }
                else if (annotations_str == "speed")
                {
                    params->annotations_type =
                        params->annotations_type | osrm::RouteParameters::AnnotationsType::Speed;
                }
                else
                {
                    Napi::Error::New(env, "this 'annotations' param is not supported").ThrowAsJavaScriptException();

                    return false;
                }
            }
        }
        else
        {
            Napi::Error::New(env, "this 'annotations' param is not supported").ThrowAsJavaScriptException();

            return false;
        }
    }

    if (obj->Has(Napi::String::New(env, "geometries")))
    {
        Napi::Value geometries = obj->Get(Napi::String::New(env, "geometries"));
        if (geometries.IsEmpty())
            return false;

        if (!geometries.IsString())
        {
            Napi::Error::New(env, "Geometries must be a string: [polyline, polyline6, geojson]").ThrowAsJavaScriptException();

            return false;
        }
        const std::string geometries_utf8str = geometries.As<Napi::String>();
        std::string geometries_str{*geometries_utf8str,
                                   *geometries_utf8str + geometries_utf8str.Length()};

        if (geometries_str == "polyline")
        {
            params->geometries = osrm::RouteParameters::GeometriesType::Polyline;
        }
        else if (geometries_str == "polyline6")
        {
            params->geometries = osrm::RouteParameters::GeometriesType::Polyline6;
        }
        else if (geometries_str == "geojson")
        {
            params->geometries = osrm::RouteParameters::GeometriesType::GeoJSON;
        }
        else
        {
            Napi::Error::New(env, "'geometries' param must be one of [polyline, polyline6, geojson]").ThrowAsJavaScriptException();

            return false;
        }
    }

    if (obj->Has(Napi::String::New(env, "overview")))
    {
        Napi::Value overview = obj->Get(Napi::String::New(env, "overview"));
        if (overview.IsEmpty())
            return false;

        if (!overview.IsString())
        {
            Napi::Error::New(env, "Overview must be a string: [simplified, full, false]").ThrowAsJavaScriptException();

            return false;
        }

        const std::string overview_utf8str = overview.As<Napi::String>();
        std::string overview_str{*overview_utf8str, *overview_utf8str + overview_utf8str.Length()};

        if (overview_str == "simplified")
        {
            params->overview = osrm::RouteParameters::OverviewType::Simplified;
        }
        else if (overview_str == "full")
        {
            params->overview = osrm::RouteParameters::OverviewType::Full;
        }
        else if (overview_str == "false")
        {
            params->overview = osrm::RouteParameters::OverviewType::False;
        }
        else
        {
            Napi::Error::New(env, "'overview' param must be one of [simplified, full, false]").ThrowAsJavaScriptException();

            return false;
        }
    }

    return true;
}

inline PluginParameters
argumentsToPluginParameters(const Napi::CallbackInfo&args)
{
    if (args.Length() < 3 || !args[1].IsObject())
    {
        return {};
    }
    Napi::Object obj = args[1].To<Napi::Object>();
    if (obj->Has(Napi::String::New(env, "format")))
    {

        Napi::Value format = obj->Get(Napi::String::New(env, "format"));
        if (format.IsEmpty())
        {
            return {};
        }

        if (!format.IsString())
        {
            Napi::Error::New(env, "format must be a string: \"object\" or \"json_buffer\"").ThrowAsJavaScriptException();

            return {};
        }

        const std::string format_utf8str = format.As<Napi::String>();
        std::string format_str{*format_utf8str, *format_utf8str + format_utf8str.Length()};

        if (format_str == "object")
        {
            return {false};
        }
        else if (format_str == "json_buffer")
        {
            return {true};
        }
        else
        {
            Napi::Error::New(env, "format must be a string: \"object\" or \"json_buffer\"").ThrowAsJavaScriptException();

            return {};
        }
    }

    return {};
}

inline route_parameters_ptr
argumentsToRouteParameter(const Napi::CallbackInfo&args,
                          bool requires_multiple_coordinates)
{
    route_parameters_ptr params = std::make_unique<osrm::RouteParameters>();
    bool has_base_params = argumentsToParameter(args, params, requires_multiple_coordinates);
    if (!has_base_params)
        return route_parameters_ptr();

    Napi::Object obj = args[0].To<Napi::Object>();

    if (obj->Has(Napi::String::New(env, "continue_straight")))
    {
        auto value = obj->Get(Napi::String::New(env, "continue_straight"));
        if (value.IsEmpty())
            return route_parameters_ptr();

        if (!value->IsBoolean() && !value->IsNull())
        {
            Napi::Error::New(env, "'continue_straight' param must be boolean or null").ThrowAsJavaScriptException();

            return route_parameters_ptr();
        }
        if (value->IsBoolean())
        {
            params->continue_straight = value.As<Napi::Boolean>().Value();
        }
    }

    if (obj->Has(Napi::String::New(env, "alternatives")))
    {
        auto value = obj->Get(Napi::String::New(env, "alternatives"));
        if (value.IsEmpty())
            return route_parameters_ptr();

        if (value->IsBoolean())
        {
            params->alternatives = value.As<Napi::Boolean>().Value();
            params->number_of_alternatives = value.As<Napi::Boolean>().Value() ? 1u : 0u;
        }
        else if (value.IsNumber())
        {
            params->alternatives = value.As<Napi::Boolean>().Value();
            params->number_of_alternatives = static_cast<unsigned>(value.As<Napi::Number>().DoubleValue());
        }
        else
        {
            Napi::Error::New(env, "'alternatives' param must be boolean or number").ThrowAsJavaScriptException();

            return route_parameters_ptr();
        }
    }

    if (obj->Has(Napi::String::New(env, "waypoints")))
    {
        Napi::Value waypoints = obj->Get(Napi::String::New(env, "waypoints"));
        if (waypoints.IsEmpty())
            return route_parameters_ptr();

        // must be array
        if (!waypoints->IsArray())
        {
            Napi::ThrowError(
                "Waypoints must be an array of integers corresponding to the input coordinates.");
            return route_parameters_ptr();
        }

        auto waypoints_array = waypoints.As<Napi::Array>();
        // must have at least two elements
        if (waypoints_array->Length() < 2)
        {
            Napi::Error::New(env, "At least two waypoints must be provided").ThrowAsJavaScriptException();

            return route_parameters_ptr();
        }
        auto coords_size = params->coordinates.size();
        auto waypoints_array_size = waypoints_array->Length();

        const auto first_index = Napi::To<std::uint32_t>(waypoints_array->Get(0));
        const auto last_index =
            Napi::To<std::uint32_t>(waypoints_array->Get(waypoints_array_size - 1));
        if (first_index != 0 || last_index != coords_size - 1)
        {
            Napi::ThrowError("First and last waypoints values must correspond to first and last "
                            "coordinate indices");
            return route_parameters_ptr();
        }

        for (uint32_t i = 0; i < waypoints_array_size; ++i)
        {
            Napi::Value waypoint_value = waypoints_array->Get(i);
            // all elements must be numbers
            if (!waypoint_value.IsNumber())
            {
                Napi::Error::New(env, "Waypoint values must be an array of integers").ThrowAsJavaScriptException();

                return route_parameters_ptr();
            }
            // check that the waypoint index corresponds with an inpute coordinate
            const auto index = Napi::To<std::uint32_t>(waypoint_value);
            if (index >= coords_size)
            {
                Napi::Error::New(env, "Waypoints must correspond with the index of an input coordinate").ThrowAsJavaScriptException();

                return route_parameters_ptr();
            }
            params->waypoints.emplace_back(static_cast<unsigned>(waypoint_value.As<Napi::Number>().DoubleValue()));
        }

        if (!params->waypoints.empty())
        {
            for (std::size_t i = 0; i < params->waypoints.size() - 1; i++)
            {
                if (params->waypoints[i] >= params->waypoints[i + 1])
                {
                    Napi::Error::New(env, "Waypoints must be supplied in increasing order").ThrowAsJavaScriptException();

                    return route_parameters_ptr();
                }
            }
        }
    }

    if (obj->Has(Napi::String::New(env, "snapping")))
    {
        Napi::Value snapping = obj->Get(Napi::String::New(env, "snapping"));
        if (snapping.IsEmpty())
            return route_parameters_ptr();

        if (!snapping.IsString())
        {
            Napi::Error::New(env, "Snapping must be a string: [default, any]").ThrowAsJavaScriptException();

            return route_parameters_ptr();
        }
        const std::string snapping_utf8str = snapping.As<Napi::String>();
        std::string snapping_str{*snapping_utf8str, *snapping_utf8str + snapping_utf8str.Length()};

        if (snapping_str == "default")
        {
            params->snapping = osrm::RouteParameters::SnappingType::Default;
        }
        else if (snapping_str == "any")
        {
            params->snapping = osrm::RouteParameters::SnappingType::Any;
        }
        else
        {
            Napi::Error::New(env, "'snapping' param must be one of [default, any]").ThrowAsJavaScriptException();

            return route_parameters_ptr();
        }
    }

    bool parsedSuccessfully = parseCommonParameters(obj, params);
    if (!parsedSuccessfully)
    {
        return route_parameters_ptr();
    }

    return params;
}

inline tile_parameters_ptr
argumentsToTileParameters(const Napi::CallbackInfo&args, bool /*unused*/)
{
    tile_parameters_ptr params = std::make_unique<osrm::TileParameters>();

    if (args.Length() < 2)
    {
        Napi::TypeError::New(env, "Coordinate object and callback required").ThrowAsJavaScriptException();

        return tile_parameters_ptr();
    }

    if (!args[0]->IsArray())
    {
        Napi::TypeError::New(env, "Parameter must be an array [x, y, z]").ThrowAsJavaScriptException();

        return tile_parameters_ptr();
    }

    Napi::Array array = args[0].As<Napi::Array>();

    if (array->Length() != 3)
    {
        Napi::TypeError::New(env, "Parameter must be an array [x, y, z]").ThrowAsJavaScriptException();

        return tile_parameters_ptr();
    }

    Napi::Value x = array->Get(0);
    Napi::Value y = array->Get(1);
    Napi::Value z = array->Get(2);
    if (x.IsEmpty() || y.IsEmpty() || z.IsEmpty())
        return tile_parameters_ptr();

    if (!x->IsUint32() && !x->IsUndefined())
    {
        Napi::Error::New(env, "Tile x coordinate must be unsigned interger").ThrowAsJavaScriptException();

        return tile_parameters_ptr();
    }
    if (!y->IsUint32() && !y->IsUndefined())
    {
        Napi::Error::New(env, "Tile y coordinate must be unsigned interger").ThrowAsJavaScriptException();

        return tile_parameters_ptr();
    }
    if (!z->IsUint32() && !z->IsUndefined())
    {
        Napi::Error::New(env, "Tile z coordinate must be unsigned interger").ThrowAsJavaScriptException();

        return tile_parameters_ptr();
    }

    params->x = x.As<Napi::Number>().Uint32Value();
    params->y = y.As<Napi::Number>().Uint32Value();
    params->z = z.As<Napi::Number>().Uint32Value();

    if (!params->IsValid())
    {
        Napi::Error::New(env, "Invalid tile coordinates").ThrowAsJavaScriptException();

        return tile_parameters_ptr();
    }

    return params;
}

inline nearest_parameters_ptr
argumentsToNearestParameter(const Napi::CallbackInfo&args,
                            bool requires_multiple_coordinates)
{
    nearest_parameters_ptr params = std::make_unique<osrm::NearestParameters>();
    bool has_base_params = argumentsToParameter(args, params, requires_multiple_coordinates);
    if (!has_base_params)
        return nearest_parameters_ptr();

    Napi::Object obj = args[0].To<Napi::Object>();
    if (obj.IsEmpty())
        return nearest_parameters_ptr();

    if (obj->Has(Napi::String::New(env, "number")))
    {
        Napi::Value number = obj->Get(Napi::String::New(env, "number"));

        if (!number->IsUint32())
        {
            Napi::Error::New(env, "Number must be an integer greater than or equal to 1").ThrowAsJavaScriptException();

            return nearest_parameters_ptr();
        }
        else
        {
            unsigned number_value = static_cast<unsigned>(number.As<Napi::Number>().DoubleValue());

            if (number_value < 1)
            {
                Napi::Error::New(env, "Number must be an integer greater than or equal to 1").ThrowAsJavaScriptException();

                return nearest_parameters_ptr();
            }

            params->number_of_results = static_cast<unsigned>(number.As<Napi::Number>().DoubleValue());
        }
    }

    return params;
}

inline table_parameters_ptr
argumentsToTableParameter(const Napi::CallbackInfo&args,
                          bool requires_multiple_coordinates)
{
    table_parameters_ptr params = std::make_unique<osrm::TableParameters>();
    bool has_base_params = argumentsToParameter(args, params, requires_multiple_coordinates);
    if (!has_base_params)
        return table_parameters_ptr();

    Napi::Object obj = args[0].To<Napi::Object>();
    if (obj.IsEmpty())
        return table_parameters_ptr();

    if (obj->Has(Napi::String::New(env, "sources")))
    {
        Napi::Value sources = obj->Get(Napi::String::New(env, "sources"));
        if (sources.IsEmpty())
            return table_parameters_ptr();

        if (!sources->IsArray())
        {
            Napi::Error::New(env, "Sources must be an array of indices (or undefined)").ThrowAsJavaScriptException();

            return table_parameters_ptr();
        }

        Napi::Array sources_array = sources.As<Napi::Array>();
        for (uint32_t i = 0; i < sources_array->Length(); ++i)
        {
            Napi::Value source = sources_array->Get(i);
            if (source.IsEmpty())
                return table_parameters_ptr();

            if (source->IsUint32())
            {
                size_t source_value = static_cast<size_t>(source.As<Napi::Number>().DoubleValue());
                if (source_value > params->coordinates.size())
                {
                    Napi::ThrowError(
                        "Source indices must be less than or equal to the number of coordinates");
                    return table_parameters_ptr();
                }

                params->sources.push_back(static_cast<size_t>(source.As<Napi::Number>().DoubleValue()));
            }
            else
            {
                Napi::Error::New(env, "Source must be an integer").ThrowAsJavaScriptException();

                return table_parameters_ptr();
            }
        }
    }

    if (obj->Has(Napi::String::New(env, "destinations")))
    {
        Napi::Value destinations = obj->Get(Napi::String::New(env, "destinations"));
        if (destinations.IsEmpty())
            return table_parameters_ptr();

        if (!destinations->IsArray())
        {
            Napi::Error::New(env, "Destinations must be an array of indices (or undefined)").ThrowAsJavaScriptException();

            return table_parameters_ptr();
        }

        Napi::Array destinations_array = destinations.As<Napi::Array>();
        for (uint32_t i = 0; i < destinations_array->Length(); ++i)
        {
            Napi::Value destination = destinations_array->Get(i);
            if (destination.IsEmpty())
                return table_parameters_ptr();

            if (destination->IsUint32())
            {
                size_t destination_value = static_cast<size_t>(destination.As<Napi::Number>().DoubleValue());
                if (destination_value > params->coordinates.size())
                {
                    Napi::ThrowError("Destination indices must be less than or equal to the number "
                                    "of coordinates");
                    return table_parameters_ptr();
                }

                params->destinations.push_back(static_cast<size_t>(destination.As<Napi::Number>().DoubleValue()));
            }
            else
            {
                Napi::Error::New(env, "Destination must be an integer").ThrowAsJavaScriptException();

                return table_parameters_ptr();
            }
        }
    }

    if (obj->Has(Napi::String::New(env, "annotations")))
    {
        Napi::Value annotations = obj->Get(Napi::String::New(env, "annotations"));
        if (annotations.IsEmpty())
            return table_parameters_ptr();

        if (!annotations->IsArray())
        {
            Napi::ThrowError(
                "Annotations must an array containing 'duration' or 'distance', or both");
            return table_parameters_ptr();
        }

        params->annotations = osrm::TableParameters::AnnotationsType::None;

        Napi::Array annotations_array = annotations.As<Napi::Array>();
        for (std::size_t i = 0; i < annotations_array->Length(); ++i)
        {
            const std::string annotations_utf8str = annotations_array->Get(i.As<Napi::String>());
            std::string annotations_str{*annotations_utf8str,
                                        *annotations_utf8str + annotations_utf8str.Length()};

            if (annotations_str == "duration")
            {
                params->annotations =
                    params->annotations | osrm::TableParameters::AnnotationsType::Duration;
            }
            else if (annotations_str == "distance")
            {
                params->annotations =
                    params->annotations | osrm::TableParameters::AnnotationsType::Distance;
            }
            else
            {
                Napi::Error::New(env, "this 'annotations' param is not supported").ThrowAsJavaScriptException();

                return table_parameters_ptr();
            }
        }
    }

    if (obj->Has(Napi::String::New(env, "fallback_speed")))
    {
        auto fallback_speed = obj->Get(Napi::String::New(env, "fallback_speed"));

        if (!fallback_speed.IsNumber())
        {
            Napi::Error::New(env, "fallback_speed must be a number").ThrowAsJavaScriptException();

            return table_parameters_ptr();
        }
        else if (fallback_speed.As<Napi::Number>().DoubleValue() <= 0)
        {
            Napi::Error::New(env, "fallback_speed must be > 0").ThrowAsJavaScriptException();

            return table_parameters_ptr();
        }

        params->fallback_speed = static_cast<double>(fallback_speed.As<Napi::Number>().DoubleValue());
    }

    if (obj->Has(Napi::String::New(env, "fallback_coordinate")))
    {
        auto fallback_coordinate = obj->Get(Napi::String::New(env, "fallback_coordinate"));

        if (!fallback_coordinate.IsString())
        {
            Napi::Error::New(env, "fallback_coordinate must be a string: [input, snapped]").ThrowAsJavaScriptException();

            return table_parameters_ptr();
        }

        std::string fallback_coordinate_str = *v8::String::Utf8Value(fallback_coordinate);

        if (fallback_coordinate_str == "snapped")
        {
            params->fallback_coordinate_type =
                osrm::TableParameters::FallbackCoordinateType::Snapped;
        }
        else if (fallback_coordinate_str == "input")
        {
            params->fallback_coordinate_type = osrm::TableParameters::FallbackCoordinateType::Input;
        }
        else
        {
            Napi::Error::New(env, "'fallback_coordinate' param must be one of [input, snapped]").ThrowAsJavaScriptException();

            return table_parameters_ptr();
        }
    }

    if (obj->Has(Napi::String::New(env, "scale_factor")))
    {
        auto scale_factor = obj->Get(Napi::String::New(env, "scale_factor"));

        if (!scale_factor.IsNumber())
        {
            Napi::Error::New(env, "scale_factor must be a number").ThrowAsJavaScriptException();

            return table_parameters_ptr();
        }
        else if (scale_factor.As<Napi::Number>().DoubleValue() <= 0)
        {
            Napi::Error::New(env, "scale_factor must be > 0").ThrowAsJavaScriptException();

            return table_parameters_ptr();
        }

        params->scale_factor = static_cast<double>(scale_factor.As<Napi::Number>().DoubleValue());
    }

    return params;
}

inline trip_parameters_ptr
argumentsToTripParameter(const Napi::CallbackInfo&args,
                         bool requires_multiple_coordinates)
{
    trip_parameters_ptr params = std::make_unique<osrm::TripParameters>();
    bool has_base_params = argumentsToParameter(args, params, requires_multiple_coordinates);
    if (!has_base_params)
        return trip_parameters_ptr();

    Napi::Object obj = args[0].To<Napi::Object>();

    bool parsedSuccessfully = parseCommonParameters(obj, params);
    if (!parsedSuccessfully)
    {
        return trip_parameters_ptr();
    }

    if (obj->Has(Napi::String::New(env, "roundtrip")))
    {
        auto roundtrip = obj->Get(Napi::String::New(env, "roundtrip"));
        if (roundtrip.IsEmpty())
            return trip_parameters_ptr();

        if (roundtrip->IsBoolean())
        {
            params->roundtrip = roundtrip.As<Napi::Boolean>().Value();
        }
        else
        {
            Napi::Error::New(env, "'roundtrip' param must be a boolean").ThrowAsJavaScriptException();

            return trip_parameters_ptr();
        }
    }

    if (obj->Has(Napi::String::New(env, "source")))
    {
        Napi::Value source = obj->Get(Napi::String::New(env, "source"));
        if (source.IsEmpty())
            return trip_parameters_ptr();

        if (!source.IsString())
        {
            Napi::Error::New(env, "Source must be a string: [any, first]").ThrowAsJavaScriptException();

            return trip_parameters_ptr();
        }

        std::string source_str = *v8::String::Utf8Value(source);

        if (source_str == "first")
        {
            params->source = osrm::TripParameters::SourceType::First;
        }
        else if (source_str == "any")
        {
            params->source = osrm::TripParameters::SourceType::Any;
        }
        else
        {
            Napi::Error::New(env, "'source' param must be one of [any, first]").ThrowAsJavaScriptException();

            return trip_parameters_ptr();
        }
    }

    if (obj->Has(Napi::String::New(env, "destination")))
    {
        Napi::Value destination = obj->Get(Napi::String::New(env, "destination"));
        if (destination.IsEmpty())
            return trip_parameters_ptr();

        if (!destination.IsString())
        {
            Napi::Error::New(env, "Destination must be a string: [any, last]").ThrowAsJavaScriptException();

            return trip_parameters_ptr();
        }

        std::string destination_str = *v8::String::Utf8Value(destination);

        if (destination_str == "last")
        {
            params->destination = osrm::TripParameters::DestinationType::Last;
        }
        else if (destination_str == "any")
        {
            params->destination = osrm::TripParameters::DestinationType::Any;
        }
        else
        {
            Napi::Error::New(env, "'destination' param must be one of [any, last]").ThrowAsJavaScriptException();

            return trip_parameters_ptr();
        }
    }

    return params;
}

inline match_parameters_ptr
argumentsToMatchParameter(const Napi::CallbackInfo&args,
                          bool requires_multiple_coordinates)
{
    match_parameters_ptr params = std::make_unique<osrm::MatchParameters>();
    bool has_base_params = argumentsToParameter(args, params, requires_multiple_coordinates);
    if (!has_base_params)
        return match_parameters_ptr();

    Napi::Object obj = args[0].To<Napi::Object>();

    if (obj->Has(Napi::String::New(env, "timestamps")))
    {
        Napi::Value timestamps = obj->Get(Napi::String::New(env, "timestamps"));
        if (timestamps.IsEmpty())
            return match_parameters_ptr();

        if (!timestamps->IsArray())
        {
            Napi::Error::New(env, "Timestamps must be an array of integers (or undefined)").ThrowAsJavaScriptException();

            return match_parameters_ptr();
        }

        Napi::Array timestamps_array = timestamps.As<Napi::Array>();

        if (params->coordinates.size() != timestamps_array->Length())
        {
            Napi::ThrowError("Timestamp array must have the same size as the coordinates "
                            "array");
            return match_parameters_ptr();
        }

        for (uint32_t i = 0; i < timestamps_array->Length(); ++i)
        {
            Napi::Value timestamp = timestamps_array->Get(i);
            if (timestamp.IsEmpty())
                return match_parameters_ptr();

            if (!timestamp.IsNumber())
            {
                Napi::Error::New(env, "Timestamps array items must be numbers").ThrowAsJavaScriptException();

                return match_parameters_ptr();
            }
            params->timestamps.emplace_back(static_cast<std::size_t>(timestamp.As<Napi::Number>().DoubleValue()));
        }
    }

    if (obj->Has(Napi::String::New(env, "gaps")))
    {
        Napi::Value gaps = obj->Get(Napi::String::New(env, "gaps"));
        if (gaps.IsEmpty())
            return match_parameters_ptr();

        if (!gaps.IsString())
        {
            Napi::Error::New(env, "Gaps must be a string: [split, ignore]").ThrowAsJavaScriptException();

            return match_parameters_ptr();
        }

        const std::string gaps_utf8str = gaps.As<Napi::String>();
        std::string gaps_str{*gaps_utf8str, *gaps_utf8str + gaps_utf8str.Length()};

        if (gaps_str == "split")
        {
            params->gaps = osrm::MatchParameters::GapsType::Split;
        }
        else if (gaps_str == "ignore")
        {
            params->gaps = osrm::MatchParameters::GapsType::Ignore;
        }
        else
        {
            Napi::Error::New(env, "'gaps' param must be one of [split, ignore]").ThrowAsJavaScriptException();

            return match_parameters_ptr();
        }
    }

    if (obj->Has(Napi::String::New(env, "tidy")))
    {
        Napi::Value tidy = obj->Get(Napi::String::New(env, "tidy"));
        if (tidy.IsEmpty())
            return match_parameters_ptr();

        if (!tidy->IsBoolean())
        {
            Napi::Error::New(env, "tidy must be of type Boolean").ThrowAsJavaScriptException();

            return match_parameters_ptr();
        }

        params->tidy = tidy.As<Napi::Boolean>().Value();
    }

    if (obj->Has(Napi::String::New(env, "waypoints")))
    {
        Napi::Value waypoints = obj->Get(Napi::String::New(env, "waypoints"));
        if (waypoints.IsEmpty())
            return match_parameters_ptr();

        // must be array
        if (!waypoints->IsArray())
        {
            Napi::ThrowError(
                "Waypoints must be an array of integers corresponding to the input coordinates.");
            return match_parameters_ptr();
        }

        auto waypoints_array = waypoints.As<Napi::Array>();
        // must have at least two elements
        if (waypoints_array->Length() < 2)
        {
            Napi::Error::New(env, "At least two waypoints must be provided").ThrowAsJavaScriptException();

            return match_parameters_ptr();
        }
        auto coords_size = params->coordinates.size();
        auto waypoints_array_size = waypoints_array->Length();

        const auto first_index = Napi::To<std::uint32_t>(waypoints_array->Get(0));
        const auto last_index =
            Napi::To<std::uint32_t>(waypoints_array->Get(waypoints_array_size - 1));
        if (first_index != 0 || last_index != coords_size - 1)
        {
            Napi::ThrowError("First and last waypoints values must correspond to first and last "
                            "coordinate indices");
            return match_parameters_ptr();
        }

        for (uint32_t i = 0; i < waypoints_array_size; ++i)
        {
            Napi::Value waypoint_value = waypoints_array->Get(i);
            // all elements must be numbers
            if (!waypoint_value.IsNumber())
            {
                Napi::Error::New(env, "Waypoint values must be an array of integers").ThrowAsJavaScriptException();

                return match_parameters_ptr();
            }
            // check that the waypoint index corresponds with an inpute coordinate
            const auto index = Napi::To<std::uint32_t>(waypoint_value);
            if (index >= coords_size)
            {
                Napi::Error::New(env, "Waypoints must correspond with the index of an input coordinate").ThrowAsJavaScriptException();

                return match_parameters_ptr();
            }
            params->waypoints.emplace_back(static_cast<unsigned>(waypoint_value.As<Napi::Number>().DoubleValue()));
        }
    }

    bool parsedSuccessfully = parseCommonParameters(obj, params);
    if (!parsedSuccessfully)
    {
        return match_parameters_ptr();
    }

    return params;
}

} // namespace node_osrm

#endif
