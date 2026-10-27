#pragma once

/// Reader and writer for the `sppg` v1 instance format of spp-graph, the
/// instance file shared by the flowty pricer family (spp-graph's
/// docs/instance-format.md). One file is one shortest-path instance in the
/// ProblemView shape:
///
///   p sppg 1 <n_vertices> <n_arcs> <n_resources> <n_main_resources>
///   t <source> <sink>
///   k <d|n> ...                                    optional
///   domain <general|single_resource|time_indexed>  optional
///   layers                                         optional
///   w <vertex> <lb_0> <ub_0> ...     (missing w = [-inf, +inf])
///   a <from> <to> <cost> <q_0> ...
///   l <vertex> <layer>
///
/// Ids are 0-based; reals are written shortest-round-trip, so a write and a
/// read give back the same bits. The reader checks the grammar, the counts
/// and the ids, and reports the 1-based line of an error (0 when no single
/// line is to blame). It does not check the domain's rules (empty windows,
/// signs, cycles), nor whether bgspprc can solve the instance. The domain tag
/// and the layers are checked and dropped: bgspprc uses neither.

#include "instance_io.h"

#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <expected>
#include <fstream>
#include <limits>
#include <memory>
#include <new>
#include <ostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace bgspprc::io {

namespace sppg_detail {

inline std::string error_at(int line, std::string_view what) {
    return "line " + std::to_string(line) + ": " + std::string(what);
}

/// Fields separated by spaces, tabs, \r, \v and \f.
inline std::vector<std::string_view> split_fields(std::string_view line) {
    constexpr std::string_view kSpace = " \t\r\v\f";
    std::vector<std::string_view> fields;
    std::size_t pos = line.find_first_not_of(kSpace);
    while (pos != std::string_view::npos) {
        const std::size_t end = line.find_first_of(kSpace, pos);
        fields.push_back(line.substr(pos, end == std::string_view::npos ? end : end - pos));
        pos = line.find_first_not_of(kSpace, end);
    }
    return fields;
}

template <typename T>
bool parse_int(std::string_view s, T& out) {
    const char* first = std::to_address(s.begin());
    const char* last = std::to_address(s.end());
    auto [ptr, ec] = std::from_chars(first, last, out);
    return ec == std::errc{} && ptr == last;
}

/// General-format double, `inf` and `-inf` included; NaN is refused.
inline bool parse_real(std::string_view s, double& out) {
    const char* first = std::to_address(s.begin());
    const char* last = std::to_address(s.end());
    auto [ptr, ec] = std::from_chars(first, last, out, std::chars_format::general);
    return ec == std::errc{} && ptr == last && !std::isnan(out);
}

}  // namespace sppg_detail

/// Read an sppg v1 instance. The Instance carries its resources in the
/// general `res_*` arrays, in file order, and an empty name.
inline std::expected<Instance, std::string> read_sppg(std::istream& in) {
    using sppg_detail::error_at;
    using sppg_detail::parse_int;
    using sppg_detail::parse_real;

    Instance inst;
    inst.general_resources = true;

    bool seen_p = false;
    bool seen_t = false;
    bool seen_k = false;
    bool seen_domain = false;
    bool layers = false;
    bool in_body = false;
    int n_arcs = 0;
    int n_res = 0;
    int n_layers = 0;
    std::vector<char> has_window;
    std::vector<char> has_layer;

    std::string line;
    int line_no = 0;
    while (std::getline(in, line)) {
        ++line_no;
        const auto f = sppg_detail::split_fields(line);
        if (f.empty() || f[0] == "c") {
            continue;
        }
        const std::string_view tag = f[0];
        if (!seen_p) {
            if (tag != "p") {
                return std::unexpected(error_at(line_no, "expected the p line first"));
            }
            if (f.size() != 7 || f[1] != "sppg") {
                return std::unexpected(error_at(
                    line_no,
                    "expected p sppg <version> <n_vertices> <n_arcs> <n_resources> <n_main>"));
            }
            int version = 0;
            if (!parse_int(f[2], version)) {
                return std::unexpected(error_at(line_no, "bad version"));
            }
            if (version != 1) {
                return std::unexpected(
                    error_at(line_no, "unsupported sppg version " + std::string(f[2])));
            }
            if (!parse_int(f[3], inst.n_vertices) || !parse_int(f[4], n_arcs) ||
                !parse_int(f[5], n_res) || !parse_int(f[6], inst.n_main_resources) ||
                inst.n_vertices < 0 || n_arcs < 0 || n_res < 0 || inst.n_main_resources < 0) {
                return std::unexpected(error_at(line_no, "counts must be non-negative integers"));
            }
            seen_p = true;
            const auto nv = static_cast<std::size_t>(inst.n_vertices);
            const auto na = static_cast<std::size_t>(n_arcs);
            // The header sizes everything: counts a corrupt file inflates are
            // an error of its p line, not an exception out of the reader.
            try {
                inst.arc_from.reserve(na);
                inst.arc_to.reserve(na);
                inst.arc_cost.reserve(na);
                inst.res_arc.assign(static_cast<std::size_t>(n_res), {});
                for (auto& r : inst.res_arc) {
                    r.reserve(na);
                }
                inst.res_lb.assign(
                    static_cast<std::size_t>(n_res),
                    std::vector<double>(nv, -std::numeric_limits<double>::infinity()));
                inst.res_ub.assign(
                    static_cast<std::size_t>(n_res),
                    std::vector<double>(nv, std::numeric_limits<double>::infinity()));
                has_window.assign(nv, 0);
            } catch (const std::bad_alloc&) {
                return std::unexpected(error_at(line_no, "counts too large to allocate"));
            } catch (const std::length_error&) {
                return std::unexpected(error_at(line_no, "counts too large to allocate"));
            }
            continue;
        }

        auto vertex_id = [&](std::string_view s, int& v) {
            return parse_int(s, v) && v >= 0 && v < inst.n_vertices;
        };

        if (tag == "p" || tag == "t" || tag == "k" || tag == "domain" || tag == "layers") {
            if (in_body) {
                return std::unexpected(
                    error_at(line_no, "header line " + std::string(tag) + " after the body"));
            }
            if (tag == "p") {
                return std::unexpected(error_at(line_no, "second p line"));
            }
            if (tag == "t") {
                if (seen_t) {
                    return std::unexpected(error_at(line_no, "second t line"));
                }
                if (f.size() != 3) {
                    return std::unexpected(error_at(line_no, "expected t <source> <sink>"));
                }
                if (!vertex_id(f[1], inst.source) || !vertex_id(f[2], inst.sink)) {
                    return std::unexpected(error_at(line_no, "source or sink out of range"));
                }
                seen_t = true;
            } else if (tag == "k") {
                if (seen_k) {
                    return std::unexpected(error_at(line_no, "second k line"));
                }
                if (f.size() != static_cast<std::size_t>(n_res) + 1) {
                    return std::unexpected(error_at(line_no, "expected one kind per resource"));
                }
                inst.res_nondisposable = std::make_unique<bool[]>(static_cast<std::size_t>(n_res));
                for (int r = 0; r < n_res; ++r) {
                    const auto kind = f[static_cast<std::size_t>(r) + 1];
                    if (kind != "d" && kind != "n") {
                        return std::unexpected(error_at(line_no, "kind must be d or n"));
                    }
                    inst.res_nondisposable[static_cast<std::size_t>(r)] = kind == "n";
                }
                seen_k = true;
            } else if (tag == "domain") {
                if (seen_domain) {
                    return std::unexpected(error_at(line_no, "second domain line"));
                }
                if (f.size() != 2 ||
                    (f[1] != "general" && f[1] != "single_resource" && f[1] != "time_indexed")) {
                    return std::unexpected(error_at(
                        line_no, "expected domain <general|single_resource|time_indexed>"));
                }
                seen_domain = true;
            } else {
                if (layers) {
                    return std::unexpected(error_at(line_no, "second layers line"));
                }
                if (f.size() != 1) {
                    return std::unexpected(error_at(line_no, "layers takes no fields"));
                }
                layers = true;
                has_layer.assign(static_cast<std::size_t>(inst.n_vertices), 0);
            }
            continue;
        }

        if (tag != "w" && tag != "a" && tag != "l") {
            return std::unexpected(error_at(line_no, "unknown tag " + std::string(tag)));
        }
        if (!in_body && !seen_t) {
            return std::unexpected(error_at(line_no, "missing t line before the body"));
        }
        in_body = true;

        if (tag == "w") {
            if (f.size() != 2 + (2 * static_cast<std::size_t>(n_res))) {
                return std::unexpected(
                    error_at(line_no, "expected w <vertex> and lb ub per resource"));
            }
            int v = 0;
            if (!vertex_id(f[1], v)) {
                return std::unexpected(error_at(line_no, "vertex out of range"));
            }
            if (has_window[static_cast<std::size_t>(v)]) {
                return std::unexpected(error_at(line_no, "second w line for one vertex"));
            }
            has_window[static_cast<std::size_t>(v)] = 1;
            for (int r = 0; r < n_res; ++r) {
                const auto ri = static_cast<std::size_t>(r);
                if (!parse_real(f[2 + (2 * ri)], inst.res_lb[ri][static_cast<std::size_t>(v)]) ||
                    !parse_real(f[3 + (2 * ri)], inst.res_ub[ri][static_cast<std::size_t>(v)])) {
                    return std::unexpected(error_at(line_no, "bad number"));
                }
            }
        } else if (tag == "a") {
            if (f.size() != 4 + static_cast<std::size_t>(n_res)) {
                return std::unexpected(error_at(
                    line_no, "expected a <from> <to> <cost> and a consumption per resource"));
            }
            if (static_cast<int>(inst.arc_from.size()) == n_arcs) {
                return std::unexpected(error_at(line_no, "more a lines than n_arcs"));
            }
            int from = 0;
            int to = 0;
            double cost = 0;
            if (!vertex_id(f[1], from) || !vertex_id(f[2], to)) {
                return std::unexpected(error_at(line_no, "vertex out of range"));
            }
            if (!parse_real(f[3], cost)) {
                return std::unexpected(error_at(line_no, "bad number"));
            }
            inst.arc_from.push_back(from);
            inst.arc_to.push_back(to);
            inst.arc_cost.push_back(cost);
            for (int r = 0; r < n_res; ++r) {
                double q = 0;
                if (!parse_real(f[4 + static_cast<std::size_t>(r)], q)) {
                    return std::unexpected(error_at(line_no, "bad number"));
                }
                inst.res_arc[static_cast<std::size_t>(r)].push_back(q);
            }
        } else {
            if (!layers) {
                return std::unexpected(error_at(line_no, "l line without a layers line"));
            }
            if (f.size() != 3) {
                return std::unexpected(error_at(line_no, "expected l <vertex> <layer>"));
            }
            int v = 0;
            std::int64_t layer = 0;
            if (!vertex_id(f[1], v)) {
                return std::unexpected(error_at(line_no, "vertex out of range"));
            }
            if (!parse_int(f[2], layer)) {
                return std::unexpected(error_at(line_no, "bad layer"));
            }
            if (has_layer[static_cast<std::size_t>(v)]) {
                return std::unexpected(error_at(line_no, "second l line for one vertex"));
            }
            has_layer[static_cast<std::size_t>(v)] = 1;
            ++n_layers;
        }
    }

    if (!seen_p) {
        return std::unexpected(error_at(0, "no p line"));
    }
    if (!seen_t) {
        return std::unexpected(error_at(0, "no t line"));
    }
    if (static_cast<int>(inst.arc_from.size()) != n_arcs) {
        return std::unexpected(error_at(0, "fewer a lines than n_arcs"));
    }
    if (layers && n_layers != inst.n_vertices) {
        return std::unexpected(error_at(0, "layers given but not one l line per vertex"));
    }
    return inst;
}

inline std::expected<Instance, std::string> load_sppg(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        return std::unexpected("cannot open " + path);
    }
    return read_sppg(file);
}

/// Write `pv` as sppg v1. Vertices whose windows are [-inf, +inf] in every
/// resource get no w line.
inline void write_sppg(std::ostream& out, const ProblemView& pv) {
    std::array<char, 32> buf{};
    auto put_real = [&](double x) {
        auto [ptr, ec] = std::to_chars(buf.data(), buf.data() + buf.size(), x);
        out << ' ' << std::string_view(buf.data(), static_cast<std::size_t>(ptr - buf.data()));
    };
    const double inf = std::numeric_limits<double>::infinity();

    out << "p sppg 1 " << pv.n_vertices << ' ' << pv.n_arcs << ' ' << pv.n_resources << ' '
        << pv.n_main_resources << '\n';
    out << "t " << pv.source << ' ' << pv.sink << '\n';
    if (pv.resource_nondisposable != nullptr && pv.n_resources > 0) {
        out << 'k';
        for (int r = 0; r < pv.n_resources; ++r) {
            out << (pv.resource_nondisposable[r] ? " n" : " d");
        }
        out << '\n';
    }
    for (int v = 0; v < pv.n_vertices; ++v) {
        bool unbounded = true;
        for (int r = 0; r < pv.n_resources; ++r) {
            if (pv.vertex_lb[r][v] != -inf || pv.vertex_ub[r][v] != inf) {
                unbounded = false;
                break;
            }
        }
        if (unbounded) {
            continue;
        }
        out << "w " << v;
        for (int r = 0; r < pv.n_resources; ++r) {
            put_real(pv.vertex_lb[r][v]);
            put_real(pv.vertex_ub[r][v]);
        }
        out << '\n';
    }
    for (int a = 0; a < pv.n_arcs; ++a) {
        out << "a " << pv.arc_from[a] << ' ' << pv.arc_to[a];
        put_real(pv.arc_base_cost[a]);
        for (int r = 0; r < pv.n_resources; ++r) {
            put_real(pv.arc_resource[r][a]);
        }
        out << '\n';
    }
}

}  // namespace bgspprc::io
