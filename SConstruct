#!/usr/bin/env python
"""
SConstruct — GDExtension build script for the PointCloud extension.
Usage:
    scons                                        # editor build, host platform
    scons target=template_debug platform=windows
    scons target=template_release platform=linux arch=x86_64
    scons target=template_debug platform=android arch=arm64
    scons target=template_debug platform=web
"""
import os

# ---- project settings --------------------------------------------------------
LIBNAME = "libpointcloud"
BINDIR  = "addons/point_cloud/bin"  # addons/ lives at repo root

# ---- godot-cpp ---------------------------------------------------------------
# SConscript returns the configured Environment after building godot-cpp.
env = SConscript("godot-cpp/SConstruct")

# ---- laz-perf ----------------------------------------------------------------
LAZPERF_ROOT = "thirdparty/laz-perf/cpp"

# Headers: #include <lazperf/lazperf.hpp>  →  requires lazperf/ on the path
env.Append(CPPPATH=[LAZPERF_ROOT])

# Compile laz-perf as part of our library (vendored, no separate DLL).
# LAZPERF_VENDORED suppresses __declspec(dllexport) decorations on Windows.
env.Append(CPPDEFINES=["LAZPERF_VENDORED"])

lazperf_sources  = Glob("{}/lazperf/*.cpp".format(LAZPERF_ROOT))
lazperf_sources += Glob("{}/lazperf/detail/*.cpp".format(LAZPERF_ROOT))

# Enable standard C++ exception handling — laz-perf throws std::exception on corrupt input.
if env["platform"] == "windows":
    env.Append(CXXFLAGS=["/EHsc"])  # MSVC
elif env["platform"] == "linux":
    env.Append(CXXFLAGS=["-fexceptions"])  # GCC/Clang

# ---- extension sources -------------------------------------------------------
env.Append(CPPPATH=["src/", "."])  # "." lets src/ files include root-level headers
own_sources = Glob("src/*.cpp")

sources = own_sources + lazperf_sources

# ---- output path -------------------------------------------------------------
# env["suffix"]       → e.g. .windows.template_debug.x86_64
# env["SHLIBSUFFIX"]  → .dll / .so / .wasm

if env["platform"] == "macos":
    # macOS uses a .framework bundle
    library = env.SharedLibrary(
        "{}/{}.{}.{}.framework/{}.{}.{}".format(
            BINDIR,
            LIBNAME, env["platform"], env["target"],
            LIBNAME, env["platform"], env["target"],
        ),
        source=sources,
    )
elif env["platform"] == "ios":
    library = env.StaticLibrary(
        "{}/{}{}.a".format(BINDIR, LIBNAME, env["suffix"]),
        source=sources,
    )
else:
    library = env.SharedLibrary(
        "{}/{}{}{}".format(BINDIR, LIBNAME, env["suffix"], env["SHLIBSUFFIX"]),
        source=sources,
    )

env.NoCache(library)
Default(library)
