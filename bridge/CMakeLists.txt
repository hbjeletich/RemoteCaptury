cmake_minimum_required(VERSION 3.10)

# CMakeLists.txt for the Captury → stdout bridge.
#
# This file lives at: <RemoteCaptury repo>/bridge/CMakeLists.txt
# We add it as a subdirectory in the top-level CMakeLists.txt with:
#   add_subdirectory(bridge)
#
# Builds bridge.exe and links statically against RemoteCapturyStatic so we
# don't need to ship a separate .dll alongside the .exe.

add_executable(bridge bridge.cpp)

# Link against the static library target that's already defined in the
# parent CMakeLists. Linking statically means the .exe is self-contained.
target_link_libraries(bridge PRIVATE RemoteCapturyStatic)

# RemoteCaptury.h includes "captury/PublicStructs.h", so we need to make
# sure the bridge can find both. The parent project sets up include dirs
# for RemoteCapturyStatic, but we may need to add them explicitly here.
target_include_directories(bridge PRIVATE
    ${CMAKE_SOURCE_DIR}
)

# C++17 for std::atomic, structured bindings, etc.
target_compile_features(bridge PRIVATE cxx_std_17)

# On Windows, link sockets library (RemoteCaptury static lib uses winsock).
if(WIN32)
    target_link_libraries(bridge PRIVATE ws2_32)
endif()

# On Linux/Mac, link pthread (callbacks run on a thread).
if(UNIX)
    find_package(Threads REQUIRED)
    target_link_libraries(bridge PRIVATE Threads::Threads)
endif()

