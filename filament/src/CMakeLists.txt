gz_get_libsources_and_unittests(sources gtest_sources)

set(engine_name "filament")
gz_add_component(${engine_name} SOURCES ${sources} GET_TARGET_NAME filament_target)

# Filament's public headers need C++20
target_compile_features(${filament_target} PUBLIC cxx_std_20)

# Compile .mat sources into .filamat packages next to the build tree.
# The engine looks them up under FILAMENT_RESOURCE_PATH/materials.
set(FILAMENT_MATC "${FILAMENT_ROOT}/bin/matc")
set(filament_media_dir "${CMAKE_CURRENT_BINARY_DIR}/media")
set(filament_material_dir "${filament_media_dir}/materials")
file(MAKE_DIRECTORY ${filament_material_dir})
file(GLOB filament_mat_sources CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/media/materials/*.mat")
foreach(mat ${filament_mat_sources})
  get_filename_component(mat_name ${mat} NAME_WE)
  set(mat_out "${filament_material_dir}/${mat_name}.filamat")
  add_custom_command(
    OUTPUT ${mat_out}
    COMMAND ${FILAMENT_MATC} --platform=all --api=all --optimize-size
            -o ${mat_out} ${mat}
    DEPENDS ${mat}
    COMMENT "Compiling filament material ${mat_name}")
  list(APPEND filament_materials ${mat_out})
endforeach()
# ogre2's skybox cube map, for Scene::SetSkyEnabled
configure_file(
  "${PROJECT_SOURCE_DIR}/ogre2/src/media/materials/textures/skybox.dds"
  "${filament_media_dir}/textures/skybox.dds" COPYONLY)
add_custom_target(filament_materials DEPENDS ${filament_materials})
add_dependencies(${filament_target} filament_materials)

set_property(
  SOURCE FilamentRenderEngine.cc
  PROPERTY COMPILE_DEFINITIONS
  FILAMENT_RESOURCE_PATH="${filament_media_dir}"
)

target_include_directories(${filament_target}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
  SYSTEM PUBLIC ${FILAMENT_ROOT}/include)

# Filament ships static archives only; link them all as one group so the
# linker resolves their mutual dependencies.
file(GLOB filament_archives "${FILAMENT_ROOT}/lib/x86_64/*.a")
target_link_libraries(${filament_target}
  PUBLIC
    ${gz-common_LIBRARIES}
    gz-math::eigen3
  PRIVATE
    gz-plugin::register
    -Wl,--start-group ${filament_archives} -Wl,--end-group
    ${CMAKE_DL_LIBS}
    ${OPENGL_LIBRARIES}
    pthread)

if (TARGET OpenGL::EGL)
  target_link_libraries(${filament_target} PRIVATE OpenGL::EGL)
endif()

install(TARGETS ${filament_target} DESTINATION ${GZ_RENDERING_ENGINE_RELATIVE_INSTALL_DIR})
install(DIRECTORY "${filament_media_dir}" DESTINATION ${GZ_RENDERING_RELATIVE_RESOURCE_PATH}/filament)

gz_build_tests(TYPE UNIT
               SOURCES ${gtest_sources}
               LIB_DEPS ${filament_target})

# Smoke test tool: renders the demo scene with any engine, saves rgb/depth
add_executable(filament_smoke ${CMAKE_CURRENT_SOURCE_DIR}/../tools/filament_smoke.cc)
target_link_libraries(filament_smoke
  PRIVATE
    ${PROJECT_LIBRARY_TARGET_NAME}
    gz-plugin::loader
    gz-common::gz-common
    gz-common::graphics)

# Sensor parity dump: raw depth/point cloud/lidar outputs + timings per engine
add_executable(filament_parity ${CMAKE_CURRENT_SOURCE_DIR}/../tools/filament_parity.cc)
target_link_libraries(filament_parity
  PRIVATE
    ${PROJECT_LIBRARY_TARGET_NAME}
    gz-common::gz-common)
target_compile_definitions(filament_parity
  PRIVATE PROJECT_SOURCE_PATH="${PROJECT_SOURCE_DIR}")
