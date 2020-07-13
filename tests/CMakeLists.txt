macro(test_module TARGET NAME)
  set(MODULE_NAME ${NAME})
  set(TARGET_NAME ${TARGET})
  add_executable(test_${NAME}_module test_module.cpp)
  target_link_libraries(test_${NAME}_module PRIVATE mc_rtc::mc_rbdyn)
  configure_file(test_module.in.h ${CMAKE_CURRENT_BINARY_DIR}/${NAME}/test_module.cmake.h)
  file(GENERATE
       OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${NAME}/$<CONFIGURATION>/test_module.h
       INPUT  ${CMAKE_CURRENT_BINARY_DIR}/${NAME}/test_module.cmake.h)
  target_include_directories(test_${NAME}_module PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/${NAME}/$<CONFIGURATION>)
  add_test(test_${NAME}_module test_${NAME}_module)
endmacro()

test_module(panda PANDA)
