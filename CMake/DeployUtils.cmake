function(PutLibsInDeployFolder lib_path)
  file(COPY ${lib_path} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/deploy)
endfunction()
