file(REMOVE_RECURSE
  "../msg_gen"
  "../src/drift/msg"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/roscpp_generate_messages_nodejs.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
