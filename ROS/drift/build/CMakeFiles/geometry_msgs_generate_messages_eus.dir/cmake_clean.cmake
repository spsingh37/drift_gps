file(REMOVE_RECURSE
  "../msg_gen"
  "../src/drift/msg"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/geometry_msgs_generate_messages_eus.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
