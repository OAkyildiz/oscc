file(REMOVE_RECURSE
  "libosccapi.pdb"
  "libosccapi.a"
)

# Per-language clean rules from dependency scanning.
foreach(lang C)
  include(CMakeFiles/osccapi_static_lib.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
