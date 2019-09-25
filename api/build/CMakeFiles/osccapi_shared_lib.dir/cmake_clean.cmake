file(REMOVE_RECURSE
  "libosccapi.pdb"
  "libosccapi.so"
)

# Per-language clean rules from dependency scanning.
foreach(lang C)
  include(CMakeFiles/osccapi_shared_lib.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
